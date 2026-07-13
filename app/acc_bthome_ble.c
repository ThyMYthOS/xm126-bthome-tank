// SPDX-License-Identifier: BSD-3-Clause

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/hci.h>

#include "acc_bthome_ble.h"

/* How often a connectable window for mcumgr BLE DFU / tank config is opened, and how
 * long it stays open if nobody connects. Edit to match the deployment's needs -- a
 * shorter period gives faster access for updates/config at the cost of slightly higher
 * average power draw from the extra connectable advertising bursts.
 */
#define DFU_WINDOW_PERIOD_S   (300U)
#define DFU_WINDOW_DURATION_S (30U)

/* Advertising interval for the BTHome broadcast, in 0.625 ms units. */
#define BTHOME_ADV_MIN_INTERVAL (0x00A0) /* 100 ms */
#define BTHOME_ADV_MAX_INTERVAL (0x00F0) /* 150 ms */

/* How long each update repeats the advertisement for before going silent again (until
 * the next update). A single near-instantaneous event (the previous approach) is easy
 * for a scanner to miss entirely, especially now that updates can be many minutes
 * apart -- a multi-second burst at a fast interval gives it several chances to catch
 * at least one packet, while still being a small fraction of the time between updates.
 */
#define BTHOME_ADV_BURST_MS (5U * 1000U)

/* BTHome v2 (https://bthome.io/format) service data, UUID 0xFCD2 */
#define BTHOME_SVC_UUID_LO                 (0xD2)
#define BTHOME_SVC_UUID_HI                  (0xFC)
#define BTHOME_DEVICE_INFO_V2_UNENCRYPTED   (0x40)
#define BTHOME_OBJ_PACKET_ID                (0x00) /* uint8, factor 1 */
#define BTHOME_OBJ_BATTERY_PCT              (0x01) /* uint8, factor 1, % */
#define BTHOME_OBJ_TEMPERATURE_C            (0x02) /* sint16, factor 0.01 -> raw value == 0.01 degC */
#define BTHOME_OBJ_VOLTAGE_V                (0x0C) /* uint16, factor 0.001 -> raw value == mV */
#define BTHOME_OBJ_DISTANCE_MM              (0x40) /* uint16, factor 1 */
#define BTHOME_OBJ_VOLUME_STORAGE_L         (0x55) /* uint32, factor 0.001 L -> raw value == mL */

/* UUID(2) + device info(1) + packet id(2) + battery(2) + temperature(3) + voltage(3) +
 * distance(3) + volume(5), the last two only present while the level is valid.
 *
 * Worst case (21) + its 2-byte AD header (23) + the short-name AD structure below
 * (2-byte header + 5 chars = 7) = 30 bytes, against a 31-byte legacy advertising PDU
 * budget -- 1 byte of slack. Adding another object here will need to shrink
 * BTHOME_ADV_SHORT_NAME (or drop it) to still fit.
 */
#define BTHOME_PAYLOAD_MAX_LEN (21U)

/* Short enough to leave room for the (larger, worst-case) payload above within the
 * 31-byte legacy advertising PDU budget -- the DFU/config connectable set still
 * advertises the full CONFIG_BT_DEVICE_NAME, since it doesn't share this budget.
 */
#define BTHOME_ADV_SHORT_NAME "XM126"

static struct bt_le_ext_adv *adv_bthome;
static struct bt_le_ext_adv *adv_dfu;

static uint8_t packet_id;
static bool    dfu_connected;

static struct k_timer open_window_timer;
static struct k_timer close_window_timer;
static struct k_work  open_window_work;
static struct k_work  close_window_work;

/* BT_LE_ADV_OPT_USE_IDENTITY on both sets pins them to the device's fixed identity
 * address. Without it, Zephyr hands out a fresh private address on every single
 * bt_le_ext_adv_start() call (see subsys/bluetooth/host/adv.c) -- and the BTHome set
 * below does a full stop/restart every measurement (~1/s), so it was effectively
 * broadcasting a new MAC address almost every packet. Home Assistant's BTHome
 * integration keys entities by MAC, so that looked like a constant stream of brand new
 * devices instead of one. No BT_LE_ADV_OPT_EXT_ADV on either set: this keeps both as
 * legacy PDUs (ADV_NONCONN_IND / ADV_IND) so that BlueZ's default passive scan -- what
 * Home Assistant's Bluetooth integration uses -- can see them without extended
 * scanning.
 */
static const struct bt_le_adv_param bthome_adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_USE_IDENTITY, BTHOME_ADV_MIN_INTERVAL, BTHOME_ADV_MAX_INTERVAL, NULL);

static const struct bt_le_adv_param dfu_adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN | BT_LE_ADV_OPT_USE_IDENTITY, BT_GAP_ADV_FAST_INT_MIN_2, BT_GAP_ADV_FAST_INT_MAX_2, NULL);

static const struct bt_data dfu_ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void open_window_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	int err = bt_le_ext_adv_start(adv_dfu, BT_LE_EXT_ADV_START_DEFAULT);

	if (err)
	{
		printk("Failed to open DFU/config advertising window (err %d)\n", err);
	}

	k_timer_start(&close_window_timer, K_SECONDS(DFU_WINDOW_DURATION_S), K_NO_WAIT);
}

static void close_window_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Safe no-op if a connection already stopped this advertising set on its own. */
	(void)bt_le_ext_adv_stop(adv_dfu);

	if (!dfu_connected)
	{
		k_timer_start(&open_window_timer, K_SECONDS(DFU_WINDOW_PERIOD_S), K_NO_WAIT);
	}
}

static void open_window_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&open_window_work);
}

static void close_window_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&close_window_work);
}

static void connected(struct bt_conn *conn, uint8_t err)
{
	ARG_UNUSED(conn);

	if (!err)
	{
		dfu_connected = true;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(reason);

	dfu_connected = false;

	/* Deferred (not immediate) by design: the next window opens one full period from
	 * now, not right away, to keep the connectable radio time bounded.
	 */
	k_timer_start(&open_window_timer, K_SECONDS(DFU_WINDOW_PERIOD_S), K_NO_WAIT);
}

BT_CONN_CB_DEFINE(acc_tank_conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

static uint8_t build_bthome_payload(uint8_t *buf, uint16_t distance_mm, uint32_t volume_milliliters, bool valid, uint8_t id,
                                     uint8_t battery_percent, uint16_t battery_voltage_mv, int16_t temperature_centi_c)
{
	uint8_t  len = 0;
	uint16_t temperature_raw = (uint16_t)temperature_centi_c; /* two's complement LE, same as unsigned */

	buf[len++] = BTHOME_SVC_UUID_LO;
	buf[len++] = BTHOME_SVC_UUID_HI;
	buf[len++] = BTHOME_DEVICE_INFO_V2_UNENCRYPTED;

	buf[len++] = BTHOME_OBJ_PACKET_ID;
	buf[len++] = id;

	buf[len++] = BTHOME_OBJ_BATTERY_PCT;
	buf[len++] = battery_percent;

	buf[len++] = BTHOME_OBJ_TEMPERATURE_C;
	buf[len++] = temperature_raw & 0xFF;
	buf[len++] = (temperature_raw >> 8) & 0xFF;

	buf[len++] = BTHOME_OBJ_VOLTAGE_V;
	buf[len++] = battery_voltage_mv & 0xFF;
	buf[len++] = (battery_voltage_mv >> 8) & 0xFF;

	if (valid)
	{
		buf[len++] = BTHOME_OBJ_DISTANCE_MM;
		buf[len++] = distance_mm & 0xFF;
		buf[len++] = (distance_mm >> 8) & 0xFF;

		buf[len++] = BTHOME_OBJ_VOLUME_STORAGE_L;
		buf[len++] = volume_milliliters & 0xFF;
		buf[len++] = (volume_milliliters >> 8) & 0xFF;
		buf[len++] = (volume_milliliters >> 16) & 0xFF;
		buf[len++] = (volume_milliliters >> 24) & 0xFF;
	}

	return len;
}

void acc_bthome_ble_update(uint16_t distance_mm, uint32_t volume_milliliters, bool valid, uint8_t battery_percent,
                           uint16_t battery_voltage_mv, int16_t temperature_centi_c)
{
	uint8_t payload[BTHOME_PAYLOAD_MAX_LEN];
	uint8_t payload_len = build_bthome_payload(payload, distance_mm, volume_milliliters, valid, packet_id++, battery_percent,
	                                            battery_voltage_mv, temperature_centi_c);

	/* Shortened name, not CONFIG_BT_DEVICE_NAME -- the full name plus this (larger,
	 * battery-carrying) payload would exceed the 31-byte legacy advertising PDU
	 * budget. The DFU/config connectable set is a separate advertising set with its
	 * own budget and still uses the full name.
	 */
	const struct bt_data ad[] = {
	    BT_DATA(BT_DATA_SVC_DATA16, payload, payload_len),
	    BT_DATA(BT_DATA_NAME_SHORTENED, BTHOME_ADV_SHORT_NAME, sizeof(BTHOME_ADV_SHORT_NAME) - 1),
	};

	/* Stop (safe no-op if not running), refresh the data, then start a fresh
	 * BTHOME_ADV_BURST_MS burst -- re-armed each time the caller has a real update
	 * (see volume_broadcast_needed() in the app), which can be minutes apart now that
	 * radar sampling and BLE broadcasting are decoupled.
	 */
	int err = bt_le_ext_adv_stop(adv_bthome);

	if (err && err != -EALREADY)
	{
		printk("Failed to stop BTHome advertising (err %d)\n", err);
	}

	err = bt_le_ext_adv_set_data(adv_bthome, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err)
	{
		printk("Failed to set BTHome advertising data (err %d)\n", err);
		return;
	}

	struct bt_le_ext_adv_start_param start_param = {
	    .timeout    = BTHOME_ADV_BURST_MS / 10U, /* param is in N * 10 ms units */
	    .num_events = 0,
	};

	err = bt_le_ext_adv_start(adv_bthome, &start_param);
	if (err)
	{
		printk("Failed to start BTHome advertising (err %d)\n", err);
	}
}

void acc_bthome_ble_init(void)
{
	int err = bt_enable(NULL);

	if (err)
	{
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_create(&bthome_adv_param, NULL, &adv_bthome);
	if (err)
	{
		printk("Failed to create BTHome advertising set (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_create(&dfu_adv_param, NULL, &adv_dfu);
	if (err)
	{
		printk("Failed to create DFU/config advertising set (err %d)\n", err);
		return;
	}

	err = bt_le_ext_adv_set_data(adv_dfu, dfu_ad, ARRAY_SIZE(dfu_ad), NULL, 0);
	if (err)
	{
		printk("Failed to set DFU/config advertising data (err %d)\n", err);
		return;
	}

	k_work_init(&open_window_work, open_window_work_handler);
	k_work_init(&close_window_work, close_window_work_handler);
	k_timer_init(&open_window_timer, open_window_timer_handler, NULL);
	k_timer_init(&close_window_timer, close_window_timer_handler, NULL);

	/* Open the first DFU/config window right away at boot; subsequent windows follow
	 * the periodic schedule above.
	 */
	k_timer_start(&open_window_timer, K_NO_WAIT, K_NO_WAIT);

	printk("BTHome broadcaster started, DFU/config window every %u s for %u s\n", DFU_WINDOW_PERIOD_S, DFU_WINDOW_DURATION_S);
}
