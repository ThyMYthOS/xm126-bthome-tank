// SPDX-License-Identifier: BSD-3-Clause

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#include <zephyr/bluetooth/att.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>

#include <zephyr/settings/settings.h>

#include "acc_tank_config_ble.h"

/* Sane bounds for a vertical cylindrical tank; matches the distance detector's
 * supported range (see xm126_bthome_tank.c presets).
 */
#define TANK_DIAMETER_MIN_MM     (50U)
#define TANK_DIAMETER_MAX_MM     (5000U)
#define TANK_FLOOR_OFFSET_MIN_MM (50U)
#define TANK_FLOOR_OFFSET_MAX_MM (15000U)

/* Custom vendor UUID base for this service, generated once for this project:
 * c9c9b3ec-1cee-4f9e-b62c-9d8f6a1b6f00. Characteristics reuse the base with the last
 * byte changed. Do not reuse this UUID for an unrelated service.
 */
#define BT_UUID_TANK_CONFIG_SERVICE_VAL BT_UUID_128_ENCODE(0xc9c9b3ec, 0x1cee, 0x4f9e, 0xb62c, 0x9d8f6a1b6f00)
#define BT_UUID_TANK_CONFIG_DIAMETER_VAL BT_UUID_128_ENCODE(0xc9c9b3ec, 0x1cee, 0x4f9e, 0xb62c, 0x9d8f6a1b6f01)
#define BT_UUID_TANK_CONFIG_FLOOR_OFFSET_VAL BT_UUID_128_ENCODE(0xc9c9b3ec, 0x1cee, 0x4f9e, 0xb62c, 0x9d8f6a1b6f02)

static struct bt_uuid_128 tank_config_service_uuid  = BT_UUID_INIT_128(BT_UUID_TANK_CONFIG_SERVICE_VAL);
static struct bt_uuid_128 tank_diameter_uuid         = BT_UUID_INIT_128(BT_UUID_TANK_CONFIG_DIAMETER_VAL);
static struct bt_uuid_128 tank_floor_offset_uuid     = BT_UUID_INIT_128(BT_UUID_TANK_CONFIG_FLOOR_OFFSET_VAL);

static uint32_t tank_diameter_mm;
static uint32_t tank_floor_offset_mm;
static atomic_t config_dirty;

static ssize_t read_u32(struct bt_conn *conn, const struct bt_gatt_attr *attr, void *buf, uint16_t len, uint16_t offset)
{
	const uint32_t *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t write_bounded_u32(const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset, uint32_t min,
                                  uint32_t max, uint32_t *storage, const char *settings_key)
{
	if (offset != 0)
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	if (len != sizeof(uint32_t))
	{
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}

	uint32_t value;

	memcpy(&value, buf, sizeof(value));

	if (value < min || value > max)
	{
		return BT_GATT_ERR(BT_ATT_ERR_VALUE_NOT_ALLOWED);
	}

	*storage = value;
	atomic_set(&config_dirty, 1);

	int err = settings_save_one(settings_key, storage, sizeof(*storage));

	if (err)
	{
		printk("Failed to persist %s (err %d)\n", settings_key, err);
	}

	return len;
}

static ssize_t write_diameter(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset,
                               uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(flags);

	return write_bounded_u32(attr, buf, len, offset, TANK_DIAMETER_MIN_MM, TANK_DIAMETER_MAX_MM, &tank_diameter_mm, "tank/diam");
}

static ssize_t write_floor_offset(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf, uint16_t len, uint16_t offset,
                                   uint8_t flags)
{
	ARG_UNUSED(conn);
	ARG_UNUSED(flags);

	return write_bounded_u32(attr, buf, len, offset, TANK_FLOOR_OFFSET_MIN_MM, TANK_FLOOR_OFFSET_MAX_MM, &tank_floor_offset_mm,
	                          "tank/floor");
}

BT_GATT_SERVICE_DEFINE(tank_config_svc, BT_GATT_PRIMARY_SERVICE(&tank_config_service_uuid),
                       BT_GATT_CHARACTERISTIC(&tank_diameter_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                               BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_u32, write_diameter, &tank_diameter_mm),
                       BT_GATT_CHARACTERISTIC(&tank_floor_offset_uuid.uuid, BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE,
                                               BT_GATT_PERM_READ | BT_GATT_PERM_WRITE, read_u32, write_floor_offset,
                                               &tank_floor_offset_mm));

/* Loaded values go through the same bounds as a BLE write. storage_partition is never
 * erased by a mcumgr/serial image upload (that only touches the image slots), so on a
 * board that shipped pre-flashed with different firmware, this region can still hold
 * whatever that firmware left behind -- garbage here should fall back to the compiled
 * default rather than being trusted blindly, and the bad value is overwritten so this
 * self-heals after one boot instead of persisting forever.
 */
static int load_bounded_u32(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg, uint32_t min, uint32_t max,
                             uint32_t *storage, const char *settings_key)
{
	if (len != sizeof(*storage))
	{
		return -EINVAL;
	}

	uint32_t value;

	if (read_cb(cb_arg, &value, sizeof(value)) < 0)
	{
		return -EINVAL;
	}

	if (value < min || value > max)
	{
		printk("Ignoring out-of-range persisted %s=%u (valid %u..%u), keeping default %u\n", settings_key, value, min, max,
		       *storage);
		(void)settings_save_one(settings_key, storage, sizeof(*storage));
		return 0;
	}

	*storage = value;

	return 0;
}

static int tank_config_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg)
{
	const char *next;

	if (settings_name_steq(name, "diam", &next) && !next)
	{
		return load_bounded_u32(name, len, read_cb, cb_arg, TANK_DIAMETER_MIN_MM, TANK_DIAMETER_MAX_MM, &tank_diameter_mm,
		                         "tank/diam");
	}

	if (settings_name_steq(name, "floor", &next) && !next)
	{
		return load_bounded_u32(name, len, read_cb, cb_arg, TANK_FLOOR_OFFSET_MIN_MM, TANK_FLOOR_OFFSET_MAX_MM,
		                         &tank_floor_offset_mm, "tank/floor");
	}

	return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(tank_config, "tank", NULL, tank_config_settings_set, NULL, NULL);

uint32_t acc_tank_config_ble_get_diameter_mm(void)
{
	return tank_diameter_mm;
}

uint32_t acc_tank_config_ble_get_floor_offset_mm(void)
{
	return tank_floor_offset_mm;
}

bool acc_tank_config_ble_consume_dirty_flag(void)
{
	return atomic_set(&config_dirty, 0) != 0;
}

void acc_tank_config_ble_reset_to_defaults(void)
{
	(void)settings_delete("tank/diam");
	(void)settings_delete("tank/floor");
}

void acc_tank_config_ble_init(uint32_t default_diameter_mm, uint32_t default_floor_offset_mm)
{
	tank_diameter_mm     = default_diameter_mm;
	tank_floor_offset_mm = default_floor_offset_mm;

	int err = settings_subsys_init();

	if (err)
	{
		printk("settings_subsys_init() failed (err %d)\n", err);
		return;
	}

	err = settings_load();
	if (err)
	{
		printk("settings_load() failed (err %d)\n", err);
	}

	printk("Tank config in use: diameter=%u mm, floor_offset=%u mm\n", tank_diameter_mm, tank_floor_offset_mm);
}
