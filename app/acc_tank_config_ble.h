// SPDX-License-Identifier: BSD-3-Clause

#ifndef ACC_TANK_CONFIG_BLE_H_
#define ACC_TANK_CONFIG_BLE_H_

#include <stdbool.h>
#include <stdint.h>

/**@brief Initialize the tank-configuration GATT service and load persisted values.
 *
 * Registers a small custom GATT service (read/write "diameter_mm" and
 * "floor_offset_mm" characteristics, both little-endian uint32) in the same GATT
 * database as the mcumgr SMP service, reachable whenever the periodic connectable
 * window (see acc_bthome_ble.h) is open. Any generic BLE tool (e.g. nRF Connect's
 * attribute screen) can read/write these without app-specific tooling.
 *
 * Loads persisted values from flash (Zephyr settings/NVS on storage_partition) if
 * previously configured, otherwise falls back to the given defaults. Must be called
 * before the first read of the getters below, and before BLE is enabled.
 *
 * @param[in] default_diameter_mm     Fallback tank diameter, in mm, if never configured.
 * @param[in] default_floor_offset_mm Fallback sensor-to-floor mounting distance, in mm,
 *                                    if never configured.
 */
void acc_tank_config_ble_init(uint32_t default_diameter_mm, uint32_t default_floor_offset_mm);

/**@brief Get the current tank diameter, in mm (persisted value, or the default). */
uint32_t acc_tank_config_ble_get_diameter_mm(void);

/**@brief Get the current sensor-to-floor mounting distance, in mm (persisted value, or
 * the default).
 */
uint32_t acc_tank_config_ble_get_floor_offset_mm(void);

/**@brief Check and clear the "configuration changed over BLE" flag.
 *
 * @return True if either characteristic was written since the last call, meaning the
 *         caller should rebuild its distance-detector configuration and recalibrate.
 */
bool acc_tank_config_ble_consume_dirty_flag(void);

/**@brief Erase the persisted diameter and floor-offset values from flash.
 *
 * Takes effect on the next boot (the compiled-in defaults passed to
 * acc_tank_config_ble_init() apply again) -- this does not change the values
 * currently in RAM, so a reboot is required, e.g. via the "resetcfg" serial console
 * command (see acc_serial_console_commands.h).
 */
void acc_tank_config_ble_reset_to_defaults(void);

#endif
