// SPDX-License-Identifier: BSD-3-Clause

#ifndef ACC_BTHOME_BLE_H_
#define ACC_BTHOME_BLE_H_

#include <stdbool.h>
#include <stdint.h>

/**@brief Initialize BLE.
 *
 * Starts the always-on, non-connectable BTHome v2 broadcaster (picked up natively by
 * Home Assistant's Bluetooth integration) and arms a periodic connectable advertising
 * window used for BLE firmware updates (mcumgr SMP) and tank configuration (see
 * acc_tank_config_ble.h). No physical button is needed to reach the device; it opens
 * the window on its own schedule.
 */
void acc_bthome_ble_init(void);

/**@brief Publish a fresh tank measurement as a BTHome v2 broadcast.
 *
 * @param[in] distance_mm         Raw sensor-to-surface distance, in mm.
 * @param[in] volume_milliliters  Computed tank volume, in mL.
 * @param[in] valid               True if the measurement is in range and the
 *                                distance/volume objects should be published; false to
 *                                advertise only a fresh packet id (no distance/volume
 *                                objects) when the level is out of range.
 * @param[in] battery_percent     Remaining battery capacity, 0-100 %. Always
 *                                published, independent of `valid`.
 * @param[in] battery_voltage_mv  Raw supply voltage, in mV. Always published,
 *                                independent of `valid`.
 * @param[in] temperature_centi_c MCU die temperature, in units of 0.01 degC (i.e. the
 *                                actual temperature * 100, matching the BTHome wire
 *                                format directly). Always published, independent of
 *                                `valid`.
 */
void acc_bthome_ble_update(uint16_t distance_mm, uint32_t volume_milliliters, bool valid, uint8_t battery_percent,
                           uint16_t battery_voltage_mv, int16_t temperature_centi_c);

#endif
