// SPDX-License-Identifier: BSD-3-Clause

#ifndef ACC_SERIAL_CONSOLE_COMMANDS_H_
#define ACC_SERIAL_CONSOLE_COMMANDS_H_

/**@brief Watch the console UART for typed commands, each followed by Enter:
 *
 * - "dfu"      -- reboot into MCUboot serial recovery (DFU) mode, no button needed.
 *                 Sets the same retained-memory boot-mode flag the physical DFU button
 *                 sets, so from MCUboot's point of view it is indistinguishable from a
 *                 button-triggered entry.
 * - "resetcfg" -- erase the persisted tank configuration (diameter / floor offset,
 *                 see acc_tank_config_ble.h) and reboot back into the app, so the
 *                 compiled-in defaults apply again. Useful if the persisted values are
 *                 wrong -- e.g. a board that shipped with different firmware
 *                 pre-flashed can have unrelated leftover data in the same flash
 *                 region, since a mcumgr/serial image upload only ever touches the
 *                 image slots, never storage_partition.
 */
void acc_serial_console_commands_init(void);

#endif
