# Tank Configuration BLE GATT Service

Reference documentation for the custom GATT service used to configure tank geometry
(diameter and sensor mounting offset) on the `xm126_bthome_tank` firmware.
Implementation: `app/acc_tank_config_ble.c`, `app/acc_tank_config_ble.h`.

## Overview

- **Purpose**: lets a BLE central set the tank's diameter and the sensor's mounting
  height above the tank floor, so the firmware can convert its raw distance
  measurement into volume. Without configuration, compiled-in defaults are used
  (see [Defaults](#defaults)).
- **Where it lives**: registered in the same GATT database as the mcumgr SMP
  firmware-update service. Both are reachable under the same conditions.
- **Reachability**: only when the periodic connectable advertising window is open —
  every `DFU_WINDOW_PERIOD_S` = 300 s, held open for `DFU_WINDOW_DURATION_S` = 30 s
  (constants in `app/acc_bthome_ble.c`), plus one window right after boot. Outside
  that window the device is only broadcasting (non-connectable BTHome), and this
  service cannot be reached.
- **Discovery**: the device advertises as `CONFIG_BT_DEVICE_NAME` = `"XM126 Tank"`
  (standard connectable legacy advertising — visible to any BLE central/scanner, no
  special app required).
- **Security**: plain read/write permissions, no pairing, bonding, encryption, or
  authentication required. Anyone who can connect during the window can read or write
  these values — see [Security note](#security-note).

## Service and characteristic UUIDs

Custom 128-bit vendor UUIDs, generated once for this project (base
`c9c9b3ec-1cee-4f9e-b62c-9d8f6a1b6f00`, characteristics reuse the base with the last
byte changed). Do not reuse this UUID range for an unrelated service.

| Name | UUID | Attribute type |
|---|---|---|
| Tank Configuration Service | `c9c9b3ec-1cee-4f9e-b62c-9d8f6a1b6f00` | Primary Service |
| Diameter | `c9c9b3ec-1cee-4f9e-b62c-9d8f6a1b6f01` | Characteristic (Read, Write) |
| Floor Offset | `c9c9b3ec-1cee-4f9e-b62c-9d8f6a1b6f02` | Characteristic (Read, Write) |

## Characteristics

Both characteristics use the same wire format: a **4-byte little-endian `uint32`, in
millimeters**, read/written at offset 0 only.

### Diameter — `...6f01`

| | |
|---|---|
| Meaning | Tank inner diameter. Used with the measured level to compute volume: `volume = level × π × (diameter / 2)²` (vertical cylindrical tank). |
| Unit | millimeters |
| Valid range | 50 – 5000 |
| Properties | Read, Write (Write Request — expects an ATT response, not Write Without Response) |
| Permissions | Read, Write (no encryption/authentication) |
| Settings key | `tank/diam` |
| Default if never configured | 1000 mm |

### Floor Offset — `...6f02`

| | |
|---|---|
| Meaning | Distance from the sensor (mounted in the tank lid) down to the tank floor when empty — the detector's maximum range / the "0 % full" reference distance. Equivalent to the algorithm's internal `tank_range_end_m`. |
| Unit | millimeters |
| Valid range | 50 – 15000 |
| Properties | Read, Write (Write Request) |
| Permissions | Read, Write (no encryption/authentication) |
| Settings key | `tank/floor` |
| Default if never configured | 500 mm (matches the `SMALL_TANK` preset's range end) |

## Defaults

Defaults are only used the very first time the device boots with nothing yet persisted
in flash; any successful write permanently overrides them (see [Persistence](#persistence-and-side-effects)).
Compiled-in defaults live in `app/xm126_bthome_tank.c`:

```c
#define DEFAULT_FLOOR_OFFSET_MM  ((uint32_t)(SMALL_TANK_RANGE_END_M * 1000.0f))  // 500 mm
#define DEFAULT_TANK_DIAMETER_MM (1000U)                                        // 1000 mm
```

## Write validation and ATT error codes

| Condition | ATT error | Code |
|---|---|---|
| Write offset ≠ 0 | Invalid Offset | `0x07` |
| Write length ≠ 4 bytes | Invalid Attribute Length | `0x0D` |
| Value outside the characteristic's valid range | Value Not Allowed | `0x13` |

On any error, the stored value is left unchanged — nothing is applied or persisted.

## Persistence and side effects

On a successful write:

1. The new value is stored in RAM immediately (a subsequent Read returns it right away).
2. It's persisted to flash via Zephyr's `settings` subsystem (NVS backend on the
   `storage_partition` flash region) *before* the ATT Write Response is sent, so it
   survives reboots and firmware updates.
3. An internal dirty flag is set.
4. On the **next measurement cycle**, the app notices the dirty flag, destroys and
   recreates the distance detector using the new floor offset, and recalibrates.
   This takes roughly one calibration cycle (well under a second) before fresh
   in-range measurements resume with the new geometry. A diameter-only change doesn't
   require detector reconfiguration, but currently triggers the same recalibration path
   as a floor-offset change (no functional harm, just a brief unnecessary recalibration).

On **load** (every boot), a value outside the valid range is rejected and the flash
entry is immediately corrected back to the compiled-in default -- this guards against
flash corruption, but not against a stale value that happens to fall inside the valid
range (see the note below).

### Recovering from a bad persisted value

`mcumgr image upload` (over UART or BLE) never touches `storage_partition` -- only the
image slots. A board that shipped with different firmware pre-flashed can have that
firmware's leftover data in the same flash region the very first time this app boots,
and if it happens to look like a plausible (in-range) diameter/floor-offset, the
above load-time validation won't catch it. Symptom: wrong-looking volume/level from the
very first boot, without ever having written the characteristics yourself. Fix: type
`resetcfg` followed by Enter on the console UART (see the main `README.md`) to erase
both persisted values and reboot with the compiled-in defaults, or overwrite them with
a deliberate BLE write of your own.

## Reaching the service

1. Wait for (or trigger, e.g. via a fresh boot) the periodic connectable window.
2. Connect to the device advertising as `"XM126 Tank"`.
3. Discover the `...6f00` service and its two characteristics.
4. **Read**: standard GATT Read Request → 4-byte little-endian value back.
5. **Write**: standard GATT Write Request with a 4-byte little-endian value → wait for
   the Write Response (success, or one of the errors above).

Works with any generic BLE tool — e.g. nRF Connect for Mobile's plain attribute
read/write screen — no vendor-specific app needed. The same connection can also be used
to push a firmware update via mcumgr (see the main `README.md`).

## Example encodings

| Value | Bytes (little-endian) |
|---|---|
| Diameter = 1200 mm | `B0 04 00 00` |
| Floor offset = 900 mm | `84 03 00 00` |
| Floor offset = 15000 mm (max) | `98 3A 00 00` |

## Security note

This service currently has no access control beyond physical proximity and the
narrow connectable window: no pairing, bonding, encryption, or authentication is
required to read or write it. That's an intentional simplicity trade-off for this
reference application — anyone able to connect during the ~30 s window can
reconfigure the tank geometry. If this matters for your deployment, the natural
hardening path is switching the characteristics' permissions to
`BT_GATT_PERM_READ_ENCRYPT | BT_GATT_PERM_WRITE_ENCRYPT` (or the `_AUTHEN` variants)
and enabling `CONFIG_BT_SMP`, forcing pairing before access.
