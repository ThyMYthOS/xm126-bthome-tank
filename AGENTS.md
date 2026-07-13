# AGENTS.md

Context for AI agents (and future-you) working on this repo. This is a single Zephyr/nRF
Connect SDK application (`app/`), extracted from Acconeer's much larger XM126 SDK, with
the upstream Acconeer SDK content kept separately under `acconeer/`. Everything here was
learned the hard way while building and iterating on this firmware against real
hardware -- treat the "gotchas" sections as things that *will* bite you if skipped.

## Repository layout

- `acconeer/` -- everything from Acconeer's upstream XM126 SDK: board/shield defs
  (`acconeer/boards/`), devicetree bindings (`acconeer/dts/bindings/`), the SDK's
  reusable headers/sources (`acconeer/include/`, `acconeer/source/`), the Zephyr HAL
  glue (`acconeer/integration/`), and the 2 prebuilt closed-source libs
  (`acconeer/lib/`).
- `app/` -- this project's own code, flat (no further `source/`/`include/` split):
  `xm126_bthome_tank.c/.h` (the application), `acc_bthome_ble.c/.h` (BTHome +
  BLE DFU advertising), `acc_tank_config_ble.c/.h` (tank-config GATT service),
  `acc_serial_console_commands.c/.h` (console commands), plus the Zephyr build glue
  (`CMakeLists.txt`, `prj.conf`, `sysbuild.conf`, `sysbuild/mcuboot/prj.conf`,
  `boards/xm126.overlay`) and `TANK_CONFIG_BLE.md`.

`app/CMakeLists.txt` sets `ACCONEER_ROOT` to `../acconeer` (relative to `app/`) and
derives `BOARD_ROOT`/`DTS_ROOT`/`ACCONEER_RSS_LIB_DIR`/etc. from that -- if you ever move
either directory, that's the one place to fix.

## Toolchain

- nRF Connect SDK (NCS) **v3.2.0**, Zephyr 4.2.99 (bundled), Zephyr SDK **0.17.4**
  (ARM toolchain only), `west` in its own venv. Exact setup steps are in `README.md`.
- Board target: `xm126/nrf52840`. Shield: `acconeer_xb122` (set in `app/CMakeLists.txt`,
  not passed on the command line).
- Build:
  ```bash
  source ~/ncs-venv/bin/activate
  export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk/zephyr-sdk-0.17.4
  export ZEPHYR_NCS_ROOT=~/ncs
  make app
  ```

### Gotcha: `make` silently no-ops on source changes

The top-level `Makefile`'s rule for `app/out/merged.hex` has **no prerequisites**. Once
that file exists, `make app` will print `make: Nothing to be done` and skip the build
entirely -- even if you just edited every `.c` file in `app/`. Always force a real
rebuild after any source change:

```bash
rm -rf app/out
make app
```

A successful build produces, under `app/out/`: `merged.hex` (full flash image incl.
bootloader), `dfu_application.zip` (mcumgr/nRF Connect DFU package), and
`app/zephyr/xm126_bthome_tank.signed.bin` (the signed app image alone, for
`mcumgr image upload`). The `app/zephyr/` subdirectory name is the sysbuild default
image name (derived from the app source directory's basename, i.e. `app/`); the
`.elf`/`.bin`/`.hex` filenames themselves come from `CONFIG_KERNEL_BIN_NAME` in
`app/prj.conf`, not from any directory name.

## Verifying a change actually took effect (no unit tests exist)

There is no test framework in this repo. "Verification" means: rebuild clean, flash to
real hardware, and read the serial console. Two techniques that came up repeatedly and
are worth reusing:

**Check whether a symbol/library is *actually* linked, not just referenced in
`CMakeLists.txt`:**
```bash
arm-zephyr-eabi-nm app/out/app/zephyr/xm126_bthome_tank.elf \
  | grep -i <symbol_or_prefix>
```
This is how the unused `libacc_detector_presence_a121.a` was found and safely removed
(zero matching symbols in the linked binary) and how `acc_mcu_temperature_read_die_temp`
was confirmed present after porting that feature in. Don't trust what's merely listed in
`target_link_libraries`/`target_sources` -- confirm what's actually linked.

**Read the console over serial** (115200 baud, raw mode):
```bash
stty -F /dev/ttyUSB0 115200 raw -echo -crtscts
cat /dev/ttyUSB0
```
The measurement loop runs on `MEASURE_PERIOD_MS` (60 s by default) -- a short listen
window (e.g. 10 s) can easily catch total silence between cycles and look like a hang.
Listen for at least `MEASURE_PERIOD_MS + a few seconds` before concluding anything.

## Flashing (no J-Link needed)

Every XM126 module ships pre-flashed with the same MCUboot bootloader this project
builds against, with serial recovery over UART enabled. Full instructions are in
`README.md`; the short version:

1. Enter recovery: hold **DFU**, press and release **RESET**, release **DFU** -- or, if
   this firmware (any version of it) is already running, just type `dfu` + Enter on the
   console UART instead (see `app/acc_serial_console_commands.c`). The latter only
   works once *some* build of this firmware is already on the device; first flash of a
   truly blank/different board needs the button sequence.
2. `mcumgr conn add usb0 type="serial" connstring="dev=/dev/ttyUSB0,baud=115200,mtu=1024"`
3. `mcumgr -c usb0 image upload app/out/app/zephyr/xm126_bthome_tank.signed.bin`
4. `mcumgr -c usb0 reset`

If the tank-config values (diameter/floor-offset, see below) ever end up wrong after a
flash -- e.g. leftover data from different firmware, since flashing over
UART/mcumgr never touches `storage_partition` -- type `resetcfg` + Enter on the console
to wipe them back to the compiled-in defaults.

## Architecture invariants -- do not break these

These are non-obvious constraints found by shipping this firmware against real hardware
and Home Assistant. Each one was a real, working bug at some point.

1. **Both BLE advertising sets must use `BT_LE_ADV_OPT_USE_IDENTITY`.** Without it,
   Zephyr hands out a fresh private address on every `bt_le_ext_adv_start()` call. The
   BTHome set restarts on every update, so omitting this flag means Home Assistant sees
   a "new device" on nearly every packet instead of one stable one (this exact bug
   shipped once and was caught via a user report, not testing).

2. **Neither advertising set may use `BT_LE_ADV_OPT_EXT_ADV`.** BTHome's spec doesn't
   require legacy PDUs, but Home Assistant's Bluetooth integration (via BlueZ's default
   passive scan) does not see extended-PDU advertisements at all unless the scanner
   explicitly opts into extended scanning. Legacy PDUs are the real-world requirement
   even though the protocol itself is silent on it.

3. **A BTHome update must advertise for a multi-second burst, not one instant event.**
   `num_events=1` (a single legacy-PDU transmission, over in under a millisecond) was
   fine at the old 1 Hz update rate, but once updates became minutes apart (see below),
   a scanner had to be listening at that exact instant or wait a long time for the next
   chance. See `BTHOME_ADV_BURST_MS` in `app/acc_bthome_ble.c`.

4. **The BTHome advertising payload is right at the legacy PDU size limit.** 30 of 31
   bytes used (see the byte-accounting comment above `BTHOME_PAYLOAD_MAX_LEN` in
   `app/acc_bthome_ble.c`). Adding another object requires shrinking or dropping
   `BTHOME_ADV_SHORT_NAME` to still fit -- verify with actual byte counting, this budget
   has no slack for guessing.

5. **`DEFAULT_PRESET_CONFIG` must roughly match the real tank's depth.** The BLE
   tank-config GATT service (`acc_tank_config_ble.*`, see `app/TANK_CONFIG_BLE.md`) only
   lets a user reconfigure `diameter_mm`/`floor_offset_mm` -- the detection *range*. It
   does **not** touch the rest of the compiled preset's detector tuning (profile,
   threshold method/sensitivity, signal quality). A `SMALL_TANK` preset (tuned for
   <=0.5 m) pointed at a tank several meters out via BLE reconfiguration can end up
   permanently stuck at `NO_DETECTION`/`OUT_OF_RANGE` even though the range itself is
   "correct" -- this shipped once and silently suppressed the BTHome `distance`/
   `volume` objects (they're only sent when a measurement is valid) until diagnosed.

6. **Radar sampling and BLE broadcasting are intentionally decoupled.** The radar
   samples on `MEASURE_PERIOD_MS` regardless (cheap), but a BLE broadcast only fires via
   `volume_broadcast_needed()` -- on a meaningful volume change
   (`BROADCAST_VOLUME_THRESHOLD_ML`) or a heartbeat timeout
   (`BROADCAST_HEARTBEAT_MS`) -- since BLE TX, not radar sampling, is the actual power
   cost. Don't reflexively call `acc_bthome_ble_update()` on every measurement; that
   was the original (power-hungry) design and was deliberately changed.

7. **Battery percent uses a piecewise 1S LiPo discharge curve** (`lipo_1s_curve[]` in
   `app/xm126_bthome_tank.c`), not a 2-point linear map -- LiPo cells sit
   flat around 3.7-4.0 V for most of their capacity then drop steeply near empty, so a
   straight line reads very inaccurately. If the deployment's battery chemistry/cell
   count changes, this table needs replacing, not just its endpoints.

8. **MCU temperature comes from the nRF52840's own die-temperature sensor**
   (`acc_mcu_temperature_read_die_temp()`), not the radar's
   `acc_detector_distance_result_t.temperature` field -- the SDK's own header documents
   that field as "poor absolute accuracy," meant only to drive the sensor's internal
   recalibration logic, not as a standalone reading.

## Wire-format facts worth not re-deriving from memory

- BTHome v2 object IDs in use: `0x00` packet id (uint8), `0x01` battery % (uint8),
  `0x02` temperature (sint16, factor 0.01), `0x0C` voltage (uint16, factor 0.001 -> raw
  value is mV), `0x40` distance mm (uint16), `0x55` volume storage (uint32, factor
  0.001 L -> raw value is mL). These were verified directly against
  <https://bthome.io/format/> rather than trusted from memory -- a first pass on the
  battery object's byte width (uint8 vs uint16) actually disagreed between two fetches
  of the same page; always re-verify object IDs/factors/widths against the live spec
  before adding a new one, don't trust a cached memory of the table.
- BTHome does not support custom per-entity attributes, icons, or names -- it's a fixed
  enumeration. The only device-level customization available is the advertised local
  name (`BTHOME_ADV_SHORT_NAME`).
- The tank-config GATT service protocol (custom 128-bit UUID, `diameter_mm`/
  `floor_offset_mm` characteristics) is fully documented in `app/TANK_CONFIG_BLE.md`,
  including exact ATT error codes and byte encodings -- read that before touching
  `acc_tank_config_ble.*`.

## Closed-source dependencies (relevant if changing what's linked)

This repo bundles exactly 2 closed-source libraries in `acconeer/lib/` (Acconeer's,
BSD-3-Clause redistribution license, no source provided): `libacconeer_a121.a` (core
sensor control/calibration) and `libacc_detector_distance_a121.a` (the distance
algorithm). A third, `libacc_detector_presence_a121.a`, was removed after confirming via
`nm` that it contributed zero symbols to the linked binary -- this app never calls the
presence detector API, the original sample template just linked it unconditionally.

Separately, the *compiled firmware* (not this repo's own files) also incorporates 5
closed-source Nordic libraries from NCS's `nrfxlib` module at build time: the
SoftDevice Controller, MPSL core, MPSL FEM common glue, and two CC310 crypto libraries
(one in the app image, one in MCUboot). These aren't bundled/redistributed by this repo
-- the build toolchain pulls them from the NCS checkout -- but they are part of what
actually runs on the device. Nordic's license (`nrfxlib/LICENSE`, "Nordic-5-Clause") is
more restrictive than Acconeer's: binary use is permitted only on a Nordic IC (true
here) and reverse engineering is explicitly prohibited.

If you add a new dependency, verify with `nm` on the linked `.elf` whether it's actually
used before assuming it needs to be redistributed or documented as a dependency --
unused-but-linked static libraries contribute zero bytes and zero legal exposure, but an
accidentally-added *used* one changes both.

## How this repo relates to the upstream Acconeer SDK

This is a deliberate extraction, not a fork with extra files removed after the fact.
Only the files this app's `CMakeLists.txt` actually needs are present under `acconeer/`:
board/shield defs, devicetree bindings, the 2 required prebuilt libs, and the
`source/`/`include/` subset this app references (computed as the transitive closure of
`#include "..."` statements, not guessed). If pulling in more code from upstream
Acconeer XM126 SDK releases in the future (e.g. a newer detector algorithm version),
re-trace that closure rather than copying whole directories -- `include/` alone had
~90 headers in the full SDK covering 25 other sample applications this repo doesn't
build.

## CI

`.github/workflows/build.yml` builds the app from scratch on every push/PR -- installs
the exact NCS v3.2.0 + Zephyr SDK 0.17.4 toolchain (no pre-built Docker image could be
confirmed to correctly bundle this exact version pairing, so a from-scratch install was
used instead of guessing at an image reference), with the NCS checkout, Zephyr SDK, and
west venv cached keyed on the pinned versions. Bump the `NCS_VERSION`/
`ZEPHYR_SDK_VERSION` env vars in that file (and the cache key) together if upgrading the
toolchain, or the cache will silently serve a stale/mismatched toolchain.
