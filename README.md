# XM126 Tank Volume (BTHome + BLE DFU)

Firmware for the [Acconeer XM126](https://www.acconeer.com) radar module (A121 radar,
nRF52840) that turns it into a battery-friendly BLE tank-level/volume sensor for [Home
Assistant](https://www.home-assistant.io/) -- no custom integration or companion app
required -- with wireless firmware updates over BLE.

This is a single-purpose extraction of one sample from Acconeer's XM126 SDK, kept in its
own repo so the firmware, its history, and its CI aren't buried inside the much larger
upstream SDK (which ships ~25 other example applications this project doesn't use).

## Features

- **BTHome v2 broadcast** -- Home Assistant's built-in Bluetooth integration discovers
  and decodes this natively. No pairing, no custom component. Publishes tank distance,
  volume, battery percentage, supply voltage, and MCU temperature.
- **Wireless firmware updates over BLE** -- a periodic connectable advertising window
  (open right after boot, then every few minutes) exposes an [mcumgr](https://docs.zephyrproject.org/latest/services/device_mgmt/mcumgr.html)
  SMP service, updatable via `mcumgr` CLI or the nRF Connect mobile app's DFU tab. No
  physical DFU button needed -- the firmware can also drop itself into MCUboot serial
  recovery on a typed console command (see below).
- **BLE-configurable tank geometry** -- a small custom GATT service lets you set the
  tank's diameter and the sensor's mounting height over BLE, persisted in flash, instead
  of baking it into the firmware at compile time.
- **Power-aware by design** -- radar sampling and BLE broadcasting are decoupled: the
  radar samples on a short, cheap interval, but a BLE broadcast is only sent when the
  volume changes meaningfully or a heartbeat interval elapses, since the BLE transmit is
  the expensive part.

See [`app/TANK_CONFIG_BLE.md`](app/TANK_CONFIG_BLE.md) for the tank-configuration GATT
service's protocol reference (ATT error codes, exact byte encodings, security notes).

## Toolchain setup

Built with [nRF Connect SDK](https://www.nordicsemi.com/Products/Development-software/nrf-connect-sdk)
(NCS) v3.2.0 and the matching Zephyr SDK toolchain (0.17.4).

```bash
# 1. apt dependencies (Ubuntu)
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  git cmake ninja-build gperf ccache dfu-util device-tree-compiler wget \
  python3-dev python3-venv python3-pip python3-setuptools python3-wheel \
  xz-utils file make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1

# 2. west, in a venv
python3 -m venv ~/ncs-venv
source ~/ncs-venv/bin/activate
pip install -U pip wheel west

# 3. NCS checkout, pinned to v3.2.0
west init -m https://github.com/nrfconnect/sdk-nrf --mr v3.2.0 ~/ncs
cd ~/ncs && west update   # clones zephyr, nrfxlib, mcuboot, etc. -- several GB, slow

# 4. Python requirements
pip install -r zephyr/scripts/requirements.txt \
            -r nrf/scripts/requirements.txt \
            -r bootloader/mcuboot/scripts/requirements.txt

# 5. Zephyr SDK (ARM toolchain only)
west sdk install -t arm-zephyr-eabi -b ~/zephyr-sdk
```

## Building and flashing

```bash
source ~/ncs-venv/bin/activate
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-sdk/zephyr-sdk-0.17.4
export ZEPHYR_NCS_ROOT=~/ncs
cd xm126-bthome-tank

make app          # -> app/out/merged.hex
make flash        # needs a J-Link on real hardware
```

No J-Link? See [Initial flash and firmware updates over UART](#initial-flash-and-firmware-updates-over-uart)
below for flashing via MCUboot's serial recovery (every XM126 module ships pre-flashed
with this bootloader), or use the `dfu_application.zip` produced in the same `out/`
directory with the nRF Connect mobile app's DFU tab once any version of this firmware is
already running (it opens its own BLE DFU window).

## Repository layout

- `acconeer/` -- everything from Acconeer's upstream XM126 SDK this firmware needs:
  the `xm126` board definition and `acconeer_xb122` shield (`acconeer/boards/`),
  devicetree bindings (`acconeer/dts/bindings/`), headers and sources for the SDK's
  reusable modules (`acconeer/include/`, `acconeer/source/`), the Zephyr HAL glue
  (`acconeer/integration/`), and the two prebuilt closed-source radar signal processing
  libraries this firmware actually links (`acconeer/lib/`): `libacconeer_a121` (core
  sensor control/calibration) and `libacc_detector_distance_a121` (the
  distance-detection algorithm). No source is provided for either, only a
  BSD-3-Clause redistribution license (`LICENSES/`) that permits shipping the compiled
  binaries as-is.
- `app/` -- this project's own code: the application (`xm126_bthome_tank.c`),
  the BTHome/BLE-DFU layer (`acc_bthome_ble.*`), the tank-config GATT service
  (`acc_tank_config_ble.*`), the serial console commands (`acc_serial_console_commands.*`),
  and the Zephyr build glue (`CMakeLists.txt`, `prj.conf`, `sysbuild.conf`,
  `boards/xm126.overlay`).

## Tank geometry defaults

`DEFAULT_TANK_DIAMETER_MM` and `DEFAULT_FLOOR_OFFSET_MM` in
`app/xm126_bthome_tank.c` are only used the first time the device boots (before
anything has been configured over BLE, see [Tank configuration over
BLE](#tank-configuration-over-ble)). `DEFAULT_PRESET_CONFIG` (`SMALL_TANK` /
`MEDIUM_TANK` / `LARGE_TANK`) selects the distance detector's sensitivity/profile tuning
for the expected tank size class -- edit it to match your tank, and keep
`DEFAULT_FLOOR_OFFSET_MM` in sync if you do.

## BTHome broadcast

Always-on, non-connectable advertisement. Home Assistant should discover it
automatically via its Bluetooth integration once BTHome support is enabled; no app or
pairing needed. Fields:

| Object | Meaning |
|---|---|
| `0x00` packet id | increments every update |
| `0x01` battery (%) | see below -- tuned for a 1S LiPo cell |
| `0x02` temperature (degC, 0.01 resolution) | nRF52840 MCU die temperature |
| `0x0C` voltage (V, 0.001 resolution) | raw supply/battery voltage |
| `0x40` distance (mm) | raw sensor-to-surface distance |
| `0x55` volume storage (L, 0.001 resolution) | computed tank volume |

Distance/volume are only included while the level is in range; battery/voltage/
temperature are always included, independent of level validity. Out-of-range readings
advertise packet id + battery + temperature + voltage, no distance/volume.

MCU temperature is read via the SDK's existing `acc_mcu_temperature` module (the
nRF52840's own die-temperature sensor, `SENSOR_CHAN_DIE_TEMP` via Zephyr's `TEMP_NRF5`
driver -- no devicetree changes needed, that node is enabled by default at the SoC
level). It reflects the MCU's own junction temperature (including self-heating), not a
calibrated ambient reading -- treat it as indicative. This is a different, more
accurate temperature source than the radar's own `acc_detector_distance_result_t.temperature`
field, which the SDK's header explicitly documents as "poor absolute accuracy,"
intended only to drive the sensor's internal recalibration-needed logic, not as a
standalone reading.

Adding this object pushed the worst-case BTHome payload close to the 31-byte legacy
advertising PDU limit (30 of 31 bytes used, see the byte-accounting comment above
`BTHOME_PAYLOAD_MAX_LEN` in `app/acc_bthome_ble.c`) -- there's exactly 1 byte of
slack left; a future object will need to shrink or drop the shortened advertised name to
still fit.

Battery voltage is read via the SDK's existing `acc_battery_info` module (ADC +
voltage-divider, the same one `example_detector_distance_ble_monitor` uses) -- see the
`vbatt` node enabled in `app/boards/xm126.overlay`. The voltage divider
(`output-ohms`/`full-ohms` in that overlay, a 3x scale-up) combined with the ADC's
~1.8V full-scale range (`ADC_GAIN_1_3` against the internal reference) comfortably
covers a 1S LiPo's full 4.2V charged voltage.

The battery-percent mapping (`battery_voltage_to_percent()` in
`app/xm126_bthome_tank.c`) is a piecewise-linear approximation of a 1S LiPo's
open-circuit discharge curve (`lipo_1s_curve[]`, 4.20V=100% down to 3.27V=0%) -- LiPo
cells sit fairly flat around 3.7-4.0V for most of their capacity then drop steeply near
empty, so a straight 2-point linear map (fine for some chemistries) reads very
inaccurately for LiPo. Treat the percentage as indicative rather than exact -- it
doesn't account for voltage sag under load or cell aging/temperature. For a different
battery chemistry/cell count, replace the table with the appropriate curve, or ignore
the percent entirely and use the raw voltage object (`0x0C`) to compute remaining
capacity in Home Assistant instead.

**Measurement vs. broadcast cadence**: the radar samples every `MEASURE_PERIOD_MS`
(`app/xm126_bthome_tank.c`, default 60 s -- once a minute), since sampling
is cheap and this is fast enough to catch a fill/drain event shortly after it starts.
But a BLE broadcast -- the actually power-hungry part -- is only sent when the volume
has moved by at least `BROADCAST_VOLUME_THRESHOLD_ML` (default 2 L, filters out
measurement noise around a static level) since the last one, or when
`BROADCAST_HEARTBEAT_MS` (default 15 min) has elapsed regardless, so Home Assistant
still sees a reasonably fresh packet while the tank is untouched for a long time. Tune
both to your tank: a small/fast-filling tank wants a shorter measurement period and/or
smaller threshold; a large slow-changing one can go the other way for even less power
use. This matters for a battery deployment -- with a 1/s broadcast cadence, BLE TX
dominates power use even though the tank realistically only changes level a few minutes
a day (e.g. while a pump runs).

Each update repeats for `BTHOME_ADV_BURST_MS` (`app/acc_bthome_ble.c`, default 5 s)
at the fast interval before going quiet again, rather than sending a single
near-instantaneous packet -- with updates potentially minutes apart, a lone packet is
too easy for a scanner to simply miss (this is what breaks Home Assistant discovery if
the broadcast cadence is lowered without a burst window; the burst window fixes it
while keeping the on-air time tiny compared to the gap between updates).

The device advertises from its fixed identity address (`BT_LE_ADV_OPT_USE_IDENTITY`),
so the MAC address is stable across measurements and reboots -- Home Assistant sees one
consistent BTHome device rather than a new one on every packet. (An earlier build of
this sample was missing that flag; every advertising restart picked a fresh private
address, which showed up in Home Assistant as an ever-growing pile of "new" BTHome
sensors -- fixed, but worth knowing about if you're running an old build.)

## Initial flash and firmware updates over UART

The very first flash (or any update without BLE) uses MCUboot's serial recovery over
UART0, per `doc/XM126 Software User Guide.pdf` \S4.2 -- every XM126 module ships
pre-flashed with this same bootloader:

```bash
mcumgr conn add usb0 type="serial" connstring="dev=/dev/ttyUSB0,baud=115200,mtu=1024"
mcumgr -c usb0 image upload app/out/app/zephyr/xm126_bthome_tank.signed.bin
mcumgr -c usb0 reset
```

Entering serial recovery normally needs the physical DFU button (hold DFU, press and
release RESET, release DFU). Once this firmware is running, there's a second way that
needs no button: type `dfu` followed by Enter on the same console UART, at any time --
the app sets the same retained-memory boot-mode flag the button does and reboots
straight into serial recovery (see `app/acc_serial_console_commands.c`). Handy when
the DFU button isn't wired up (e.g. a bare module without a breakout board). Only works
while the app's console UART is active, i.e. `suspend_uart` is `false`.

**Note**: `mcumgr image upload` (over UART *or* BLE) only ever writes to the image
slot partitions, never to `storage_partition` (where the tank config settings in
[`app/TANK_CONFIG_BLE.md`](app/TANK_CONFIG_BLE.md) are persisted). A board that shipped
with different firmware pre-flashed can still have that firmware's leftover data
sitting in `storage_partition` the first time this app boots -- if the diameter/
floor-offset readings look wrong from the very first boot (not after a BLE write),
that's likely why. Type `resetcfg` followed by Enter on the console UART to erase the
persisted tank config and reboot with the compiled-in defaults.

## BLE firmware updates (mcumgr / "nRF DFU")

No button needed. Every 5 minutes (`DFU_WINDOW_PERIOD_S` in `app/acc_bthome_ble.c`),
the device opens a connectable advertising window for 30 seconds
(`DFU_WINDOW_DURATION_S`) under the name `CONFIG_BT_DEVICE_NAME` ("XM126 Tank"). A
window also opens right after boot. Connect during that window with any mcumgr client,
e.g.:

```bash
mcumgr --conntype ble --connstring peer_name='XM126 Tank' image list
mcumgr --conntype ble --connstring peer_name='XM126 Tank' image upload app/out/app/zephyr/xm126_bthome_tank.signed.bin
mcumgr --conntype ble --connstring peer_name='XM126 Tank' reset
```

or with nRF Connect for Mobile's Device Manager, scanning for "XM126 Tank" during the
window.

## Tank configuration over BLE

A custom GATT service (UUID `c9c9b3ec-1cee-4f9e-b62c-9d8f6a1b6f00`) is reachable during
the same connectable window as above -- one connection can both update firmware and
reconfigure the tank. Two read/write characteristics, both little-endian `uint32`
in millimeters:

| Characteristic UUID | Meaning | Range |
|---|---|---|
| `...6f01` | tank diameter (mm) | 50 - 5000 |
| `...6f02` | sensor-to-floor mounting distance (mm) | 50 - 15000 |

Any generic BLE tool can read/write these (e.g. nRF Connect for Mobile's plain
attribute screen -- connect, find the service, write 4 raw bytes little-endian). Values
persist in flash (Zephyr settings/NVS) and survive reboots and firmware updates; writing
either one triggers the detector to reconfigure and recalibrate for the new range on the
next measurement cycle.

See [`app/TANK_CONFIG_BLE.md`](app/TANK_CONFIG_BLE.md) for the full protocol reference
(ATT error codes, exact byte encodings, security notes).

## License

Acconeer's original SDK code is BSD-3-Clause licensed (see `LICENSES/`); the prebuilt
libraries in `acconeer/lib/` are proprietary Acconeer binaries redistributed under the
SDK's terms. This project's own additions (everything under `app/`) follow the same
BSD-3-Clause terms.
