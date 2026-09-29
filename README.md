# Embedded System Project - Robo Car Firmware

Firmware for the line-following car (Robo Pico / Pico W): line following
with barcode-driven navigation, obstacle avoidance, IMU/terrain
monitoring, and WiFi/MQTT telemetry.

This repo is a **single shared project**, not five separate repos. CMake
compiles every subsystem's `.c` file into one `.uf2` (see
`CMakeLists.txt` - the pico-sdk build always produces one firmware
image per board, so splitting into separate repos would just mean
copy-pasting files back together at build time). What keeps everyone's
work separated instead is **one header + one source file per
subsystem** (per NFR-02), plus a shared `config.h` for every pin
assignment so there's a single place to resolve pin conflicts.

## Structure

Each subsystem lives in its own folder, containing just its `.h`/`.c`
pair - open your folder, everything you own is right there.

| Folder / file                     | Subsystem                              | Owner    | Covers |
|-----------------------------------|-----------------------------------------|----------|--------|
| `config.h` (repo root)            | Shared pin map (no logic here)          | everyone | -      |
| `motor/`                          | Motor / Encoders / PID                  | Dave     | FR-11, FR-12, FR-13, NFR-13, NFR-14 |
| `line_tracking/`                  | IR Line Following                       | Vania    | FR-01, FR-09, FR-10, NFR-03, NFR-10 |
| `barcode/`                        | Barcode Detection & Decoding            | Vania    | FR-02, FR-03, NFR-04 |
| `imu_terrain/`                    | IMU / Terrain Monitoring                | Sook Yee | FR-04, FR-05, FR-06, NFR-05, NFR-06 |
| `obstacle_scan/`                  | Ultrasonic Scanning / Servo / Avoidance | Felix    | FR-07, FR-08, FR-09, FR-10, NFR-07, NFR-08, NFR-09 |
| `mission_state/`                  | Mission State (top-level statechart)    | Felix    | FR-17 (+ coordinates all of the above) |
| `telemetry/`                      | Telemetry / Logging                     | Felix    | FR-16, FR-18, NFR-11, NFR-12 |
| `wifi_mqtt/`                      | WiFi / MQTT connection + publish        | Jun Kai  | FR-14, FR-15, FR-17, FR-19, NFR-11 |
| `main.c` (repo root)              | Brings every subsystem up, super-loop   | -        | - |

Every subsystem's `.h` is `#include`d from `main.c` (or from another
subsystem's `.c`) by its bare filename, e.g. `#include "motor.h"` - you
don't need `motor/motor.h`. `CMakeLists.txt`'s
`target_include_directories` adds every subsystem folder (plus the repo
root, for `config.h`) to the include path, so this works regardless of
which folder the including file lives in.

Every subsystem exposes just `_init()` + `_update()` (barcode is
event-driven instead: `barcode_on_edge_detected()`, called by
`line_tracking.c`). `main.c` is intentionally thin - it just calls
each subsystem's `_init()` once and `_update()` every loop iteration,
in the order sensing -> mission_state -> actuation -> comms. It runs as
a bare pico-sdk polling loop for now (matching the working
`motor_test.c` button/motor demo), not the FreeRTOS task table from the
design review yet - swapping it for real RTOS tasks later shouldn't
require changing any subsystem's function signatures.

**Rule of thumb:** only touch the `.h`/`.c` pair you own, plus
`config.h` if you need a new pin (check it's actually free first - the
comments in there list what's already claimed, including the pins the
Pico W's wireless chip reserves internally: GP23/24/25/29). If you need
to call into someone else's subsystem, add the function to *their*
header and let them fill in the body, or ask in the group chat before
changing another owner's file.

## Building

This uses the same VS Code Raspberry Pi Pico extension workflow as the
motor demo:

1. Open this folder in VS Code (`File > Open Folder...`).
2. Raspberry Pi Pico extension sidebar -> **Import Project** (not "New
   Project" - this folder already has its own `CMakeLists.txt` and
   `pico_sdk_import.cmake`, Import Project just configures CMake
   against the existing files). Point it at your local Pico SDK install
   when asked, and leave the board as `pico_w`.
3. **Compile Project** from the same sidebar. The `.uf2` shows up under
   `build/robo_car.uf2`.
4. Hold BOOTSEL on the Pico, plug it in (or use **Run Project (USB)**
   in the extension), and copy/drag the `.uf2` onto the `RPI-RP2` drive
   that appears.

No need to run `pico-sdk` project setup separately - `CMakeLists.txt`
and `pico_sdk_import.cmake` are already in this repo and point at
whatever `PICO_SDK_PATH` your install of the extension resolves.

## Workflow

- Branch per subsystem/feature (e.g. `dave/motor-pid`), PR into `main`.
- Please make sure `Compile Project` succeeds locally before opening a
  PR - a broken build in one subsystem file blocks everyone else's
  `.uf2`, since it's all one executable.
- Stick to your own `.c`/`.h` pair; coordinate in the group chat before
  editing `config.h` or another owner's files.
