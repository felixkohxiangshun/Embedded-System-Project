# Robo Car - Autonomous Line-Following Firmware

Firmware for an autonomous line-following car built on the Robo Pico
(Raspberry Pi Pico W / RP2040). The car follows a line, reads barcodes
to decide turns, scans for and avoids obstacles with an ultrasonic
sensor, monitors terrain with an IMU, and reports live status over
WiFi/MQTT.

---

## Tech Stack

| Layer            | Technology |
|-------------------|------------|
| Language           | C (C11), pico-sdk |
| Build system        | CMake + Ninja |
| Toolchain           | ARM GCC (via the Pico VS Code extension) |
| Board               | Robo Pico (Raspberry Pi Pico W / RP2040) |
| Networking          | CYW43 WiFi + MQTT (`pico_cyw43_arch`) |
| Sensors/actuators   | PWM motors + quadrature encoders, IR array (ADC), ultrasonic + servo, I2C IMU, WS2812 status LED (PIO) |

---

## Team

| Member | Name      | Role                          | Primary Files |
|--------|-----------|-------------------------------|----------------|
| M1     | Dave      | Motor / PID Control            | `motor/` |
| M2     | Vania     | Line Tracking & Barcode        | `line_tracking/`, `barcode/` |
| M3     | Sook Yee  | IMU / Terrain Monitoring       | `imu_terrain/` |
| M4     | Felix     | Obstacle Avoidance, Mission State, Telemetry | `obstacle_scan/`, `mission_state/`, `telemetry/` |
| M5     | Jun Kai   | WiFi / MQTT                    | `wifi_mqtt/` |

---

## Project Structure

```
Embedded-System-Project/
├── main.c                       Startup + main super-loop
├── config.h                     Shared GPIO pin map (single source of truth)
├── CMakeLists.txt                Build config - compiles every subsystem into one .uf2
├── pico_sdk_import.cmake
│
├── motor/
│   ├── motor.h
│   └── motor.c                   Encoders + PID speed control
│
├── line_tracking/
│   ├── line_tracking.h
│   └── line_tracking.c            IR line following
│
├── barcode/
│   ├── barcode.h
│   └── barcode.c                  Barcode decode (event-driven, triggered by line_tracking)
│
├── imu_terrain/
│   ├── imu_terrain.h
│   └── imu_terrain.c              IMU-based hump/shock detection
│
├── obstacle_scan/
│   ├── obstacle_scan.h
│   └── obstacle_scan.c            Servo-swept ultrasonic scanning + avoidance
│
├── mission_state/
│   ├── mission_state.h
│   └── mission_state.c            Top-level mission statechart, E-stop
│
├── telemetry/
│   ├── telemetry.h
│   └── telemetry.c                Status packaging for MQTT publish
│
├── wifi_mqtt/
│   ├── wifi_mqtt.h
│   └── wifi_mqtt.c                WiFi connection + MQTT client
│
├── .gitignore
└── README.md
```

---

## Prerequisites

| Requirement | Version | Check |
|---|---|---|
| VS Code | latest | https://code.visualstudio.com/ |
| Raspberry Pi Pico extension | latest | installed from the VS Code Extensions panel |
| Git | any | `git --version` |
| Robo Pico board (Pico W) | - | with a USB cable |

The Pico extension installs and manages the pico-sdk, ARM toolchain,
CMake, and Ninja for you (into `~/.pico-sdk`) - there's nothing else to
install manually.

---

### Install VS Code and the Pico extension

1. Install VS Code: https://code.visualstudio.com/
2. Open the Extensions panel (`Cmd+Shift+X` / `Ctrl+Shift+X`) and
   install **Raspberry Pi Pico**.
3. Open the extension's sidebar (Pico icon in the activity bar) once -
   on first use it downloads the pico-sdk, toolchain, and build tools
   automatically.

---

## Step-by-Step Setup

### 1. Clone the repository

```bash
git clone https://github.com/felixkohxiangshun/Embedded-System-Project.git
cd Embedded-System-Project
```

### 2. Open the folder in VS Code

```bash
code .
```

### 3. Import the project

Pico extension sidebar → **Import Project**. This repo already has its
own `CMakeLists.txt` and `pico_sdk_import.cmake`, so Import Project
just configures CMake against the existing files - do **not** use "New
Project" (that scaffolds a fresh, empty project instead). Leave the
board type as `pico_w`.

### 4. Compile

Pico extension sidebar → **Compile Project**. Output firmware is
written to `build/robo_car.uf2`.

### 5. Flash

Hold **BOOTSEL** on the Pico while plugging it in (or use **Run
Project (USB)** in the extension), then copy `build/robo_car.uf2` onto
the `RPI-RP2` drive that appears.

---

## Firmware Design

### Single image, modular ownership

The firmware always builds to one `.uf2` - the RP2040 runs a single
program, so there's no way to ship separate binaries per subsystem.
Ownership is instead kept separate at the source level: each subsystem
lives in its own folder with just its own `.h`/`.c` pair, and every pin
assignment lives in the single shared `config.h` so conflicts have one
place to be resolved. `CMakeLists.txt` adds every subsystem folder to
the include path, so headers are always included by bare filename
(`#include "motor.h"`) regardless of which folder is including them.

### Subsystem contract

| Subsystem | `_init()` | Update model | Notes |
|---|---|---|---|
| `motor` | yes | `_update()` every loop | PID speed control from encoder counts |
| `line_tracking` | yes | `_update()` every loop | Signals `barcode` on a barcode-zone edge |
| `barcode` | yes | event: `barcode_on_edge_detected()` | Not polled - called by `line_tracking` |
| `imu_terrain` | yes | `_update()` every loop | Reports humps/shocks to `mission_state` |
| `obstacle_scan` | yes | `_update()` every loop | Servo-swept ultrasonic sweep, reports obstacles |
| `mission_state` | yes | `_update()` every loop | Central statechart; consumes events from the above |
| `telemetry` | yes | `_update()` every loop | Rate-limited; builds the MQTT payload |
| `wifi_mqtt` | yes | `_update()` every loop | Owns the WiFi/MQTT connection; `telemetry` publishes through it |

`main.c` calls every `_init()` once at startup, then loops calling each
`_update()` in order: sensing → mission state → actuation → comms. This
is currently a bare pico-sdk polling loop, not the FreeRTOS task table
from the design review - function signatures are written so that
swapping in real RTOS tasks later won't require changing subsystem
code.

---

## Requirements Coverage

| Folder | Requirements covered |
|---|---|
| `motor/` | FR-11, FR-12, FR-13, NFR-13, NFR-14 |
| `line_tracking/` | FR-01, FR-09, FR-10, NFR-03, NFR-10 |
| `barcode/` | FR-02, FR-03, NFR-04 |
| `imu_terrain/` | FR-04, FR-05, FR-06, NFR-05, NFR-06 |
| `obstacle_scan/` | FR-07, FR-08, FR-09, FR-10, NFR-07, NFR-08, NFR-09 |
| `mission_state/` | FR-17 (coordinates all subsystems) |
| `telemetry/` | FR-16, FR-18, NFR-11, NFR-12 |
| `wifi_mqtt/` | FR-14, FR-15, FR-17, FR-19, NFR-11 |

---

## Contributing

- Branch per subsystem/feature (e.g. `dave/motor-pid`), PR into `main`.
- Confirm **Compile Project** succeeds locally before opening a PR - a
  build failure in any one subsystem blocks the shared `.uf2`.
- Only edit the `.h`/`.c` pair you own. Changes to `config.h` (new pin
  assignments) or another subsystem's files should be coordinated with
  its owner first - check `config.h`'s comments for what's already
  claimed, including the pins the Pico W's wireless chip reserves
  internally (GP23/24/25/29).
