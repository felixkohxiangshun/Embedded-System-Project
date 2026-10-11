# Robo Pico Car: INF2004 Group 11

The aim of this project is to build an autonomous line-following car. It follows a
black line, reads Code 39 barcodes (A to D) to execute commands (left/right turn, uturn), measures hump
height, detects shocks, detects obstacle and perform obstacle avoidance, and reports to an online dashboard.

## Tech stack

| Layer | What we use |
|---|---|
| Language | C (the requirements ask for C99 and Barr-C; the build uses `gnu11` and currently compiles without warnings) |
| RTOS | **micro T-Kernel 3.0**, dual-core RP2040 port |
| SDK | Raspberry Pi **Pico C SDK** 2.2.0 (USB and WiFi driver source) |
| Compiler | Arm GNU Toolchain, `arm-none-eabi-gcc` |
| Build | GNU `make` (`build_make/`), output is a `.uf2` file |
| Microcontroller | Raspberry Pi **Pico W** (RP2040, CYW43439 WiFi) on a Cytron **Robo Pico** |
| Communication | WiFi + MQTT (QoS 0 telemetry, QoS 1 commands) |
| Tests | Unit tests on a PC (`testing/host`), on-target self-tests (`make TEST=1`), manual test logs |

### Hardware

| Part | Used for | Pins |
|---|---|---|
| 2 DC motors with quadrature encoders | driving and speed feedback | motors GP8 to GP11, encoders GP2, GP3, GP6, GP7 |
| 3 IR sensors | left and centre follow the line, right reads barcodes | GP26, GP27, GP28 (ADC) |
| LSM303DLHC IMU | hump height and shock | GP4 (SDA), GP5 (SCL) |
| HC-SR04 ultrasonic sensor on a servo | obstacle detection and profiling | echo GP16, trigger GP17, servo GP12 |
| Start/stop button, status LED | operator control | GP20, GP18 |

The full pin table, with owners, is in [TEAM.md](TEAM.md). 

## Team and roles

| Member | Student ID | Subsystem (folder) | Responsible for |
|---|---|---|---|
| Tan Kun Yi Dave | 2502730 | Motor control (`app_program/motor_control/`) | PID speed control, wheel encoders, drift correction, 90° and 180° turns, start button |
| Vania Teng | 2503206 | Line following and barcode (`app_program/line_barcode/`) | three IR sensors, 1 kHz sampler, line following, barcode decoding |
| Lau Sook Yee | 2503178 | IMU and terrain (`app_program/imu_terrain/`) | hump height, tallest hump, shock detection |
| Wong Jun Kai | 2500258 | Dashboard and telemetry (`app_program/dashboard/`) | telemetry frame every 500 ms, WiFi and MQTT status, JSON, heartbeat, remote E_STOP and RESUME |
| Felix Koh | 2500256 | Mission and obstacle avoidance (`app_program/mission_obstacle/`) | system state machine, ultrasonic and servo obstacle scan, bypass, system integration |

Each subsystem has its own folder and its own test log in `testing/<folder>/TEST_LOG.md`.
Only the owner edits their folder. Shared files (`app_program/app.h`, `app_if.h`,
`app_main.c`, `config/`, `TEAM.md`) are changed by agreement of the team.
Priorities, interfaces and the full task list are in [TEAM.md](TEAM.md).

## Repository layout

```
app_program/        our firmware: start with app_main.c
  motor_control/      Dave
  line_barcode/       Vania
  imu_terrain/        Sook Yee
  dashboard/          Jun Kai
  mission_obstacle/   Felix
  app.h  app_if.h     shared pins, priorities, message formats
config/             kernel limits and device switches
build_make/         the make build
testing/            test logs per subsystem, plus integration, system and PC tests
docs/               the template's original guide (toolchain set-up in detail)
kernel/ lib/ include/ device/ etc/   micro T-Kernel and its drivers (template code, do not edit)
TEAM.md             owners, pins, peripherals, task priorities, interfaces
```

Inside each subsystem folder: `<name>.c/.h` create its kernel objects and start its tasks,
`task_*.c` are the tasks, `drv_*.c` touch the hardware, `*_logic.c` and `*_ctrl.c` hold the
logic that is tested on a PC, and `<name>_test.c` holds the on-target tests.

## Getting started

You need a Raspberry Pi Pico W, a USB cable, and macOS, Linux, or Windows with WSL
(Ubuntu). On Windows, run every command inside the WSL terminal.

### 1. Install the tools

```sh
# macOS
xcode-select --install
# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++ gcc-arm-none-eabi

# Pico C SDK (with the USB and WiFi driver submodules)
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk && git submodule update --init lib/tinyusb lib/cyw43-driver
```

On macOS, install the Arm GNU Toolchain (AArch32 bare-metal, `arm-none-eabi`) from
<https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>. Then add both
paths to `~/.zshrc` or `~/.bashrc` (change them to match your machine):

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-<version>/bin:$PATH"
export PICO_SDK_PATH="$HOME/pico/pico-sdk"
```

Open a new terminal and check that `arm-none-eabi-gcc --version` works.
More detail: [docs/TEMPLATE_README.md](docs/TEMPLATE_README.md), section 1.

### 2. Clone and build

```sh
git clone https://github.com/felixkohxiangshun/Embedded-System-Project.git
cd Embedded-System-Project/build_make
make -j8
```

The firmware is `build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`. For the first bring-up on the
car, build single-core with `make SMP=0 -j8`.

| Option | Values (default first) | Meaning |
|---|---|---|
| `SMP` | `1`, `0` | `1` uses both RP2040 cores, `0` core 0 only |
| `CONSOLE` | `usb_cdc`, `uart` | where `tm_printf` output goes: USB, or UART0 on GP0/GP1 |
| `WIFI` | `cyw43`, `none` | turn the Pico W radio on or off |
| `TEST` | `0`, `1` | `1` adds the on-target self-tests, run at power-up |
| `CRASH_REPORT` | `0`, `1` | also copies the console to UART0 (uses GP0/GP1) |

Options go after `make` and are part of the file name, for example
`mtk3pico_smp0_usb_cdc_wifi_test.uf2`. `make CONSOLE=uart WIFI=none -j8` needs no Pico SDK,
so it is a quick way to check that the code compiles.

### 3. Flash and watch the output

1. Hold **BOOTSEL**, plug the Pico W into USB, release. A drive called `RPI-RP2` appears.
2. Copy the `.uf2` file onto that drive. The Pico reboots by itself.
3. Open the serial console at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200     # macOS (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200           # Linux
```

On Windows use PuTTY or the VS Code Serial Monitor. Press the start button (GP20) to
start the car and press it again to stop. The status LED blinks slowly when idle.

### 4. Dashboard (WiFi and MQTT)

For the online dashboard build with `NET=lwip MQTT=1` and copy
`config/wifi_secrets.example.h` to `config/wifi_secrets.h`, then enter your network
details. That file is ignored by git, so never commit it. The link between the dashboard
task and the MQTT client is not finished yet (see below). Until then the telemetry JSON
is printed on the console instead.

## Testing

```sh
cd testing/host && make run        # 80 checks of the logic, on your computer
cd build_make && make TEST=1 -j8   # on-target tests: read the console for [TEST] ... PASS/FAIL
```

Record every test run in the owner's `testing/<folder>/TEST_LOG.md`, and tests that cross
subsystems in `testing/integration/` and `testing/system/`. How to write a test is in
[testing/README.md](testing/README.md).

## Working on the project

* Work in your own subsystem folder and keep the build free of compiler warnings (NFR-01).
* Claim pins and interfaces in [TEAM.md](TEAM.md) before you use them. Avoid GP23, GP24,
  GP25 and GP29 (Pico W radio). GP0 and GP1 are free unless you build with `CRASH_REPORT=1`.
* Commit small changes, pull before you push, and ask the owner before editing someone
  else's folder.
* Task priorities are 12 to 16 because the kernel uses 1 to 4. Do not change them without
  agreement from the team.

---

