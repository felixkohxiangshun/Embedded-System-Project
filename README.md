# Robo Pico Car: INF2004 Group 11

The aim of this project is to build an autonomous line-following car on micro T-Kernel 3.0.

| Feature | What the car does |
|---|---|
| Line following | Two IR sensors straddle the black line; a PD controller steers, and a lost line is searched for and picked up again |
| Barcodes | Reads Code 39 barcodes (`*A*` to `*Z*`, in either direction) with a third IR sensor and stores the letter |
| Junctions | At the next cross junction it carries out the move mapped to the stored letter: U-turn, left, right or straight on. The letter-to-move mapping is sent from the dashboard over MQTT (retained), not hard-coded |
| Humps | The IMU measures the height of each hump and keeps the tallest |
| Lift detection | If the car is picked up quickly or turned upside down, the IMU detects it and the car stops |
| Obstacles | An ultrasonic sensor on a servo stops the car at 30 cm, sweeps ±55° in 5° steps, measures the obstacle's width, picks the side with room and drives round it on a 45° dog-leg, then finds the line again |
| Dashboard | A web page shows live telemetry, line, barcode, obstacle and IMU data, and has Start/Stop and the junction-mapping controls |

Requirements (FR and NFR) and how the code meets them: [Requirements](#requirements).
Message formats and interfaces: [TEAM.md](TEAM.md), section 7, and
[testing/dashboard/CAR_DASHBOARD.md](testing/dashboard/CAR_DASHBOARD.md).
Videos of every test case: [videos/README.md](videos/README.md).

## Tech stack

| Layer | What we use |
|---|---|
| Language | C, built as `gnu11`; our code compiles with `-Wall -Wextra -Werror` (NFR-01). BARR-C:2018 (NFR-02): see [Requirements](#requirements) |
| RTOS | **micro T-Kernel 3.0**, dual-core RP2040 port |
| SDK | Raspberry Pi **Pico C SDK** 2.2.0 or newer (USB and WiFi driver source; 2.3.0 also builds without warnings) |
| Compiler | Arm GNU Toolchain, `arm-none-eabi-gcc` (tested with 15.3.Rel1) |
| Build | GNU `make` (`build_make/`), output is a `.uf2` file |
| Microcontroller | Raspberry Pi **Pico W** (RP2040, CYW43439 WiFi) on a Cytron **Robo Pico** |
| Communication | WiFi + MQTT (Mosquitto broker on a laptop), compact bit-packed binary messages: 11-byte telemetry every 500 ms, 17-byte debug every 100 ms, 8-byte IMU every 200 ms, 1-byte Start/Stop, 3-byte junction mapping (QoS 1, retained) |
| Dashboard | HTML and JavaScript page (`app_program/dashboard/web/`) connected to the broker over MQTT WebSocket |
| Tests | Unit tests on a PC (`testing/host`, `testing/line_barcode`, the dashboard page's `node --test`), on-target self-tests (`TEST=1`), manual test logs |

### Hardware

| Part | Used for | Pins |
|---|---|---|
| 2 DC motors with quadrature encoders | driving and speed feedback | motors GP8 to GP11, encoders GP2, GP3, GP6, GP7 |
| 2 IR line sensors | follow the line (they straddle it) and find junctions | GP26 (ADC0), GP27 (ADC1) |
| IR barcode sensor | reads the barcodes (digital DO output) | GP28 |
| LSM303DLHC accelerometer | hump height and lift detection | GP4 (SDA), GP5 (SCL), I2C0 |
| HC-SR04 ultrasonic sensor on a servo | obstacle detection, width and bypass side sensing | echo GP16, trigger GP17, servo GP12 |
| Start/stop button, NeoPixel status LEDs | operator control and state | GP20, GP18 |

The full pin table, with owners, is in [TEAM.md](TEAM.md). 

## Team and roles

| Member | Student ID | Subsystem (folder) | Responsible for |
|---|---|---|---|
| Tan Kun Yi Dave | 2502730 | Motor control (`app_program/motor_control/`) | PID speed control, wheel encoders, drift correction, 90° and 180° turns, start button |
| Vania Teng | 2503206 | Line following and barcode (`app_program/line_barcode/`) | IR sensors, 1 kHz sampler, line following and recovery, cross-junction detection, Code 39 decoding (A–Z, both directions) |
| Lau Sook Yee | 2503178 | IMU and terrain (`app_program/imu_terrain/`) | hump height, tallest hump, lift detection |
| Wong Jun Kai | 2500258 | Dashboard and telemetry (`app_program/dashboard/`) | binary telemetry and debug frames, WiFi and MQTT link, heartbeat, Start/Stop and junction mapping received over MQTT, web dashboard |
| Felix Koh | 2500256 | Mission and obstacle avoidance (`app_program/mission_obstacle/`) | system state machine, stored barcode letter carried out at cross junctions, ultrasonic and servo obstacle scan and width, bypass, system integration |

Each subsystem has its own folder and its own test log in `testing/<folder>/TEST_LOG.md`.
Only the owner edits their folder. Shared files (`app_program/app.h`, `app_if.h`,
`app_main.c`, `config/`, `TEAM.md`) are changed by agreement of the team.
Priorities, interfaces and the full task list are in [TEAM.md](TEAM.md).

## Requirements

These are the functional (FR) and non-functional (NFR) requirements as revised for the
Tribunal 2 pre-submission (23 Nov 2026). Some requirements has been partially changed to
improve functionality. In the table, **Met** = the code does what the requirement says;
**Differs** = the code does it another way (see the note); **Test** = a measured target,
see the owner's `testing/<folder>/TEST_LOG.md`.

### Functional requirements

| ID | Requirement (short) | Owner | Status | Note |
|---|---|---|---|---|
| FR-01 | Follow the line between the two line IR sensors, PD steering every 10 ms | Vania | Met | |
| FR-02 | Decode Code 39 A–Z both ways; back up 50 mm, read at 18 mm/s, retry once in reverse | Vania, Felix | Met | |
| FR-03 | Store the letter; at the next cross junction do its mapped move; none or unmapped = forward | Vania, Felix | Met | Mapping sent over MQTT; a letter for straight can be set too |
| FR-04 | Detect each hump under one drive wheel | Sook Yee | Met | |
| FR-05 | Hump height from roll (track × tan roll), keep the tallest, slow to 18 mm/s on a hump | Sook Yee | Met | |
| FR-06 | Lifted and upside down (100 ms) → all motors stop | Sook Yee | Met | Also stops when picked up quickly (upward speed about 0.4 m/s) |
| FR-07 | Detect an obstacle at ≤ 30 cm (two readings in a row) and stop | Felix | Met | |
| FR-08 | Sweep seven angles (±70°, ±45°, ±22.5°, 0°) and choose a side | Felix | Differs | 23 readings in 5° steps over ±55°; side and width from the obstacle's edges (LordPercivalXII method, calibrated); first "no way round" creeps 10 cm closer |
| FR-09 | 45° dog-leg; straight leg ends 3 readings after nothing within 35 cm, then 25 cm | Felix, Dave | Differs | Straight leg ends after 1.5 s with nothing within 35 cm; outward diagonal 10 cm shorter than the formula (tuned on the floor); 55° is commanded for each 45° turn |
| FR-10 | Turn in, stop on the second IR sensor, 10 cm on, turn along the line, creep to re-find it | Felix, Vania | Met | |
| FR-11 | Wheel speed PID: 78 mm/s line, 18 mm/s barcode, 65 mm/s bypass | Dave | Met | |
| FR-12 | Distance, speed and heading from the encoders | Dave | Met | |
| FR-13 | 90°, 180° and bypass turns by encoder count | Dave | Met | |
| FR-14 | WiFi and MQTT with back-off, never blocking a control task | Jun Kai | Met | Back-off 1 to 16 s |
| FR-15 | Live state to the dashboard; commands on a QoS 1 topic | Jun Kai | Met | |
| FR-16 | 11-byte telemetry every 500 ms (QoS 0), 24-byte debug frame every 100 ms | Jun Kai | Differs | Debug frame is now 17 bytes (unused sweep fields removed); an 8-byte IMU frame is added |
| FR-17 | Start with the GP20 button | Dave, Felix | Met | Start/Stop from the dashboard too |
| FR-18 | Heartbeat while no telemetry is sent | Jun Kai | Met | |
| FR-19 | IMU or WiFi failure: keep driving; line lost, encoder, scan fault or line not re-found in 5 s: stop, wait in IDLE | All | Met | |

### Non-functional requirements

| ID | Requirement (short) | Status | Note |
|---|---|---|---|
| NFR-01 | Builds with no warnings under `-Wall -Wextra -Werror` | Met | Every file in `app_program/` is compiled with these flags (`APP_WARN` in `build_make/mtkernel_3/app_program/subdir.mk`); the kernel and SDK keep their own flags. The PC tests use them too |
| NFR-02 | BARR-C:2018 | Differs | Every file has the BARR-C layout: spaces, no tabs, a Doxygen `@file` header and the end-of-file footer. `line_barcode` passes `testing/line_barcode/check_style.py` (6 findings); the other four folders still have about 900, mostly lines over 80 columns and the constant on the right of `==`. They keep the kernel's `INT`/`UB` types, which its calls use |
| NFR-03 | Line task every 10 ms | Met | |
| NFR-04 | IR sampler at 1 kHz | Met | 1 ms cyclic handler |
| NFR-05 | Barcodes decode in ≥ 95 % of passes | Test | |
| NFR-06 | IMU every 10 ms, even when the sensor fails | Met | |
| NFR-07 | Hump height ±5 mm | Test | |
| NFR-08 | Obstacles within 30 cm detected in ≥ 95 % of runs | Test | |
| NFR-09 | Never touch the obstacle; planned sideways gap ≥ 4 cm | Differs | The 10 cm diagonal trim leaves a planned gap below 4 cm on paper; the real gap depends on the turns, so measure it on the floor |
| NFR-10 | Bypass within 30 s; a slower one is recorded as late | Met | `BYPASS_TARGET_MS` |
| NFR-11 | Line re-found after a bypass in ≥ 95 % of 20 runs | Test | |
| NFR-12 | Telemetry every 500 ms; WiFi loss never stops the car | Met | |
| NFR-13 | Minimum-bit data: bit-packed frames, 1-byte command, 10-bit line readings, one 8-bit fault mask | Met | |
| NFR-14 | Motor PID every 10 ms | Met | |
| NFR-15 | 90° and 180° turns within ±5° | Test | |

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
  dashboard/          broker launcher, dashboard build script, CAR_DASHBOARD.md
videos/             links to the test case videos (YouTube)
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

If you already have SDK 2.3.0 installed you can keep it: every build option below
compiles with it, with no warnings.

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

The firmware is `build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`. Build the car single-core
(`make SMP=0 -j8`): in the dual-core build the motor task never ran. For the full car with
the dashboard, use the dashboard build in step 4.

| Option | Values (default first) | Meaning |
|---|---|---|
| `SMP` | `1`, `0` | `1` uses both RP2040 cores, `0` core 0 only |
| `CONSOLE` | `usb_cdc`, `uart` | where `tm_printf` output goes: USB, or UART0 on GP0/GP1 |
| `WIFI` | `cyw43`, `none` | turn the Pico W radio on or off |
| `TEST` | `0`, `1` | `1` adds the on-target self-tests, run at power-up |
| `CRASH_REPORT` | `0`, `1` | also copies the console to UART0 (uses GP0/GP1) |

Options go after `make` and are part of the file name, for example
`mtk3pico_smp0_usb_cdc_wifi_test.uf2`. `make CONSOLE=uart WIFI=none -j8` compiles every file
without the Pico SDK, a quick check for compile errors, but it does not link: the NeoPixel
driver (`drv_rgb.c`) needs the SDK's PIO functions.

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

1. Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and enter the WiFi
   name and password, the laptop's address (`MQTT_BROKER_IP`) and a client ID. That file is
   ignored by git: never commit it.
2. Start the broker (Mosquitto, port 1883 for the car, WebSocket 9001 for the page):
   `python3 testing/dashboard/broker/start_lan_broker.py`. It prints the address to use.
3. Build and flash the car firmware with the dashboard link:
   `sh testing/dashboard/broker/build_dashboard.sh`, then copy
   `build_make/mtk3pico_dashboard.uf2` to the Pico (or flash the `.elf` with a debug probe).
4. Serve the page: `python3 app_program/dashboard/web/serve.py`, then open
   <http://127.0.0.1:8765/car.html>. The badge turns green ("Car live") once frames arrive.
5. Before the run, type the junction letters (U-turn, left, right, straight) and press
   **Send mapping**. The broker keeps it, so the car gets it again whenever it reconnects.
   Start and Stop work from the page as well as from the GP20 button.

Topics, frame layouts and troubleshooting:
[testing/dashboard/CAR_DASHBOARD.md](testing/dashboard/CAR_DASHBOARD.md). Without WiFi the
car still drives; Mission leaves INIT after 20 s.

## Testing

```sh
cd testing/host && make run        # logic tests on your computer: 226 checks, then the
                                   # line/barcode tests (63 checks and the digital barcode cases)
node --test app_program/dashboard/web/car_model.test.mjs   # dashboard page decoders
cd build_make && make TEST=1 -j8   # on-target tests: read the console for [TEST] ... PASS/FAIL
```

Record every test run in the owner's `testing/<folder>/TEST_LOG.md`, and tests that cross
subsystems in `testing/integration/` and `testing/system/`. How to write a test is in
[testing/README.md](testing/README.md).
