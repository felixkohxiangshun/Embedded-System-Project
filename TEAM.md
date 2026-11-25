# Team and subsystems — Group 11 robo car

Who owns what in the final design: the Week 6 design review as revised for the
Tribunal 2 pre-submission (23 Nov 2026), whose FR and NFR numbers are used below and
in [README.md](README.md#requirements). One folder per subsystem;
the owner is the only person who edits it. Shared files (`app_program/app.h`,
`app_program/app_if.h`, `app_program/app_main.c`, `config/`, this file) are changed
by agreement of the team.

## 1. Members and subsystems

| Subsystem | Owner | Student ID | GitHub | Code | Tests |
|---|---|---|---|---|---|
| Motor control: PID speed loop, wheel encoders, start button interrupt | Tan Kun Yi Dave | 2502730 | *(add)* | `app_program/motor_control/` | `testing/motor_control/` |
| Line following and barcode: two line IR sensors and the barcode IR sensor, 1 kHz sampler, Code 39 decoding (A–Z, both directions), cross-junction detection | Vania Teng | 2503206 | *(add)* | `app_program/line_barcode/` | `testing/line_barcode/` |
| IMU and terrain: hump height, tallest hump, lift detection | Lau Sook Yee | 2503178 | *(add)* | `app_program/imu_terrain/` | `testing/imu_terrain/` |
| Online dashboard: telemetry task, WiFi, MQTT, compact binary telemetry, web page, Start/Stop and junction mapping received over MQTT | Wong Jun Kai | 2500258 | *(add)* | `app_program/dashboard/` | `testing/dashboard/` |
| Mission state, stored barcode letter carried out at cross junctions, obstacle scanning, width and avoidance, integration | Felix Koh | 2500256 | *(add)* | `app_program/mission_obstacle/` | `testing/mission_obstacle/` |

Test IDs in the logs use short codes: S1 = motor_control, S2 = line_barcode,
S3 = imu_terrain, S4 = dashboard, S5 = mission_obstacle.

### Responsibilities

| Owner | Requirements | Task(s) and priority | Statechart | Assumptions | Also responsible for (report) |
|---|---|---|---|---|---|
| Dave | FR-11, FR-12, FR-13, NFR-14, NFR-15; FR-17 (button interrupt); the turns of FR-03 and FR-09 | `motor_task` 12 | 6.3 Motor | A5 battery, A6 wheel slip | motor pin layout, PID/control task, motion statechart |
| Vania | FR-01, FR-02, FR-03 (decode A–Z in both directions; cross-junction detection), FR-10 (line found), NFR-03, NFR-04, NFR-05, NFR-13 (10-bit line readings) | `line_task` 13, `barcode_task` 17 | 6.1 Line and barcode | A3 IR contrast | line/barcode requirements, sensor calibration, navigation interface |
| Sook Yee | FR-04, FR-05, FR-06, NFR-06, NFR-07 | `imu_task` 16 | 6.2 IMU | A4 hump vs vibration | IMU interface, terrain requirements, terrain statechart |
| Jun Kai | FR-14, FR-15, FR-16, FR-18, NFR-12, NFR-13 (frames and command) | `dashboard_task` 19, `telemetry_task` 18 | 6.5 Dashboard | A7, A8 WiFi / MQTT | communication requirements, telemetry interface, WiFi task |
| Felix | FR-03 (store the decoded character, carry out its mapped movement at the next cross junction), FR-07, FR-08, FR-09, FR-10, FR-17 (Start/Stop handling), FR-19, NFR-08..NFR-11, NFR-13 (fault bitmask) | `mission_task` 14, `scan_task` 15 | 5 System, 6.4 Obstacle scan | A1 ultrasonic range, A2 shared priority | system architecture, task interaction diagram, obstacle scanning statechart |

## 2. Pins

Pin assignments follow the design review (section 2); each pin has one owner. `GP0` and
`GP1` are free and become UART0 only with `make CRASH_REPORT=1`.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | free (UART0 only with `CRASH_REPORT=1`) |
| GP1 | | free (UART0 only with `CRASH_REPORT=1`) |
| GP2 | Dave | `ENC_LEFT_A` quadrature encoder, left motor |
| GP3 | Dave | `ENC_LEFT_B` |
| GP4 | Sook Yee | `IMU_SDA` (I2C0) |
| GP5 | Sook Yee | `IMU_SCL` (I2C0) |
| GP6 | Dave | `ENC_RIGHT_A` quadrature encoder, right motor |
| GP7 | Dave | `ENC_RIGHT_B` |
| GP8 | Dave | `MOT_LEFT_FWD` (PWM slice 4 A) |
| GP9 | Dave | `MOT_LEFT_BWD` (PWM slice 4 B) |
| GP10 | Dave | `MOT_RIGHT_FWD` (PWM slice 5 A) |
| GP11 | Dave | `MOT_RIGHT_BWD` (PWM slice 5 B) |
| GP12 | Felix | `SERVO_PWM` (PWM slice 6 A) |
| GP13 | | free |
| GP14 | | free |
| GP15 | | free |
| GP16 | Felix | `ULTRA_ECHO` (HC-SR04, **5 V: needs a divider**) |
| GP17 | Felix | `ULTRA_TRIG` |
| GP18 | Felix | `LED_STATUS`: the Robo Pico NeoPixel RGB LEDs (PIO0, `drv_rgb.c`) |
| GP19 | | free |
| GP20 | Dave | `BTN_START` (active low; read by the GPIO interrupt) |
| GP21 | | `BTN_2`: only the bench test modes use it (servo calibration, motor forward test) |
| GP22 | | free |
| GP26 | Vania | `IR_RIGHT` (ADC0): right sensor of the line pair |
| GP27 | Vania | `IR_CENTER` (ADC1): left sensor of the line pair |
| GP28 | Vania | `IR_BARCODE`: barcode sensor, digital DO output read as a GPIO with an edge interrupt |

The pin numbers in `app_program/app.h` (`PIN_...`) must match this table.

## 3. Peripherals

Each peripheral is driven by exactly one owner; every other task reaches it through a
kernel object (section 5) and never touches it.

| Peripheral | Owner subsystem | Owning code | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 | Sook Yee | `imu_task` (through the kernel I2C driver, device `iica`) | GP4, GP5 |
| I2C1 | | not initialised | |
| SPI0 / SPI1 | | | |
| ADC | Vania | `ir_sampler_isr` (1 ms cyclic handler) — see note | GP26, GP27 |
| PWM slices 4, 5 | Dave | `motor_task` | GP8–GP11 |
| PWM slice 6 | Felix | `scan_task` | GP12 |
| GPIO interrupt (IO bank 0, NVIC 13) | Dave | `gpio_bank0_handler` (encoders and button; forwards the ECHO and barcode edges) | GP2, GP3, GP6, GP7, GP16, GP20, GP28 |
| HC-SR04 GPIO | Felix | `scan_task` (TRIG, 10 µs pulse); ECHO edges via `ultra_echo_isr()` — see note | GP16, GP17 |
| PIO0 (NeoPixel status LEDs) | Felix | `mission_task` (`drv_rgb.c`) | GP18 |
| WiFi radio, TCP/IP | *kernel* | `cyw43_utk_task` and lwIP (built in); the MQTT client is Jun Kai's `mqtt_bridge.c` | GP23–25, GP29 |

**ADC note.** The RP2040 has one converter behind a multiplexer, so the line
sensors cannot be read by separate tasks. The owner is the 1 kHz sampler handler,
which reads both line channels (cut to 10 bits, NFR-13) and averages them over
10 ms; `line_task` reads the filtered values. The barcode sensor is read through its
digital output on GP28: each change raises the GPIO interrupt and is timestamped,
and the 1 kHz handler only confirms a level that has stopped changing. The kernel's own ADC
driver is switched off (`DEVCNF_USE_ADC 0` in `config/config_device.h`). The kernel
I2C driver is switched on (`DEVCNF_USE_IIC 1`, pins GP4/GP5).

**GPIO interrupt and the echo pin (Tribunal 1).** The RP2040 has one interrupt line for
all of GPIO bank 0, so Dave's `gpio_bank0_handler` stays the only handler. It stamps the
timer on entry and forwards the ultrasonic ECHO edges (GP16) to `ultra_echo_isr()`
in `mission_obstacle/drv_ultra.c` (declared in `app.h`, interrupt context, never blocks).
Felix's `ultra_hw_init()` enables the two GP16 edges; Dave's file only calls the hook.
`scan_task` pulses TRIG, then blocks on its private event flag `flg_echo` with a 30 ms
timeout (timeout = no echo), so it no longer busy-waits for the echo. It also forwards
the barcode edges (GP28) to Vania's `bc_level` debouncer.

**Timer note.** `APP_NOW_US()` does not count at 1 MHz on this board: nothing starts the
RP2040 watchdog tick, so the timer counts at 23.4 kHz (one count = 128/3 µs). The echo
and barcode edge times are converted with `TIMER_COUNTS_TO_US` (`scan_logic.h`).

### WiFi and MQTT

No student task calls `cyw43_...`, lwIP or MQTT functions directly. The dashboard
build (`testing/dashboard/broker/build_dashboard.sh`, `-DDASH_MQTT_BRIDGE=1`) replaces
the template's demo MQTT client with `app_program/dashboard/mqtt_bridge.c`, which runs in
the lwIP context: it connects to the broker (reconnecting with 1 to 16 s back-off,
FR-14), publishes what `dashboard_task` puts in `mbf_mqtt_out`, and puts received
commands in `mbf_mqtt_in` (IF-12). Both sides use `TMO_POL`, so a WiFi problem never
blocks a control task (NFR-12).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4), so the
design review's ranking 1–5 is shifted into the application band. Instead of one
band per subsystem (the template's default) the priorities follow the
**criticality order of the design review**, which spans subsystems.

**Tribunal 1 change.** Every task now has its own priority: the kernel does not
time-slice tasks of equal priority, so the old shared "Medium (4)" level (IMU, Scan,
Barcode) let one task hold the CPU until it blocked, and the ultrasonic busy-wait
could do that for up to ~17 ms per ping. Inside that rank the most urgent event goes
first (2026-11-23): an obstacle ahead, then humps and lifts, then barcode decoding,
whose edges are already timestamped and buffered (64 edges in `mbf_edge`), so a short
wait costs no accuracy. The Week 6 order of the ranks is unchanged.

| Review rank | Task | Subsystem | Priority | Trigger | Period | WCET | Core |
|---|---|---|---|---|---|---|---|
| Highest (1) | `motor_task` | Dave | 12 | cyclic 10 ms | 10 ms | *not measured* | any |
| High (2) | `line_task` | Vania | 13 | cyclic 10 ms | 10 ms | *not measured* | any |
| Medium (3) | `mission_task` | Felix | 14 | 5 ms poll of its inputs | 5 ms | *not measured* | any |
| Medium (4) | `scan_task` | Felix | 15 | 60 ms ping timer / commands; blocks on the echo interrupt (≤ 30 ms) | 60 ms | *not measured* | any |
| Medium (4) | `imu_task` | Sook Yee | 16 | cyclic 10 ms | 10 ms | *not measured* | any |
| Medium (4) | `barcode_task` | Vania | 17 | blocks on `mbf_edge` | event (a capture ends after max(2.5 s, 4 × its longest element), at most 6 s, of quiet) | *not measured* | any |
| Low (5) | `telemetry_task` | Jun Kai | 18 | 500 ms | 500 ms | *not measured* | any |
| Low (5) | `dashboard_task` | Jun Kai | 19 | 100 ms | 100 ms | *not measured* | any |

WCET (worst-case execution time) is measured on the board by each owner and filled
in here; record the method and the raw numbers in your `TEST_LOG.md`.

Handlers (not tasks): `ir_sampler_isr` 1 ms, and the cyclic wake-up handlers for
Motor, Line and IMU. Interrupt: `gpio_bank0_handler`. The system tick is 1 ms
(`CNF_TIMER_PERIOD 1` in `config/config.h`), needed for the 1 kHz sampler.

## 5. Interfaces between subsystems

The **provider** (receiver or owner) creates the object; the **user** only sends to
it or waits on it. Message formats are in `app_program/app_if.h`.

| ID | From → To | Mechanism | Object | Data / meaning | Owners (from → to) |
|---|---|---|---|---|---|
| IF-01 | Line → Mission | message buffer (owned by Felix) | `mbf_steer` | `MSG_STEER`: steer −1000..+1000, flags LOST / TIMEOUT / JUNCTION (cross junction found, sent once per junction) / DARK_C / DARK_R (which sensor sees black, for the bypass return) | Vania → Felix |
| IF-02 | 1 kHz handler → Barcode | message buffer (owned by Vania) | `mbf_edge` | `MSG_EDGE`: timestamp (µs) and level of each barcode edge | Vania |
| IF-03 | Barcode → Mission | message buffer (owned by Felix) | `mbf_bccmd` | `MSG_BCCMD`: FIRST (a code starts) / SEEN / OK(A–Z, either scan direction) / FAIL | Vania → Felix |
| IF-04 | IMU → Mission | message buffer (owned by Felix) | `mbf_terrain` | `MSG_TERRAIN`: hump start / hump done (height, tallest) / lifted / fault / recovered | Sook Yee → Felix |
| IF-05 | Scan → Mission | message buffer (owned by Felix) | `mbf_obs` | `MSG_OBS`: detected / result (side, distance, width, sideways reach) / fail (no way round, less-blocked side) / clear / range (one side reading on the bypass straight leg) | Felix |
| IF-06 | Mission → Scan | message buffer (owned by Felix) | `mbf_scancmd` | `MSG_SCANCMD`: ARM / DISARM / PROFILE / TRACK (hold the servo at an angle and report every reading) | Felix |
| IF-07 | Dashboard → Mission | message buffer (owned by Felix) | `mbf_mqttcmd` | `MSG_MQTTCMD`: START / STOP, or MAP with the letters for U-turn, left, right and straight (unpacked from the binary commands, section 7) | Jun Kai → Felix |
| IF-08 | Mission → Motor | message buffer (owned by Dave) | `mbf_nav` | `MSG_NAV`: steer, slow, resume, turn (degrees), move (mm, either direction, at a given speed) | Felix → Dave |
| IF-09 | Telemetry → Dashboard | message buffer (owned by Jun Kai) | `mbf_telem` | `MSG_TELEM` frame every 500 ms | Jun Kai (both ends) |
| IF-10 | Mission / button ISR → Motor, Mission, Barcode | event flag (owned by Dave) | `flg_evt` | START, STOP, RESUME (to Motor); BTN (ISR to Mission); BC_HOLD, BC_ARM (Mission to Barcode, around the back-up over a code) | Felix, Dave, Vania |
| IF-11 | Motor, Mission → Mission, Telemetry, Dashboard | shared data under a mutex with priority inheritance (owned by Felix) | `app_state`, `mtx_state` | `APP_STATE`: system/motor state, speed, distance, results (incl. the stored character and the current mapping), network status | Dave, Felix, Jun Kai |
| IF-12 | Dashboard ↔ MQTT bridge | two message buffers (owned by Jun Kai) | `mbf_mqtt_out`, `mbf_mqtt_in` | binary frames to publish; broker events and binary command payloads (section 7) | Jun Kai |

Why message buffers and not queues: micro T-Kernel has no data queue. A message
buffer copies the message in and out and has bounded size; a mailbox passes only a
pointer. Sends use `TMO_POL`, so a handler or control loop never blocks; a full
buffer drops the message.

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. The kernel's own drivers (serial, I2C, USB,
WiFi) also use a few objects, so the message-buffer limit was raised from 12 to 24.

| Object | Limit | Dave | Vania | Sook Yee | Jun Kai | Felix | Total |
|---|---|---|---|---|---|---|---|
| Task | 32 | 1 | 2 | 1 | 2 | 2 | 8 |
| Semaphore | 16 | | | | | | 0 |
| Event flag | 16 | 1 | | | | 1 | 2 |
| Mailbox | 12 | | | | | | 0 |
| Mutex | 12 | | | | | 1 | 1 |
| Message buffer | 24 | 1 | 1 | | 3 | 6 | 11 |
| Fixed memory pool | 12 | | | | | | 0 |
| Variable memory pool | 12 | | | | | | 0 |
| Cyclic handler | 12 | 1 | 2 | 1 | | | 4 |
| Alarm handler | 12 | | | | | | 0 |

Interrupt handler defined: IO bank 0 (NVIC 13), by Dave.

## 7. Barcode commands and MQTT formats

These follow the demo requirements (FR-02, FR-03, FR-15, FR-16, NFR-13). Every frame is
packed with Jun Kai's `protocol_bits`: each field uses only the bits its largest value
needs, in the order listed, **least significant bit first** from bit 0 of byte 0. Signed
values are sent with an offset; values out of range are clamped, never wrapped; spare
bits are 0. Packing lives in `app_program/dashboard/dash_logic.c` and decoding on the
page in `app_program/dashboard/web/car_model.mjs`; both are tested.

### Demo behaviour

1. The car scans a barcode in either direction and decodes any letter A–Z (Vania). On
   the first bars Mission backs up 50 mm and the code is read at 18 mm/s; a failed read
   is retried once in reverse (FR-02).
2. Mission stores the letter; a newer barcode replaces an older one (Felix).
3. The car keeps following the line until it detects a cross junction (Vania).
4. At the cross junction Mission drives on for 1.5 s so the car is over the junction,
   then carries out the movement mapped to the stored letter: U-turn, left, right or
   straight (Felix; the turn itself is Dave's `NAV_TURN`).
5. The stored letter is cleared and the car carries on to the next barcode.

No barcode since the last junction, or a letter that is not in the mapping: the car goes
straight. The letter-to-movement mapping is **not hard-coded**: the dashboard sends it
over MQTT before the run.

### Topics

| Topic | Direction | Size | When | QoS |
|---|---|---|---|---|
| `csc11/dev/car/telemetry` | car → PC | 11 bytes | every 500 ms | 0 |
| `csc11/dev/car/debug` | car → PC | 17 bytes | every 100 ms | 0 |
| `csc11/dev/car/imu` | car → PC | 8 bytes | every 200 ms | 0 |
| `csc11/dev/car/barcode` | car → PC | 3 + 2 per element | once per barcode capture | 0 |
| `csc11/dev/car/heartbeat` | car → PC | 1 byte (system state) | every 1 s while telemetry stops | 0 |
| `csc11/dev/car/cmd` | PC → car | 1 byte | on Start / Stop | 1 |
| `csc11/dev/car/map` | PC → car | 3 bytes, retained | on Send mapping | 1 |

Field lists for the debug, IMU and barcode frames:
[testing/dashboard/CAR_DASHBOARD.md](testing/dashboard/CAR_DASHBOARD.md).

### Command: Start / Stop, 1 byte

Bit 0 = 1 for START, 0 for STOP; bits 1–7 must be 0. Any other payload or length is
ignored. Remote E-stop and RESUME were removed (with the `E_STOP` system state); the
GP20 button does the same as Start / Stop.

### Command: junction mapping, 3 bytes

Published retained, so the broker gives it to the car again whenever it reconnects,
even before Start.

| Bits | Field |
|---|---|
| 5 | letter for **U-turn** |
| 5 | letter for **left** |
| 5 | letter for **right** |
| 5 | letter for **straight** |
| 4 | spare, must be 0 |

Letters are A = 0 … Z = 25; 31 = no letter. The car rejects the whole command if a code
is 26–30, a spare bit is set, the length is not 3, or one letter is given to two moves.
Example: U-turn Z, left none, right Q, straight S = `0xF9 0x43 0x09`.

### Telemetry: 11 bytes every 500 ms

| Field | Range and resolution | Bits |
|---|---|---|
| sequence number | 0–255, wraps (lets the PC count lost frames) | 8 |
| system state | 0–15 (`SYS_xxx`) | 4 |
| speed | −256 to +255 mm/s (sent + 256) | 9 |
| distance travelled | 0–65 535 mm, wraps | 16 |
| heading | 0–359° | 9 |
| line seen | 0 / 1 | 1 |
| obstacle distance | 0–255 cm, 0 = none | 8 |
| last letter read | A–Z = 0–25, 31 = none | 5 |
| stored letter for the next junction | A–Z = 0–25, 31 = none | 5 |
| tallest hump | 0–127 mm | 7 |
| lifted | 0 / 1 | 1 |
| fault flags | the 8 `FAULT_xxx` bits | 8 |
| spare | 0 | 7 |
| **total** | | **88 (11 bytes)** |

Fault bits (one owner each, written under `mtx_state` with priority inheritance):
IMU 0x01, encoder 0x02, line 0x04, scan 0x08, motor stall 0x10, WiFi 0x20, hump out
of range 0x40, Mission timeout 0x80.
