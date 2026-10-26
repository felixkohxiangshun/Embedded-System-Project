# Team and subsystems — Group 11 robo car

Who owns what in the final design (Week 6 Design Review). One folder per subsystem;
the owner is the only person who edits it. Shared files (`app_program/app.h`,
`app_program/app_if.h`, `app_program/app_main.c`, `config/`, this file) are changed
by agreement of the team.

## 1. Members and subsystems

| Subsystem | Owner | Student ID | GitHub | Code | Tests |
|---|---|---|---|---|---|
| Motor control: PID speed loop, wheel encoders, start button interrupt | Tan Kun Yi Dave | 2502730 | *(add)* | `app_program/motor_control/` | `testing/motor_control/` |
| Line following and barcode: three IR sensors, 1 kHz sampler, Code 39 decoding | Vania Teng | 2503206 | *(add)* | `app_program/line_barcode/` | `testing/line_barcode/` |
| IMU and terrain: hump height, tallest hump, shock detection | Lau Sook Yee | 2503178 | *(add)* | `app_program/imu_terrain/` | `testing/imu_terrain/` |
| Online dashboard: telemetry task, WiFi, MQTT, telemetry JSON, remote commands | Wong Jun Kai | 2500258 | *(add)* | `app_program/dashboard/` | `testing/dashboard/` |
| Mission state, obstacle scanning and avoidance, integration | Felix Koh | 2500256 | *(add)* | `app_program/mission_obstacle/` | `testing/mission_obstacle/` |

Test IDs in the logs use short codes: S1 = motor_control, S2 = line_barcode,
S3 = imu_terrain, S4 = dashboard, S5 = mission_obstacle.

### Responsibilities

| Owner | Requirements | Task(s) and priority | Statechart | Assumptions | Also responsible for (report) |
|---|---|---|---|---|---|
| Dave | FR-11, FR-12, FR-13, NFR-14, NFR-15; FR-03 (turns), FR-09 (bypass motion) | `motor_task` 12 | 6.3 Motor | A5 battery, A6 wheel slip | motor pin layout, PID/control task, motion statechart |
| Vania | FR-01, FR-02, FR-03 (decode), FR-10 (line), NFR-03, NFR-04, NFR-05 | `line_task` 13, `barcode_task` 16 | 6.1 Line and barcode | A3 IR contrast | line/barcode requirements, sensor calibration, navigation interface |
| Sook Yee | FR-04, FR-05, FR-06, NFR-06, NFR-07 | `imu_task` 15 | 6.2 IMU | A4 hump vs vibration | IMU interface, terrain requirements, terrain statechart |
| Jun Kai | FR-14..FR-18 (incl. FR-16 telemetry), NFR-12, NFR-13 | `dashboard_task` 19, `telemetry_task` 18 | 6.5 Dashboard | A7, A8 WiFi / MQTT | communication requirements, telemetry interface, WiFi task |
| Felix | FR-07, FR-08, FR-09, FR-10, FR-17, FR-19, NFR-08..NFR-11 | `mission_task` 14, `scan_task` 17 | 5 System, 6.4 Obstacle scan | A1 ultrasonic range, A2 shared priority | system architecture, task interaction diagram, obstacle scanning statechart |

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
| GP18 | Felix | `LED_STATUS` |
| GP19 | | free |
| GP20 | Dave | `BTN_START` (active low; read by the GPIO interrupt) |
| GP21 | | `BTN_2`, not used yet |
| GP22 | | free |
| GP26 | Vania | `IR_LEFT` (ADC0) |
| GP27 | Vania | `IR_CENTER` (ADC1) |
| GP28 | Vania | `IR_RIGHT` (ADC2, barcode) |

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
| ADC | Vania | `ir_sampler_handler` (1 ms cyclic handler) — see note | GP26–GP28 |
| PWM slices 4, 5 | Dave | `motor_task` | GP8–GP11 |
| PWM slice 6 | Felix | `scan_task` | GP12 |
| GPIO interrupt (IO bank 0, NVIC 13) | Dave | `gpio_bank0_handler` (encoders and button; forwards ECHO edges) | GP2, GP3, GP6, GP7, GP16, GP20 |
| HC-SR04 GPIO | Felix | `scan_task` (TRIG, 10 µs pulse); ECHO edges via `ultra_echo_isr()` — see note | GP16, GP17 |
| Status LED | Felix | `mission_task` | GP18 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

**ADC note.** The RP2040 has one converter behind a multiplexer, so the three IR
sensors cannot be read by separate tasks. The owner is the 1 kHz sampler handler,
which reads all three channels; `line_task` reads its filtered values. This
differs from the template's "owning *task*" wording and from the design review,
which has the 1 kHz handler sampling only the barcode sensor. The kernel's own ADC
driver is switched off (`DEVCNF_USE_ADC 0` in `config/config_device.h`). The kernel
I2C driver is switched on (`DEVCNF_USE_IIC 1`, pins GP4/GP5).

**GPIO interrupt and the echo pin (Tribunal 1).** The RP2040 has one interrupt line for
all of GPIO bank 0, so Dave's `gpio_bank0_handler` stays the only handler. It stamps the
1 MHz timer on entry and forwards the ultrasonic ECHO edges (GP16) to `ultra_echo_isr()`
in `mission_obstacle/drv_ultra.c` (declared in `app.h`, interrupt context, never blocks).
Felix's `ultra_hw_init()` enables the two GP16 edges; Dave's file only calls the hook.
`scan_task` pulses TRIG, then blocks on its private event flag `flg_echo` with a 30 ms
timeout (timeout = no echo), so it no longer busy-waits for the echo.

### WiFi and MQTT

No student task may call `cyw43_...`, lwIP or MQTT functions. Subsystem 4 only reads
`cyw43_utk_get_status()` and exchanges strings through two message buffers
(IF-12). The matching change inside `mqtt_utk.c` (read `mbf_mqtt_out`, write
`mbf_mqtt_in`, both with `TMO_POL`) is **not made yet**: it edits shared library
code and needs the team's and the instructor's agreement first.

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4), so the
design review's ranking 1–5 is shifted into the application band. Instead of one
band per subsystem (the template's default) the priorities follow the
**criticality order of the design review**, which spans subsystems.

**Tribunal 1 change.** Every task now has its own priority: the kernel does not
time-slice tasks of equal priority, so the old shared "Medium (4)" level (IMU, Scan,
Barcode) let one task hold the CPU until it blocked, and the ultrasonic busy-wait
could do that for up to ~17 ms per ping. Inside one review rank the task with the
shorter period goes first (rate monotonic). The Week 6 order of the ranks is unchanged.

| Review rank | Task | Subsystem | Priority | Trigger | Period | WCET | Core |
|---|---|---|---|---|---|---|---|
| Highest (1) | `motor_task` | Dave | 12 | cyclic 10 ms | 10 ms | *not measured* | any |
| High (2) | `line_task` | Vania | 13 | cyclic 10 ms | 10 ms | *not measured* | any |
| Medium (3) | `mission_task` | Felix | 14 | 5 ms poll of its inputs | 5 ms | *not measured* | any |
| Medium (4) | `imu_task` | Sook Yee | 15 | cyclic 10 ms | 10 ms | *not measured* | any |
| Medium (4) | `barcode_task` | Vania | 16 | blocks on `mbf_edge` | event (capture ends after 120 ms silence) | *not measured* | any |
| Medium (4) | `scan_task` | Felix | 17 | 60 ms ping timer / commands; blocks on the echo interrupt (≤ 30 ms) | 60 ms | *not measured* | any |
| Low (5) | `telemetry_task` | Jun Kai | 18 | 500 ms | 500 ms | *not measured* | any |
| Low (5) | `dashboard_task` | Jun Kai | 19 | 100 ms | 100 ms | *not measured* | any |

WCET (worst-case execution time) is measured on the board by each owner and filled
in here; record the method and the raw numbers in your `TEST_LOG.md`.

Handlers (not tasks): `ir_sampler_handler` 1 ms, and the cyclic wake-up handlers for
Motor, Line and IMU. Interrupt: `gpio_bank0_handler`. The system tick is 1 ms
(`CNF_TIMER_PERIOD 1` in `config/config.h`), needed for the 1 kHz sampler.

## 5. Interfaces between subsystems

The **provider** (receiver or owner) creates the object; the **user** only sends to
it or waits on it. Message formats are in `app_program/app_if.h`.

| ID | From → To | Mechanism | Object | Data / meaning | Owners (from → to) |
|---|---|---|---|---|---|
| IF-01 | Line → Mission | message buffer (owned by Felix) | `mbf_steer` | `MSG_STEER`: steer −1000..+1000, flags LOST / TIMEOUT | Vania → Felix |
| IF-02 | 1 kHz handler → Barcode | message buffer (owned by Vania) | `mbf_edge` | `MSG_EDGE`: timestamp (µs) and level of each barcode edge | Vania |
| IF-03 | Barcode → Mission | message buffer (owned by Felix) | `mbf_bccmd` | `MSG_BCCMD`: SEEN / OK(A–D) / FAIL | Vania → Felix |
| IF-04 | IMU → Mission | message buffer (owned by Felix) | `mbf_terrain` | `MSG_TERRAIN`: hump start/done (height, tallest), shock, fault | Sook Yee → Felix |
| IF-05 | Scan → Mission | message buffer (owned by Felix) | `mbf_obs` | `MSG_OBS`: detected / result (side, width, clearance) / fail | Felix |
| IF-06 | Mission → Scan | message buffer (owned by Felix) | `mbf_scancmd` | `MSG_SCANCMD`: ARM / DISARM / PROFILE | Felix |
| IF-07 | Dashboard → Mission | message buffer (owned by Felix) | `mbf_mqttcmd` | `MSG_MQTTCMD`: E_STOP / RESUME | Jun Kai → Felix |
| IF-08 | Mission → Motor | message buffer (owned by Dave) | `mbf_nav` | `MSG_NAV`: steer, slow, turn, bypass, resume | Felix → Dave |
| IF-09 | Telemetry → Dashboard | message buffer (owned by Jun Kai) | `mbf_telem` | `MSG_TELEM` frame every 500 ms | Jun Kai (both ends) |
| IF-10 | Mission / button ISR → Motor, Mission | event flag (owned by Dave) | `flg_evt` | START, STOP, RESUME (to Motor); BTN (ISR to Mission) | Felix, Dave |
| IF-11 | Motor, Mission → Mission, Telemetry, Dashboard | shared data under a mutex with priority inheritance (owned by Felix) | `app_state`, `mtx_state` | `APP_STATE`: system/motor state, speed, distance, results, network status | Dave, Felix, Jun Kai |
| IF-12 | Dashboard ↔ kernel MQTT task | two message buffers (owned by Jun Kai) | `mbf_mqtt_out`, `mbf_mqtt_in` | JSON to publish; broker events and command payloads | Jun Kai — **pending: needs a library change** |

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
