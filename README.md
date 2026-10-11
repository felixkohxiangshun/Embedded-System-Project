# Robo Pico car — INF2004 Group 11

Autonomous line-following car on the **Raspberry Pi Pico W + Robo Pico**, written in
**C** with the **Pico C SDK**, running on the **micro T-Kernel 3.0 RTOS** (TRON Forum).
It starts from the INF2004 project template
(<https://github.com/sirfonzie/INF2004_ProjectTemplate>); the design is the final
**Week 6 Design Review**.

There is no FreeRTOS anywhere: tasks, message buffers, the event flag, the mutex,
cyclic handlers and the interrupt handler are all micro T-Kernel objects
(`tk_cre_tsk`, `tk_cre_mbf`, `tk_cre_flg`, `tk_cre_mtx`, `tk_cre_cyc`, `tk_def_int`).

> **Status:** builds cleanly in four configurations and the pure logic passes 80
> PC unit tests. It has **not been run on the board yet**. Every constant marked
> `[CAL]` in the source is a placeholder to measure on the real car.

## Who does what

Group 11, SIT INF2004 Embedded Systems. One folder per subsystem; the owner is the
only person who edits it (shared files are changed by team agreement). Details,
pins and interfaces are in [TEAM.md](TEAM.md).

| Subsystem (folder) | Owner | What it does | Tasks | Design review |
|---|---|---|---|---|
| `motor_control/` | Tan Kun Yi Dave (2502730) | PID speed loop, wheel encoders, drift trim, 90°/180° turns, bypass path, start button | `motor_task` | 6.3, FR-11..13, NFR-14, 15 |
| `line_barcode/` | Vania Teng (2503206) | 1 kHz IR sampler, PD line following, Code 39 barcode decoding (A–D) | `line_task`, `barcode_task` | 6.1, FR-01..03, NFR-03..05 |
| `imu_terrain/` | Lau Sook Yee (2503178) | LSM303DLHC hump height, tallest hump, shock detection, sensor fault retry | `imu_task` | 6.2, FR-04..06, NFR-06, 07 |
| `dashboard/` | Wong Jun Kai (2500258) | Telemetry frame every 500 ms, WiFi/MQTT status, telemetry JSON ≤ 512 B, heartbeat, E_STOP/RESUME commands | `dashboard_task`, `telemetry_task` | 6.5, FR-14..18, NFR-12, 13 |
| `mission_obstacle/` | Felix Koh (2500256) | System statechart, ultrasonic + servo obstacle scan, bypass decision, integration | `mission_task`, `scan_task` | 5, 6.4, FR-07..10, 17, 19, NFR-08..11 |

## Layout

```
app_program/              the car (app_main.c starts every subsystem)
  app.h  app_if.h           shared: pins, priorities, message formats, kernel object IDs
  motor_control/            Dave
  line_barcode/             Vania
  imu_terrain/              Sook Yee
  dashboard/                Jun Kai
  mission_obstacle/         Felix
config/                   kernel limits (config.h) and device switches (config_device.h)
kernel/ lib/ include/ device/ etc/   micro T-Kernel and drivers: template code, not ours
build_make/               the make-based build
testing/                  one TEST_LOG.md per subsystem, plus integration and system; host/ = PC tests
TEAM.md                   owners, pins, peripherals, priorities, interfaces, object budget
docs/                     the template's original README (toolchain set-up)
```

Inside each subsystem folder: `<name>.c/.h` create the kernel objects and start the
tasks, `task_*.c` are the tasks, `drv_*.c` touch the hardware, and `*_logic.c` /
`*_ctrl.c` hold the pure logic that is tested on a PC.

## Tasks and how they talk

Eight tasks, one interrupt, five handlers (design review sections 3 and 4):

| Task | Priority | Trigger | Subsystem |
|---|---|---|---|
| `motor_task` | 12 | cyclic 10 ms | motor_control |
| `line_task` | 13 | cyclic 10 ms | line_barcode |
| `mission_task` | 14 | polls inputs every 5 ms | mission_obstacle |
| `barcode_task`, `imu_task`, `scan_task` | 15 | `mbf_edge` / cyclic 10 ms / ping timer | line_barcode, imu_terrain, mission_obstacle |
| `telemetry_task`, `dashboard_task` | 16 | 500 ms / 100 ms | dashboard |

Priority 1 is the highest and 1–4 belong to the kernel, so the review's ranks 1–5
become 12–16. Tasks never call each other; they use message buffers `mbf_steer`,
`mbf_edge`, `mbf_bccmd`, `mbf_terrain`, `mbf_obs`, `mbf_scancmd`, `mbf_mqttcmd`,
`mbf_nav`, `mbf_telem`, the event flag `flg_evt` (START / STOP / RESUME / BTN) and the
mutex-protected `app_state`. Formats are in `app_program/app_if.h`; the full table
is TEAM.md section 5.

## Build and flash

Toolchain and Pico SDK set-up: `docs/TEMPLATE_README.md` section 1. Then:

```sh
export PICO_SDK_PATH=/path/to/pico-sdk          # not needed for CONSOLE=uart WIFI=none
cd build_make
make -j8                                        # default: dual core, USB console, WiFi radio
make SMP=0 -j8                                  # single core: recommended for first bring-up
make TEST=1 -j8                                 # adds the on-target self-tests
make CONSOLE=uart WIFI=none SMP=0 -j8           # no SDK needed (compile check)
```

Hold BOOTSEL, plug in the Pico, and copy the `.uf2` (named after the options, e.g.
`mtk3pico_smp1_usb_cdc_wifi.uf2`) to the `RPI-RP2` drive. The console is 115200 8N1 on
the USB serial port. For MQTT also see `docs/TEMPLATE_README.md` section 3
(`NET=lwip MQTT=1` and `config/wifi_secrets.h`).

## Tests

* `cd testing/host && make run`: 80 checks of the logic on a PC.
* `make TEST=1` and read the console for `[TEST] ... PASS/FAIL`.
* Manual and integration tests: `testing/*/TEST_LOG.md`.

## Configuration changes from the template

| File | Change | Why |
|---|---|---|
| `config/config.h` | `CNF_TIMER_PERIOD` 10 → 1 | 1 kHz barcode sampler (NFR-04) |
| `config/config.h` | `CNF_MAX_MBFID` 12 → 24 | 11 message buffers plus the device drivers' own |
| `config/config_device.h` | `DEVCNF_USE_ADC` 1 → 0 | the IR sampler owns the ADC |
| `config/config_device.h` | `DEVCNF_USE_IIC` 0 → 1 | IMU on I2C0 |
| `device/i2c/.../i2c_cnf_sysdep.h` | I2C0 pins GP8/GP9 → GP4/GP5 | pin layout of the design review |

## Open items

1. **Hardware bring-up.** Nothing has run on the board. Each owner calibrates the
   `[CAL]` constants of their own subsystem: Dave `motor_ctrl.h` (PID gains, wheel,
   encoder and track constants), Vania `line_ctrl.h` and `drv_ir.c` (white/black
   levels, gains, barcode thresholds), Sook Yee `imu_logic.h` (hump and shock
   thresholds, height factor), Felix `scan_logic.h` and `drv_ultra.c` (sweep timing,
   clearances).
2. **MQTT bridge (IF-12).** The Dashboard exchanges strings with the MQTT client through
   two message buffers, but the matching read/write inside the kernel's `mqtt_utk.c` is
   not written. It edits shared library code, so agree it with the team and the
   instructor first. Until then telemetry JSON is printed on the console.
3. **Design review differences to fix in the report** (the code follows the template and
   the kernel rules where they differ):
   * priority numbers 1–5 become 12–16 (kernel owns 1–4);
   * the 1 kHz handler samples all three IR sensors, not only the barcode one (one ADC);
   * Mission reads Motor's progress through `app_state` (a completion path the diagram lacked);
   * Mission polls its inputs every 5 ms because a task cannot wait on several message buffers;
   * the steering path Line → Mission → Motor adds up to one Mission period of delay (INT-04
     measures it; if it is too slow, send steering straight to Motor).
   Aligned with the final review, so the code now follows these (no report change needed):
   INIT waits for WiFi and MQTT but gives up after 20 s so the car still runs without a
   dashboard; FAULT_RECOVERY returns to IDLE (no restart without BTN_START); a bypass over
   5 s is flagged late, never aborted; telemetry has a `heading` (dead-reckoned from the
   encoders, so it drifts and needs the track width calibrated).
4. **SMP.** The template defaults to dual core (`SMP=1`); the code is written to work either
   way, but the design review's single-core note is not enforced. Decide, and note it in the report.
5. **Hump height** from vertical acceleration is a weak estimate (S3-M02 tests ±5 mm).
6. **FR-11 target speed** reads "1–2 cm/s", which looks like a typo; 12 cm/s is used.
7. **HC-SR04 ECHO is 5 V**; fit a divider before connecting it to GP16.
8. **Pin table typo:** the review lists the encoder rows under the opposite motor
   (`ENC_LEFT` under "Right Motor"); the code takes GP2/GP3 as left and GP6/GP7 as right.
   Swap them in `app.h` if the wiring is the other way round.

## Requirements coverage

| Requirement | Where |
|---|---|
| FR-01 line following | `line_barcode/task_line.c`, `line_ctrl.c` |
| FR-02, FR-03 barcode | `line_barcode/task_barcode.c`, `barcode_decode.c`; command in `mission_obstacle/mission_logic.c` |
| FR-04..FR-06 hump, shock | `imu_terrain/imu_logic.c`, `task_imu.c` |
| FR-07..FR-10 obstacle | `mission_obstacle/task_scan.c`, `scan_logic.c`, `mission_logic.c`; bypass in `motor_control/task_motor.c` |
| FR-11..FR-13 speed, distance, drift | `motor_control/task_motor.c`, `motor_ctrl.c` |
| FR-14..FR-18 WiFi, MQTT, telemetry | `dashboard/` (bridge pending), `dashboard/task_telemetry.c` |
| FR-19 recovery | FAULT_RECOVERY in `mission_logic.c`; IMU and encoder fault paths |
| NFR-03, NFR-04, NFR-06, NFR-14 timing | cyclic handlers (10 ms, 1 ms) and S1-U07, S2-U09, S2-U10, S3-U07 |
| NFR-12, NFR-13 MQTT QoS, ≤ 512 bytes | `dashboard/dash_logic.c` (QoS is set by the MQTT client: pending IF-12) |
