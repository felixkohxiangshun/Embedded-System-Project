# IMU and terrain — test log (test IDs S3-…)

**Subsystem:** IMU, hump and shock detection  ·  **Owner:** Sook Yee  ·  **Code:** `app_program/imu_terrain/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic: `cd testing/host && make run`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | calibration at rest removes the mounting tilt; flat road is quiet | FR-04 | auto, host | `imu_terrain_test.c`: 1.2 s still with a 2° tilted board, then 1.5 m of flat road | `[TEST] S3-U01 PASS` |
| S3-U02 | hump start and completion; height from the roll (hump under one wheel) | FR-04, NFR-07 | auto, host | 20 mm trapezoid hump (60 mm ramps, 50 mm top) under the right wheel | start + done, height 20 ± 2 mm |
| S3-U03 | tallest hump of the run is kept | FR-05 | auto, host | then a 10 mm hump | height 10 ± 2 mm, tallest still the first |
| S3-U04 | knock > 3 g from the rest vector for < 3 samples = SHOCK | FR-06 | auto, host | 3.5 g on the forward axis for 2 samples | `[TEST] S3-U04 PASS` |
| S3-U05 | a long high reading is not a SHOCK | FR-06 | auto, host | 3.5 g for 5 samples | `[TEST] S3-U05 PASS` |
| S3-U06 | fault ignores samples; recovery keeps the baseline (calibrated) or recalibrates (not yet calibrated) | FR-19 | auto, host | `imu_fault()` / `imu_recovered()` | `[TEST] S3-U06 PASS` |
| S3-U07 | IMU task runs every 10 ms even with a missing sensor | NFR-06 | auto | `imu_terrain_test.c` | `[TEST] S3-U07 PASS` |
| S3-U08 | a knock does not look like a hump | FR-04, FR-06 | auto, host | same run as S3-U04 | no hump events |
| S3-U09 | hump above `HUMP_MAX_MM` (63 mm, 6 bits) is reported but not kept as tallest | FR-05, Tribunal 1 v.C | auto, host | 70 mm hump after a 20 mm one | done, height > 63, tallest unchanged |
| S3-U10 | turning during a hump gives it up (no false height) | FR-04 | auto, host | turn starts on the hump | no HUMP_DONE, back to MONITORING |
| S3-U11 | encoder counts → µm without rounding drift | FR-04 | auto, host | 1000 × 30 counts | 1 220 799 ± 1 µm |
| S3-U13 | the car stops on top of a hump: `imu_finish()` still gives its height | FR-04 | auto, host | drive to the middle of a 20 mm hump, stop | HUMP_DONE, 20 ± 2 mm |
| S3-U14 | no measuring before calibration; calibration works with \|a\| = 885 mg (the car's sensor) | FR-04 | auto, host | tilted board, encoder twitching, motor stopped / driving | no hump before calibration; quiet after |
| S3-U12 | pitch mode still works for a hump across the whole track | FR-04 | auto, host | `imu_set_mode(HUMP_MODE_PITCH)`, 20 mm full-width hump | height 20 ± 2 mm |
| S3-S01 | 3D physics simulation: hump under one wheel (and full-width), 10–50 mm, 60–400 mm long; speed changes from Mission; vibration; board tilt; wrong track width | FR-04, NFR-07 | host, sim | `testing/imu_terrain/sim_hump.c` (build line at the top), 20 seeds: `SEED=n ./sim_hump` | all PASS, error ≤ 5 mm |
| S3-H01 | LSM303DLHC answers on I2C0 (needs the sensor wired) | hardware | auto | `imu_terrain_test.c` | `[TEST] S3-H01 PASS` |
| S3-M01 | axes and signs: at rest "up" ≈ +1000 mg; lifting the nose makes "fwd" positive | hardware | manual | `make TEST=1`, read the `[IMU] raw X Y Z` line flat and nose-up; set `IMU_AXIS_*` / `IMU_SIGN_*` in `imu_terrain.h` | up ≈ +1000, fwd > 0 nose-up |
| S3-M02 | hump height on real humps | FR-04, NFR-07 | manual + auto | Set `S3_TEST_HUMP 1` in `imu_terrain_test.c`, `make TEST=1`. Mission is paused by the test, so no line is needed. No cable needed: car still at power-on (calibration); for each run place it ~20 cm before the hump, pointing straight, press START (GP20); it drives 800 mm and stops. After 5 runs plug in USB: the results repeat every 3 s until RST. Measure the hump with a ruler. | `[TEST] S3-M02 PASS` every run; height within ±5 mm of the ruler; residual small |
| S3-M03 | drop/knock the car: SHOCK flag set | FR-06 | manual | Tap the chassis. | `shock` = 1 in telemetry |
| S3-M04 | unplug the IMU while driving: car keeps line following | FR-19 | manual | Disconnect SDA while running. | car keeps driving; fault flag set; `TERR_RECOVERED` when plugged back |
| S3-M05 | measure the track width for `CAR_TRACK_MM` | FR-04 | manual | Ruler across the car: centre of the left wheel's contact to centre of the right wheel's. 5 mm wrong ≈ 4.5 % of the height. (`CAR_WHEELBASE_MM` is only used by the full-width pitch mode.) | value entered in `imu_logic.h` |
| S3-M07 | raw recording of one run (diagnosis) | FR-04 | manual | Set `S3_TEST_LOG 1`, `make TEST=1`. Power on still; place the car as for S3-M02, press START: one 700 mm run is recorded. Then plug in USB, start PuTTY logging to a file, press START: the CSV prints between `LOG_BEGIN` and `LOG_END` (press again to reprint). | CSV file saved in `logs/` |
| S3-R01 | replay a real recording through the real code | FR-04 | host | `testing/imu_terrain/replay.c` (build line at the top): `./replay logs/2026-11-01_hump_run.txt [trace]` | 1 hump, height near the ruler value; flat parts stay below the 8 mm threshold |
| S3-M06 | no false humps on the flat track with motors running | FR-04 | manual | 3 laps of a flat section | no `[IMU] hump` lines |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S3-U01, S3-U02, S3-U03, S3-U04, S3-U05, S3-U06 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
| 2026-11-01 | S3-U01–U06, S3-U08–U11 | *(uncommitted)* | *(Claude, for Sook Yee)* | PASS | `make -C testing/host run` → 94 passed, 0 failed | new method (pitch × encoder distance with wheelbase). S3-U02/U03 now check the height against the true hump; S3-U06b now expects the baseline to be kept |
| 2026-11-01 | S3-M02 | *(uncommitted)* | Sook Yee | FAIL | console: 5 runs gave 1–3 humps each, 18–42 mm, residual up to ±10.8 mm | the course's hump is under the RIGHT wheel only; the code assumed a full-width hump (pitch). Changed to roll mode, see next rows |
| 2026-11-01 | S3-S01 | *(uncommitted)* | *(Claude, for Sook Yee)* | PASS | `sim_hump`, seeds 1–20: 0 failures; worst error ±2 mm (±3–4 mm with the wheelbase set 10 mm wrong) | simulation only; firmware also builds with `make`, `TEST=1`, `SMP=0` (no warnings). Not yet run on the board |
| 2026-11-01 | S3-U01–U12, S3-S01 | *(uncommitted)* | *(Claude, for Sook Yee)* | PASS | host 95 passed; `sim_hump` seeds 1–20: 0 failures; roll mode within ±1 mm (±2 mm with the track width 5 mm off) | roll mode (one-wheel hump) is now the default. Not yet run on the board |
| 2026-11-01 | S3-M07 | *(uncommitted)* | Sook Yee | — | `logs/hump.txt` (recording started too early: car standing still only) | Found the cause of the false humps: the left encoder twitched ~10 counts at power-up, the IMU took that as moving and skipped calibration, so the board's tilt looked like a 2.2 mm lifted wheel; `[IMU] hump detected` 0.5 s after power-up and stuck in HUMP_TRACKING. Fixed: calibration needs only Motor STOPPED; nothing is measured until calibrated (S3-U14). Sensor at rest: X -30, Y +18, Z +884 mg (Z up, \|a\| 885 mg) |
| 2026-11-01 | S3-M07 | *(uncommitted)* | Sook Yee | — | `logs/2026-11-01_hump_run.txt` (700 mm, hump under the right wheel) | The board's axes are X sideways (+ = right wheel up), Y forward, Z up (code assumed X forward). Y shakes ~330 mg rms at ~24 Hz (motors/gears); the car sways about ±3–4 mm of lean on the flat. Fixed: axis mapping, 8-sample average + ~160 ms low-pass, only a lean with the right wheel up counts, 8 mm minimum, start only while moving |
| 2026-11-01 | S3-R01 | *(uncommitted)* | *(Claude, for Sook Yee)* | PASS | `./replay logs/2026-11-01_hump_run.txt` → 1 hump, 31 mm, 208 mm long; flat lean −5.7 … +3.7 mm; standing ≈ 0 | ruler height of this hump still to be compared |
| 2026-11-01 | S3-U01–U14, S3-S01 | *(uncommitted)* | *(Claude, for Sook Yee)* | PASS | host 98 passed; `sim_hump` with the measured shaking and sway, 20 seeds: no false humps (80 flat runs); heights mostly ±5 mm, worst ±7 mm; humps from ~12 mm high and ~100 mm long | not yet re-run on the board |
