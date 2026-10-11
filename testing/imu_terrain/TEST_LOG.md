# IMU and terrain — test log (test IDs S3-…)

**Subsystem:** IMU, hump and shock detection  ·  **Owner:** Sook Yee  ·  **Code:** `app_program/imu_terrain/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic: `cd testing/host && make run`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | baseline measured, flat road is quiet | FR-04 | auto, host | `imu_terrain_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | hump start and completion; height from the peak | FR-04 | auto, host | `imu_terrain_test.c` | `[TEST] S3-U02 PASS` |
| S3-U03 | tallest hump of the run is kept | FR-05 | auto, host | `imu_terrain_test.c` | `[TEST] S3-U03 PASS` |
| S3-U04 | spike > 3 g for < 3 samples = SHOCK | FR-06 | auto, host | `imu_terrain_test.c` | `[TEST] S3-U04 PASS` |
| S3-U05 | a long high reading is not a SHOCK | FR-06 | auto, host | `imu_terrain_test.c` | `[TEST] S3-U05 PASS` |
| S3-U06 | fault ignores samples; recovery recalibrates | FR-19 | auto, host | `imu_terrain_test.c` | `[TEST] S3-U06 PASS` |
| S3-U07 | IMU task runs every 10 ms even with a missing sensor | NFR-06 | auto | `imu_terrain_test.c` | `[TEST] S3-U07 PASS` |
| S3-H01 | LSM303DLHC answers on I2C0 (needs the sensor wired) | hardware | auto | `imu_terrain_test.c` | `[TEST] S3-H01 PASS` |
| S3-M01 | raw Z at rest ≈ ±1000 mg; axis and sign correct | hardware | manual | Print `imu_read_z_mg()` flat and tilted. | ≈ 1000 mg flat |
| S3-M02 | fit `HUMP_MM_PER_G` on humps of known height; check ±5 mm | FR-04, NFR-07 | manual | Drive over 3 humps (e.g. 10, 15, 20 mm), 5 runs each. **If the spread is too large, change the method.** | error ≤ 5 mm |
| S3-M03 | drop/knock the car: SHOCK flag set | FR-06 | manual | Tap the chassis. | `shock` = 1 in telemetry |
| S3-M04 | unplug the IMU while driving: car keeps line following | FR-19 | manual | Disconnect SDA while running. | car keeps driving; fault flag set |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S3-U01, S3-U02, S3-U03, S3-U04, S3-U05, S3-U06 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
