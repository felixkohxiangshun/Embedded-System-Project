# Mission and obstacle avoidance — test log (test IDs S5-…)

**Subsystem:** Mission state, obstacle scanning  ·  **Owner:** Felix  ·  **Code:** `app_program/mission_obstacle/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic: `cd testing/host && make run`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | INIT waits for WiFi/MQTT (times out after 20 s, so usable offline) → IDLE; button starts and stops | system statechart | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | steering is forwarded only in line-following states; barcode seen → BARCODE_SCAN | IF-01 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U02 PASS` |
| S5-U03 | barcode A/B/C/D → left 90 / right 90 / straight / u-turn; turn done → LINE_FOLLOW | FR-03 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | failed decode returns to LINE_FOLLOW with no turn | FR-02 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | hump slows the car; result recorded; IMU fault does not stop it | FR-04, FR-05, FR-19 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | obstacle: stop → profile → bypass → reacquire → LINE_FOLLOW | FR-07, FR-09, FR-10 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | line lost too long → FAULT_RECOVERY → IDLE (no restart without BTN_START, review 8.2) | FR-19 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | E_STOP halts; button cannot clear it; RESUME → LINE_FOLLOW | FR-17 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | encoder fault → FAULT_RECOVERY | FR-19 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U09 PASS` |
| S5-U10 | obstacle profile: narrow box, side choice, wall wider than sweep, no echo | FR-08 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U10 PASS` |
| S5-U11 | Mission starts in IDLE after INIT | system statechart | auto | `mission_obstacle_test.c` | `[TEST] S5-U11 PASS` |
| S5-H03 | a bypass longer than 5 s is recorded as late, not aborted (review 8.2) | NFR-10 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H01 | a timed state leaves by timeout (OBSTACLE_SCAN) | FR-19 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H02 | box extending left → go right | FR-09 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H04 | echo capture: rising + falling edge give the width; timer wrap; stale and post-timeout edges ignored; 0 / > 12 ms = no echo | FR-07, Tribunal 1 (R1) | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H05 | a side branch: drive straight on for `JUNCTION_CREEP_MM`, no line fault meanwhile, then turn left 90, stop and return to IDLE (floor test) | junction | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-M06 | the echo no longer busy-waits: IMU keeps its 10 ms period while the sensor is unplugged (no echo, 30 ms wait per ping) — count `s3_ticks` over 10 s | R1, A2 | manual | Unplug ECHO, run 10 s, read `s3_ticks`. | ≥ 990 ticks (record the old busy-wait build for comparison) |
| S5-M01 | HC-SR04 distance at 10, 20, 30 cm within ±1 cm (assumption A1); ECHO divider fitted | FR-07, A1 | manual | Static obstacle at measured distances. | ±1 cm |
| S5-M02 | servo reaches −60, −30, 0, +30, +60°; set the sign and `SERVO_CENTER_US` | FR-08 | manual | Watch the sensor through a sweep. | angles within ±5° |
| S5-M03 | width estimate within 20 % of true width (3 box sizes) | NFR-09 | manual | Profile boxes 5, 10, 15 cm wide. | error ≤ 20 % |
| S5-M04 | bypass completes in ≤ 5 s and the car re-finds the line ≥ 95 % of 20 runs | NFR-10, NFR-11 | manual | Run the full obstacle course. | ≥ 19 of 20 |
| S5-M05 | status LED: IDLE slow blink, RUNNING on, FAULT medium, E_STOP fast | system statechart | manual | Watch GP18 through the states. | patterns as listed |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S5-U01, S5-U02, S5-U03, S5-U04, S5-U05, S5-U06, S5-U07, S5-U08, S5-U09, S5-U10, S5-H01, S5-H02 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| 2026-10-11 | S5-U01a/d/e, S5-U07b, S5-H03a/b, S1-H03, S4-U01 (heading) | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` (80 checks) | re-run after aligning with the final design review (INIT waits for network, recovery to IDLE, no bypass timeout, heading) |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
