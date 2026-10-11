# Motor control — test log (test IDs S1-…)

**Subsystem:** Motor control, encoders, start button  ·  **Owner:** Dave  ·  **Code:** `app_program/motor_control/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic: `cd testing/host && make run`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | quadrature: 4 forward edges = +4 | FR-12 | auto, host | `motor_control_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | quadrature: 4 reverse edges = −4; a double jump counts 0 | FR-12 | auto, host | `motor_control_test.c` | `[TEST] S1-U02 PASS` |
| S1-U03 | encoder counts ↔ mm conversion | FR-11, FR-12 | auto, host | `motor_control_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | PID pushes harder when too slow | FR-11 | auto, host | `motor_control_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | PID backs off when too fast | FR-11 | auto, host | `motor_control_test.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | PID output is clamped | FR-11 | auto, host | `motor_control_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | Motor loop runs every 10 ms (95–105 ticks in 1 s) | NFR-14 | auto | `motor_control_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | motor state is STOPPED after boot | Motor statechart | auto | `motor_control_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | encoder count steady with wheels still | FR-12 | auto | `motor_control_test.c` | `[TEST] S1-U09 PASS` |
| S1-H01 | PID reaches 120 mm/s on a model motor | FR-11 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-H02 | turn distance scales with the angle | FR-03, NFR-15 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-M01 | START → DRIVING, STOP → STOPPED (wheels lifted; build with `-DS1_TEST_MOVE=1`) | IF-10 | auto | `motor_control_test.c` | `[TEST] S1-M01 PASS` |
| S1-M02 | turn each wheel by hand one full turn: count ≈ `ENC_COUNTS_PER_REV` and positive forward; set `ENC_SIGN_*`, `ENC_COUNTS_PER_REV` | FR-12 | manual | Print `enc_cnt[]`; turn the wheel forward one turn. | ±2 % of 960, same sign both wheels |
| S1-M03 | speed settles on the target with the car on the floor; tune PID | FR-11, NFR-14 | manual | Drive straight 2 m; log speed every 10 ms. | within ±10 % of target after 1 s |
| S1-M04 | straight run drifts less than 2 cm over 2 m | FR-13 | manual | Drive 2 m on a flat floor, steer 0. | lateral drift < 2 cm |
| S1-M05 | 90° and 180° turns within ±5° | NFR-15 | manual | Send NAV_TURN, measure with a protractor, 10 runs. | error ≤ 5° in all runs |
| S1-M06 | start button toggles IDLE/RUNNING, no double triggers | FR-17 | manual | Press BTN_START, watch LED and console. | one toggle per press |
| S1-M07 | PWM 20 kHz on GP8–GP11 | hardware | manual | Scope on GP8 while driving. | ≈ 20 kHz |
| S1-M08 | encoder wire pulled: ENCODER_FAULT within 0.3 s, motors off | Motor statechart, FR-19 | manual | Drive, unplug one encoder. | motors stop; Mission FAULT_RECOVERY |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S1-U01, S1-U02, S1-U03, S1-U04, S1-U05, S1-U06, S1-H01, S1-H02 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
