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
| S1-U03 | encoder counts ↔ mm conversion (speed within 1 %, integer rounding) | FR-11, FR-12 | auto, host | `motor_control_test.c` | `[TEST] S1-U03 PASS` |
| S1-U04 | PID pushes harder when too slow | FR-11 | auto, host | `motor_control_test.c` | `[TEST] S1-U04 PASS` |
| S1-U05 | PID backs off when too fast | FR-11 | auto, host | `motor_control_test.c` | `[TEST] S1-U05 PASS` |
| S1-U06 | PID output is clamped | FR-11 | auto, host | `motor_control_test.c` | `[TEST] S1-U06 PASS` |
| S1-U07 | Motor loop runs every 10 ms (95–105 ticks in 1 s) | NFR-14 | auto | `motor_control_test.c` | `[TEST] S1-U07 PASS` |
| S1-U08 | motor state is STOPPED after boot | Motor statechart | auto | `motor_control_test.c` | `[TEST] S1-U08 PASS` |
| S1-U09 | encoder count steady with wheels still | FR-12 | auto | `motor_control_test.c` | `[TEST] S1-U09 PASS` |
| S1-H01 | PID reaches 120 mm/s on a model motor | FR-11 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-H02 | turn distance scales with the angle (180° goal within 1 % of 2 × 90°) | FR-03, NFR-15 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-H04 | one wheel silent while the other turns → FAULT_ENCODER after 300 ms | Motor statechart, FR-19 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-H05 | both wheels driven but silent → FAULT_MOTOR_STALL | Motor statechart, CR1 fault bitmask | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-H06 | low duty with no counts is not a fault | Motor statechart | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-H07 | Mission watchdog: slow after 200 ms, stop after 500 ms without MSG_NAV | Tribunal 1, FR-19 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S1-M01 | START → DRIVING, STOP → STOPPED (wheels lifted; build with `-DS1_TEST_MOVE=1`) | IF-10 | auto | `motor_control_test.c` | `[TEST] S1-M01 PASS` |
| S1-M02 | turn each wheel by hand one full turn: count ≈ `ENC_COUNTS_PER_REV` and positive forward; set `ENC_SIGN_*`, `ENC_COUNTS_PER_REV` | FR-12 | manual | Print `enc_cnt[]`; turn the wheel forward one turn. | ±2 % of 2654, same sign both wheels |
| S1-M03 | speed settles on the target with the car on the floor; tune PID | FR-11, NFR-14 | manual | Drive straight 2 m; log speed every 10 ms. | within ±10 % of target after 1 s |
| S1-M04 | straight run drifts less than 2 cm over 2 m | FR-13 | manual | Drive 2 m on a flat floor, steer 0. | lateral drift < 2 cm |
| S1-M05 | 90° and 180° turns within ±5° | NFR-15 | manual | Send NAV_TURN, measure with a protractor, 10 runs. | error ≤ 5° in all runs |
| S1-M06 | start button toggles IDLE/RUNNING, no double triggers | FR-17 | manual | Press BTN_START, watch LED and console. | one toggle per press |
| S1-M07 | PWM 20 kHz on GP8–GP11 | hardware | manual | Scope on GP8 while driving. | ≈ 20 kHz |
| S1-M08 | encoder wire pulled: ENCODER_FAULT within 0.3 s, motors off | Motor statechart, FR-19 | manual | Drive, unplug one encoder. | motors stop; `faults` shows FAULT_ENCODER (0x02) and keeps it until the next start; Mission FAULT_RECOVERY |
| S1-M09 | wheels blocked: hold both wheels while driving | Motor statechart, FR-19 | manual | Drive on the stand, hold both wheels still for 0.5 s. | motors stop; `faults` shows FAULT_MOTOR_STALL (0x10) and keeps it until the next start |
| S1-M10 | Mission silent: build with the floor-test block but without its NAV_STEER feed | Tribunal 1 | manual | EVT_START with Mission idle and no NAV messages. | slows after 0.2 s, stops after 0.5 s, `faults` shows FAULT_MISSION_TMO (0x80) |
| S1-M11 | NAV_STEER sign: +500 → left wheel faster (curves right), −500 → right wheel faster | IF-08, FR-11 | auto | `S1_TEST_CMDS` block (default on), wheels lifted, Mission paused by the test. Counts over 1 s after 1 s settle. | `[TEST] S1-M11a/b PASS`: the faster wheel ≥ 20 % ahead |
| S1-M12 | NAV_STEER beyond ±STEER_MAX is clamped | IF-08 | auto | `S1_TEST_CMDS`: steer +1000 then +2000. | `[TEST] S1-M12 PASS`: both wheels within 10 % |
| S1-M13 | NAV_SLOW → slow speed, NAV_RESUME → normal speed | IF-08, FR-11 | auto | `S1_TEST_CMDS`: measured from encoder counts over 1 s. | `[TEST] S1-M13a/b PASS`: within 15 % of 60 / 120 mm/s |
| S1-M14 | NAV_TURN while DRIVING → TURNING → STOPPED, `cmd_done_seq` +1; EVT_RESUME → DRIVING | IF-08, IF-10, IF-11 | auto | `S1_TEST_CMDS`: the sequence Mission uses for a barcode turn. | `[TEST] S1-M14a/b/c PASS` |
| S1-M15 | NAV_TURN / NAV_BYPASS sent during a turn are ignored; the first turn finishes | IF-08, Motor statechart | auto | `S1_TEST_CMDS`: turn +90, then send −90 and a bypass 200 ms in. | `[TEST] S1-M15 PASS`: STOPPED, `cmd_done_seq` +1, left wheel forward / right back |
| S1-M16 | EVT_STOP aborts a turn; the turn is not reported done | IF-10, IF-11 | auto | `S1_TEST_CMDS`: turn 180, EVT_STOP after 300 ms. | `[TEST] S1-M16a/b PASS`: STOPPED within 20 ms, wheels still, `cmd_done_seq` unchanged |
| S1-M17 | EVT_START during a turn is ignored | IF-10, Motor statechart | auto | `S1_TEST_CMDS`: turn −90, EVT_START 200 ms in. | `[TEST] S1-M17 PASS`: still TURNING, then STOPPED with `cmd_done_seq` +1 |
| S1-M18 | NAV_BYPASS runs every segment, both sides | IF-08, FR-03 | auto | `S1_TEST_CMDS`: bypass right then left, width 100 mm. | `[TEST] S1-M18a/b PASS`: first turn the right way, ends STOPPED, `cmd_done_seq` +1 |
| S1-M19 | bypass path geometry on the floor | FR-03, NFR-15 | manual + auto | Build with `S1_TEST_CMDS 0`, `S1_TEST_BYPASS 1`. Mark start position and heading; measure after each side. | ends 250 mm ahead on the start line, sideways error and heading error recorded (target ±5°) |
| S1-M20 | full run with Mission: line follow → barcode turn → resume → obstacle bypass → resume | IF-08, IF-10, IF-11 | manual | Normal build (`TEST=1` off) on the course; watch telemetry `state`. | Motor follows every command; Mission never waits on a `cmd_done_seq` that does not move |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S1-U01, S1-U02, S1-U03, S1-U04, S1-U05, S1-U06, S1-H01, S1-H02 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
| 2026-10-30 | S1-M11a, S1-M11b, S1-M12, S1-M13a, S1-M13b, S1-M14a, S1-M14b, S1-M14c, S1-M15, S1-M16a, S1-M16b, S1-M17, S1-M18a, S1-M18b | *(uncommitted)* | Dave | PASS | `logs/2026-10-30_cmds.txt` | wheels lifted; DUTY_MAX 600. Speeds within 3 % of target (steer 0: 122/122, +500: 154/92, slow 61, normal 123 mm/s). Summary 22 passed, 1 failed: S1-U01..U09 lines not captured, failure presumed S1-U03 (integer rounding with 2654 counts/rev) |
| 2026-10-30 | S1-U01, S1-U02, S1-U04, S1-U05, S1-U06, S1-U07, S1-U08, S1-U09, S1-M11a … S1-M18b | *(uncommitted)* | Dave | PASS | `logs/2026-10-30_cmds2.txt` | first on-target run of S1-U01..U09; S1-M11..M18 repeated, same numbers within a few counts |
| 2026-10-30 | S1-U03 | *(uncommitted)* | Dave | FAIL | `logs/2026-10-30_cmds2.txt` | test too strict, not a conversion bug: 2654/10 = 265 counts per tick -> 2007 mm/s, test expects exactly 2010 |
| 2026-10-30 | S1-U03, S1-H02 (and all host tests) | *(uncommitted)* | *(Claude, host run)* | PASS | `make -C testing/host run` | tests now allow 1 % for integer rounding; host run 90 passed, 0 failed. S1-U03 still to be re-run on the board |
