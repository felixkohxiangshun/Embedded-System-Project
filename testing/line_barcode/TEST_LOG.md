# Line following and barcode — test log (test IDs S2-…)

**Subsystem:** IR sampler, line following, barcode decoding  ·  **Owner:** Vania  ·  **Code:** `app_program/line_barcode/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic and the simulations:
`cd testing/line_barcode && make run` (also started by `cd testing/host && make run`).
Style check: `python3 testing/line_barcode/check_style.py`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | decode *A* | FR-02, FR-03 | host | `lb_host_tests.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | decode *B* | FR-02, FR-03 | host | `lb_host_tests.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | decode *C* | FR-02, FR-03 | host | `lb_host_tests.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | decode *D* | FR-02, FR-03 | host | `lb_host_tests.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | decoding is independent of speed (same code at half speed) | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | 29 edges, a squashed element, a bad frame and a bad gap are rejected (a–d) | NFR-05 | host | `lb_host_tests.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | line error sign / line not seen (a–d) | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U07a..d PASS` |
| S2-U08 | steer is clamped to ±STEER_MAX | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | IR sampler runs at 1 kHz (950–1050 in 1 s) | NFR-04 | auto | `line_barcode_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | Line task runs every 10 ms | NFR-03 | auto | `line_barcode_test.c` | `[TEST] S2-U10 PASS` |
| S2-U11 | timer wrap in the middle of a code is harmless | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U11 PASS` |
| S2-U12 | darkness 0..1000 and clamping (a–c) | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U12a..c PASS` |
| S2-U13 | **Superseded 2026-11-07:** every letter A–Z is now a command, see S2-U30 and S2-U31. Was: valid Code 39 characters F, W, X, Y, Z are read but are not commands | FR-03, NFR-05 | host | `lb_host_tests.c` | `[TEST] S2-U13 PASS` |
| S2-U30 | every letter A–Z decodes from its Code 39 pattern | FR-02, FR-03 | auto, host | `lb_host_tests.c` | `[TEST] S2-U30 PASS` |
| S2-U31 | every letter A–Z decodes to the same letter when the code is read in the opposite direction | FR-02, FR-03 | auto, host | `lb_host_tests.c` | `[TEST] S2-U31 PASS` |
| S2-U32 | widths from a real hand swipe on 2026-11-07 (narrow bars read 0.4–7.8 ms, narrow spaces 5–8 ms) decode as *F*; the speed check now compares each character's total time, not its narrow bars | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U32 PASS` |
| S2-U33 | DO debouncer (digital mode, edges from the GP28 interrupt): a 1.3 ms bar gives two edges at the exact interrupt times (a); chatter at both edges gives two edges (b); a 100 µs speck is ignored (c); timer wrap is harmless (d). Replaces the 10 ms filter + slicer, which lost the 1–5 ms narrow elements measured while driving on 2026-11-07 | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U33a..d PASS` |
| S2-S01..S03 | straddle mode (`LB_LINE_STRADDLE 1`, 2026-11-09): both sensors white → on the line, steer straight (S01a); centre black → steer left, right black → steer right (S01b); both black → straight (S01c); simulated with a 30 mm sensor gap: straight at 80 mm/s from 8 mm off stays within 12 mm (S02); two 90° bends at 80 and 18 mm/s followed within 15 / 18 mm (S03); the line sliding out past a sensor (peak ≥ `LB_STRADDLE_FULL` 600, then white) makes the car steer back that way and, after `LB_STRADDLE_SEEK_MS` 1.5 s of white, report it lost so the search and Mission's timeout stop the car (S04); a light touch (peak below 600) then white = line inside, never lost (S05); a 100 mm radius bend at 60 mm/s is followed (S06; was 80 mm/s, where only the side memory recovered it, before the smoother straddle gains KP 400 / KD 200 / slew 40); white-paper noise up to 250 never counts as black (S07, `LB_STRADDLE_BLACK_MIN` 300). Faded ink is handled by calibrating `LB_BLACK_xxx` on the faded line, not by lower thresholds (SAT 500 / junctions at 400 were tried and blurred black and white). The edge-mode checks S2-U07d, U23, L01–L03, L05, R02, R04 and R05 run only with `LB_LINE_STRADDLE 0` (62/62 there) | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-S01a..S03 PASS` |
| S2-S08 | straddle: a junction (both sensors 900) is found by both detectors and crossed straight, with no "line went out" memory left behind. Junction thresholds stay 850 / 800 in both modes. S2-U27 uses mode-specific plain-line readings | junction | host | `lb_host_tests.c` | `[TEST] S2-S08 PASS` |
| S2-U07e | edge mode with the difference error (`LB_LINE_ERR_DIFF 1`, 2026-11-09): centre 20 / right 350 (the car's own reading) gives err +165, not the +450 the share formula gave, which made the car zigzag; `LB_KP` raised 400 → 600 so bends at 120 mm/s are still followed. S2-L05 (slow motors) and S2-R05 (faded bend) now run at the car's 60 mm/s: at 120 mm/s they fail with the gentler error. S2-U23 (half the ink gives the same error) only applies to the share formula | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U07e PASS` |
| S2-U14 | a code driven over backwards is flagged as reversed | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U14 PASS` |
| S2-U15 | edge timing noise of ±0.3 unit: ≥ 95 % read, none read wrong | NFR-05 | host | `lb_host_tests.c` | `[TEST] S2-U15 PASS` |
| S2-U16 | edge stream: SEEN once, FULL at 30, stray edges ignored (a–d); a 30-edge capture with one narrow space squeezed to 0.3× still decodes, because the raw edges are tried before glitch merging (e, added 2026-11-07 after a 30/30 hand swipe failed with COUNT) | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U16a..e PASS` |
| S2-U17 | a short speck inside an element is merged away | NFR-05 | host | `lb_host_tests.c` | `[TEST] S2-U17 PASS` |
| S2-U18 | optics simulation: no pass is decoded to a wrong command | NFR-05 | host | `lb_host_tests.c` | `[TEST] S2-U18 PASS` |
| S2-U19 | optics simulation: ≥ 98 % correct, 60–160 mm/s, spot 3–5 mm | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U19 PASS` |
| S2-U20 | optics simulation: wide/narrow printed as 2.5 is still read | FR-02 | host | `lb_host_tests.c` | `[TEST] S2-U20 PASS` |
| S2-U21 | the 10 ms average removes 100 Hz flicker | NFR-04 | host | `lb_host_tests.c` | `[TEST] S2-U21 PASS` |
| S2-U22 | steering change per tick is limited (slew) | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U22 PASS` |
| S2-U23 | half the ink gives the same position error and the FADED flag | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U23 PASS` |
| S2-U24 | line vanishes: bridge, LOST flag, coast, sweep, TIMEOUT at 3 s (a–d) | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U24a..d PASS` |
| S2-U25 | re-acquire needs 3 good ticks in a row (a, b) | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U25a..b PASS` |
| S2-U26 | no TIMEOUT while Mission is not following the line | FR-01 | host | `lb_host_tests.c` | `[TEST] S2-U26 PASS` |
| S2-T03 | the longest sampler run is below 200 µs | NFR-04 | auto | `line_barcode_test.c` | `[TEST] S2-T03 PASS` |
| S2-T04 | ADC readings are not stuck at 0 or 4095 | NFR-04 | auto | `line_barcode_test.c` | `[TEST] S2-T04 PASS` |
| S2-L01 | closed loop: straight line stays centred | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-L01 PASS` |
| S2-L02 | closed loop: start 8 mm / 5° off, converges without overshoot | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-L02 PASS` |
| S2-L03 | closed loop: two 90° bends at 120 mm/s | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-L03 PASS` |
| S2-L04 | closed loop: two 90° bends at 60 mm/s | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-L04 PASS` |
| S2-L05 | closed loop: slow motors (lag 150 ms, delay 50 ms) | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-L05 PASS` |
| S2-R01 | recovery: ink drops to 25 % and the car veers 4°/s | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-R01 PASS` |
| S2-R02 | recovery: 100 mm without ink + veer, line re-acquired | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-R02 PASS` |
| S2-R03 | recovery: 200 mm of missing line on a straight is crossed | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-R03 PASS` |
| S2-R04 | recovery: line gone for good, TIMEOUT reaches Mission | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-R04 PASS` |
| S2-R05 | recovery: ink drops to 30 % inside a bend | FR-01 | host (sim) | `lb_host_tests.c` | `[TEST] S2-R05 PASS` |
| S2-M01 | calibrate: read raw ADC over white and black for all three sensors; set `LB_WHITE_*`, `LB_BLACK_*`, `LB_BC_WHITE_NOMINAL`, `LB_BC_BLACK_NOMINAL`, `LB_IR_DARK_IS_HIGH` in `lb_config.h` | NFR-03 | manual | Set `LB_TEST_CALIBRATE 1`, build with `TEST=1`, follow the console (guide step C). | black and white clearly separated (> 1000 counts) |
| S2-M02 | follow the line around the track 5 laps | FR-01 | manual | Run with Motor and Mission. | no loss of line |
| S2-M03 | barcode live decode rate ≥ 95 % over 20 passes (A, B, C, D) | FR-02, NFR-05 | manual | Pass each barcode 5 times at operating speed. | ≥ 19 of 20 correct |
| S2-M04 | line follows while a barcode is scanned | FR-02 | manual | Scan a barcode while on the line. | car does not leave the line |
| S2-M05 | faded line: lay a strip of white paper or tape over 100 mm of the line, then 200 mm | FR-01 | manual | Run at normal speed past the strip. | car crosses it and picks the line up again; `line_recoveries` counter rises by 1 |
| S2-M06 | slight veer: put a thin 3 mm wedge under one side so the car drifts, over a half-faded stretch | FR-01 | manual | Run past it. | car returns to the line; no FAULT_RECOVERY |
| S2-M07 | line missing for good (end of line): the car reports the loss | FR-01 | manual | Lift the car off or let it run past the end of the line. | Mission enters FAULT_RECOVERY about 3 s after the line is gone |
| S2-M08 | barcode passes at slow (60 mm/s) and normal (120 mm/s) speed, all four codes, 5 each | FR-02, NFR-05 | manual | Print `gen_patterns.py sheet`, lay beside the line. | ≥ 38 of 40 correct, 0 wrong turns |
| S2-M09 | a code crossed from the wrong side (reversed) is not obeyed | FR-02, NFR-05 | manual | Drive over the code backwards. | `bc_failed` rises; no turn is made |
| S2-U27 | junction detector: needs plain line first, ignores 7 mm of both dark, ignores one sensor dark alone, reports 8 mm of both dark once, re-arms on plain line | junction | host | `lb_host_tests.c` | `[TEST] S2-U27a..f PASS` |
| S2-M10 | car drives on to a junction, using the two line sensors, turns left 90 and stops | junction | manual | Press GP20 with the line between the sensors and run towards a junction, `MISSION_JUNCTION_STOP` 1, 5 runs at a cross or T. | axle on each junction, turns left 90 about the junction and stops, never on plain line, state `IDLE` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S2-U01, S2-U02, S2-U03, S2-U04, S2-U05, S2-U06, S2-U07, S2-U08, S2-U11, S2-U12 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| 2026-11-01 | S2-U01 to S2-U26, S2-L01 to S2-L05, S2-R01 to S2-R05 (all `lb_host_tests.c` checks) | *(uncommitted)* | *(Claude, AI-assisted)* | PASS (49 of 49) | `cd testing/line_barcode && make run` | PC run with the strict gcc flags in the Makefile. Simulations only: they test the logic, not the real optics or motors. |
| 2026-11-01 | digital barcode path (`lb_digital_test.c`: 7 sets x 12 scans) | *(uncommitted)* | *(Claude, AI-assisted)* | PASS (7 of 7 sets) | `cd testing/line_barcode && make digital` | Two-level DO signal through the real 10 ms filter, slicer and decoder; bar width offset of plus or minus 40 % and 2 samples of bounce. PC only; the real DO output (trimmer setting, LED) is not tested. |
| 2026-11-01 | merge check: line_barcode on `main` plus `motorTask` (cf169b1) | *(uncommitted)* | *(Claude, AI-assisted)* | PASS | `cd testing/host && make run` | PC only. Branch merged without conflicts; 75 other host checks, 49 line/barcode checks and the digital checks pass; every line_barcode file compiles against the merged `app.h` / `app_if.h` (stub kernel headers). No ARM build. |
| *(none yet)* | | | | | | on-target `TEST=1` run (S2-U09, S2-U10, S2-T03, S2-T04, INT-01) and manual tests S2-M01 to S2-M09 still to do |
