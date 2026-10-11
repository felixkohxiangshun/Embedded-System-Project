# Line following and barcode — test log (test IDs S2-…)

**Subsystem:** IR sampler, line following, barcode decoding  ·  **Owner:** Vania  ·  **Code:** `app_program/line_barcode/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic: `cd testing/host && make run`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | decode *A* | FR-02, FR-03 | auto, host | `line_barcode_test.c` | `[TEST] S2-U01 PASS` |
| S2-U02 | decode *B* | FR-02, FR-03 | auto, host | `line_barcode_test.c` | `[TEST] S2-U02 PASS` |
| S2-U03 | decode *C* | FR-02, FR-03 | auto, host | `line_barcode_test.c` | `[TEST] S2-U03 PASS` |
| S2-U04 | decode *D* | FR-02, FR-03 | auto, host | `line_barcode_test.c` | `[TEST] S2-U04 PASS` |
| S2-U05 | decoding is independent of speed (same code at half speed) | FR-02 | auto, host | `line_barcode_test.c` | `[TEST] S2-U05 PASS` |
| S2-U06 | 29 edges, a corrupted width and an unknown pattern are rejected | NFR-05 | auto, host | `line_barcode_test.c` | `[TEST] S2-U06 PASS` |
| S2-U07 | line error sign / lost line | FR-01 | auto, host | `line_barcode_test.c` | `[TEST] S2-U07 PASS` |
| S2-U08 | steer is clamped to ±STEER_MAX | FR-01 | auto, host | `line_barcode_test.c` | `[TEST] S2-U08 PASS` |
| S2-U09 | IR sampler runs at 1 kHz (950–1050 in 1 s) | NFR-04 | auto | `line_barcode_test.c` | `[TEST] S2-U09 PASS` |
| S2-U10 | Line task runs every 10 ms | NFR-03 | auto | `line_barcode_test.c` | `[TEST] S2-U10 PASS` |
| S2-U11 | timer wrap in the middle of a code is harmless | FR-02 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S2-U12 | darkness 0..1000 and clamping | FR-01 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S2-M01 | calibrate: read raw ADC over white and black for all three sensors; set `IR_WHITE_*`, `IR_BLACK_*`, `BC_THRESH`, `IR_DARK_IS_HIGH` | NFR-03 | manual | Print `ir_filtered(0..2)` over the track under venue light. | black and white clearly separated (> 1000 counts) |
| S2-M02 | follow the line around the track 5 laps | FR-01 | manual | Run with Motor and Mission. | no loss of line |
| S2-M03 | barcode live decode rate ≥ 95 % over 20 passes (A, B, C, D) | FR-02, NFR-05 | manual | Pass each barcode 5 times at operating speed. | ≥ 19 of 20 correct |
| S2-M04 | line follows while a barcode is scanned | FR-02 | manual | Scan a barcode while on the line. | car does not leave the line |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S2-U01, S2-U02, S2-U03, S2-U04, S2-U05, S2-U06, S2-U07, S2-U08, S2-U11, S2-U12 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
