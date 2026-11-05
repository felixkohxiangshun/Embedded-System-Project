# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | 30 edges injected on `mbf_edge` are decoded by the Barcode task | IF-02 | auto | `line_barcode_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | telemetry frames flow from Telemetry to Dashboard (both Jun Kai) (≥ 2 in 1.3 s) | IF-09 | auto | `dashboard_test.c` | `[TEST] INT-02 PASS` |
| INT-03 | **Superseded 2026-11-05 (TEAM.md §7):** remote E-stop removed. Was: an E_STOP on `mbf_mqttcmd` puts Mission in E_STOP (run last) | IF-07 | auto | `mission_obstacle_test.c` | `[TEST] INT-03 PASS` |
| INT-04 | line steering reaches the wheels: move the car off the line by hand, the left/right wheel speeds differ the right way; measure the delay from IR change to PWM change on a scope (steering goes Line → Mission → Motor) | IF-01, IF-08 | manual | Scope on a debug pin toggled in `line_task` and one PWM pin. | correct direction; delay < 20 ms |
| INT-05 | **Superseded 2026-11-05 (TEAM.md §7):** barcodes no longer turn the car directly, see INT-21. Was: barcode → turn: present A, B, C, D | IF-02, IF-03, IF-08, IF-11 | manual | Pass each barcode under the car at operating speed. | left 90 / right 90 / straight / u-turn, then back on the line |
| INT-06 | hump → slow → resume | IF-04, IF-08 | manual | Drive over a hump. | speed drops, then returns; `hump_max_mm` in telemetry |
| INT-07 | obstacle end to end: detect at ≤ 30 cm, stop, sweep, bypass, find the line | IF-05, IF-06, IF-08 | manual | Place a box on the line. | car goes round and resumes within 5 s of starting the bypass |
| INT-08 | Mission sees the Motor finish a turn (completion through `app_state.cmd_done_seq`) | IF-11 | manual | Send a barcode turn; watch `cmd_done_seq` and the state change. | state returns to LINE_FOLLOW right after the turn ends |
| INT-09 | **Superseded 2026-11-05 (TEAM.md §7):** remote E-stop removed. Was: **blocked until IF-12 is agreed:** dashboard E_STOP → Mission → Motor stops | IF-07, IF-10, IF-12 | manual | Publish E_STOP. | motors stop within 200 ms |
| INT-20 | a mapping published over MQTT reaches Mission (`mbf_mqttcmd`) and shows in telemetry | IF-07, IF-11, IF-12 | manual | Publish a mapping from the PC. | telemetry reports the new mapping |
| INT-21 | barcode → stored character → cross junction → mapped movement | IF-01, IF-03, IF-08, IF-11 | manual | Drive over a barcode, then a cross junction, for each of the four movements. | the mapped movement at each junction |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
