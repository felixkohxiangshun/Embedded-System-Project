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
| S5-U03 | **Superseded 2026-11-05 (TEAM.md §7):** the car no longer turns when it reads a barcode, see S5-U20. Was: barcode A/B/C/D → left 90 / right 90 / straight / u-turn; turn done → LINE_FOLLOW | FR-03 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U03 PASS` |
| S5-U04 | failed decode returns to LINE_FOLLOW with no turn | FR-02 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U04 PASS` |
| S5-U05 | hump slows the car; result recorded; IMU fault does not stop it | FR-04, FR-05, FR-19 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U05 PASS` |
| S5-U06 | obstacle: stop → profile → bypass → reacquire → LINE_FOLLOW | FR-07, FR-09, FR-10 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U06 PASS` |
| S5-U07 | line lost too long → FAULT_RECOVERY → IDLE (no restart without BTN_START, review 8.2) | FR-19 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U07 PASS` |
| S5-U08 | **Superseded 2026-11-05 (TEAM.md §7):** remote E-stop and RESUME removed. Was: E_STOP halts; button cannot clear it; RESUME → LINE_FOLLOW | FR-17 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U08 PASS` |
| S5-U09 | encoder fault → FAULT_RECOVERY | FR-19 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U09 PASS` |
| S5-U10 | obstacle profile (5 positions: far left, near left, centre, near right, far right, about 45 / 22.5 degrees): narrow box, side choice, wall wider than sweep, no echo | FR-08 | auto, host | `mission_obstacle_test.c` | `[TEST] S5-U10 PASS` |
| S5-U11 | Mission starts in IDLE after INIT | system statechart | auto | `mission_obstacle_test.c` | `[TEST] S5-U11 PASS` |
| S5-H03 | a bypass longer than 5 s is recorded as late, not aborted (review 8.2) | NFR-10 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H01 | a timed state leaves by timeout (OBSTACLE_SCAN) | FR-19 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H02 | box extending left → go right | FR-09 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H11 | the 5 servo positions use their pulse widths in order (1970 / 1815 / 1660 / 1500 / 1340 us; near ones are midpoints until calibrated) | FR-08 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H14 | the near left beam still hits but the far left one is clear: the obstacle's edge lies between them, so that side is wider and the car goes the other way | FR-09 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H04f (2026-11-11) | also: a 152 µs echo (26 mm, read on the car with nothing in front) is rejected as no echo; 40 mm (`SCAN_MIN_MM`) and longer is kept | FR-07 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H04 | echo capture: rising + falling edge give the width; timer wrap; stale and post-timeout edges ignored; 0 / > 12 ms = no echo | FR-07, Tribunal 1 (R1) | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H05 | a cross junction with no letter stored: drive straight on for `JUNCTION_CREEP_MS` (1.5 s, was 150 mm until 2026-11-09), no line fault meanwhile, then follow the line again with no turn (changed 2026-11-07: the floor-test U-turn and stop are gone) | junction | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H07 | barcode floor test (`S5_TEST_STOP_AT_BARCODE`): keeps driving while a barcode is seen, stops in IDLE once it is decoded or fails, keeps the letter, never turns | FR-02 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H08 | a letter read while IDLE (hand swipe) is recorded as last read and stored for the next junction; the car does not move | FR-03 | host | `host_tests.c` | `PASS S5-H08` |
| S5-H10 | obstacle floor test (`S5_TEST_STRAIGHT 1`): GP20 drives straight with detection armed, the line sensors are ignored (no steering, junction creep or line-lost fault), an obstacle stops the car, the sweep runs, the car bypasses on the clear side and then carries on straight with detection armed again, with no line-reacquire fault | FR-07, FR-08 | host | `host_tests.c` | `PASS S5-H10a..g` |
| S5-H12 | integrated run: GP20 start, line steering to Motor, barcode F stored, IMU hump slows (8 s crossing not cut short) and is recorded, IMU fault keeps driving, right turn at the junction, line following resumes, GP20 stop | INT | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H13 | barcode at speed: code starts (two bars) → reader on hold, back up 10 cm (no steering, junction or line fault meanwhile) → fresh capture, forward at 18 mm/s → decoded → 60 mm/s; back-up timeout faults; slow pass ends at the latest BC_READ_MAX_MM after it began, or on the time-out; no back-up when IDLE or on a hump | FR-01..03 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H15 | barcode re-read (2026-11-11): the slow forward read fails → fresh capture and reverse at 18 mm/s over the distance of the slow pass (back to before the code), no steering / junction / line fault meanwhile; a reverse decode stores the letter (exit "decoded on the reverse pass"); then forward at the normal speed, ignoring barcodes until 100 mm past where the reverse began; both reads failing reports "failed". Back-up is now 10 cm | FR-01..03 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H16 | 45° dog-leg geometry (2026-11-17): first turn ±45 to the chosen side; outward diagonal = (half width + 60 mm margin + half car) / sin 45; straight leg = distance + obstacle depth + car length − the diagonal's forward gain (at least 100 mm); return diagonal 150 mm longer to look for the line (equal when there is no line); width clamped 60–400 mm | FR-09, FR-10 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H17 | "no way round" keeps the car stopped and sweeps again (timeout per sweep); then turn 45, diagonal out, turn back, past, turn in, return diagonal; black under an IR sensor on the return stops the car, then it turns 45 back along the line and reacquires | FR-08..10 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H18 | black seen before the return diagonal is ignored; a return diagonal that never meets the line still turns back; obstacle gone during the sweep (centre > 400 mm or no echo) → drive on | FR-08..10 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-H21 | (2026-11-17) "no way round" twice: still stopped, sweeping; the third time it goes round anyway on the side the sweep found less blocked (default right if unknown) with the widest clearance (`BYP_WIDTH_MAX_MM`); the obstacle going away in between drives on; a new obstacle counts from zero; the sweep names the less blocked side even with no room | FR-08, FR-09 | host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-M11 | floor: obstacle dead centre on the line, wide enough for "no way round": three sweeps, then the car goes round on one side and back to the line | FR-08..10 | manual | full build | no endless sweeping; record the side and the bypass time |
| S5-M10 | floor: obstacle avoidance on the track — stops within 30 cm, sweeps, goes round on the chosen side, stops on the line on the way back, faces along it and follows on. Note the clearance and the bypass time | FR-07..10, NFR-10 | manual | Full build, obstacle on a straight, GP20 | car back on the line past the obstacle without touching it; record bypass time |
| S5-M06 | the echo no longer busy-waits: IMU keeps its 10 ms period while the sensor is unplugged (no echo, 30 ms wait per ping) — count `s3_ticks` over 10 s | R1, A2 | manual | Unplug ECHO, run 10 s, read `s3_ticks`. | ≥ 990 ticks (record the old busy-wait build for comparison) |
| S5-M01 | HC-SR04 distance at 10, 20, 30 cm within ±1 cm (assumption A1); ECHO divider fitted | FR-07, A1 | manual | Static obstacle at measured distances. | ±1 cm |
| S5-M02 | servo reaches −60, −30, 0, +30, +60°; set the sign and `SERVO_CENTER_US` | FR-08 | manual | Watch the sensor through a sweep. | angles within ±5° |
| S5-M03 | width estimate within 20 % of true width (3 box sizes) | NFR-09 | manual | Profile boxes 5, 10, 15 cm wide. | error ≤ 20 % |
| S5-M04 | bypass completes in ≤ 5 s and the car re-finds the line ≥ 95 % of 20 runs | NFR-10, NFR-11 | manual | Run the full obstacle course. | ≥ 19 of 20 |
| S5-M05 | **Superseded 2026-11-05 (TEAM.md §7):** the E_STOP state is removed; the other LED patterns stay. Was: status LED: IDLE slow blink, RUNNING on, FAULT medium, E_STOP fast | system statechart | manual | Watch GP18 through the states. | patterns as listed |
| S5-U20 | a decoded character is stored; at the next cross junction the mapped movement is sent (forward = none, left −90, right +90, U-turn 180), then the stored character is cleared and line following continues. Until the MQTT mapping exists only right turn = F is mapped (`JMAP_DEFAULT_RIGHT`, U-turn until 2026-11-09); a–e cover F → right turn → drive on → straight over the same junction on the way back | FR-03 | auto, host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-U21 | no stored character, an unmapped character, or no mapping yet: the car goes forward at the junction; a new mapping replaces the old one (a–b: unmapped and left/right from the table; receiving the mapping over MQTT is not built yet) | FR-03 | auto, host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S5-M20 | demo sequence: barcode, cross junction, mapped movement, repeated for several barcodes, with the mapping sent over MQTT first | FR-03 | manual | Run the demo course twice with two different mappings. | every junction takes the movement mapped to the last barcode |

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
