# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | follow the black line track autonomously | FR-01 | manual | Press start; run 5 laps. | stays on the line |
| SYS-02 | **Superseded 2026-11-05 (TEAM.md §7):** commands are now carried out at the next cross junction, see SYS-20. Was: barcode decode and command execution | FR-02, FR-03, NFR-05 | manual | 20 passes over A/B/C/D. | ≥ 95 % correct |
| SYS-03 | hump detection and tallest height | FR-04, FR-05, NFR-07 | manual | 3 humps of known height. | within ±5 mm; tallest reported |
| SYS-04 | shock detection | FR-06 | manual | Knock the chassis. | SHOCK flag in telemetry |
| SYS-05 | obstacle detection, profile, bypass, line reacquisition | FR-07..FR-10, NFR-08..NFR-11 | manual | 20 runs with a box on the line. | ≥ 95 % detect and reacquire; ≤ 5 s bypass |
| SYS-06 | speed regulation, distance, drift | FR-11..FR-13, NFR-14 | manual | Drive 2 m, compare encoder distance with a tape measure. | speed within ±10 %; distance within ±5 % |
| SYS-07 | WiFi and MQTT connect by themselves | FR-14, FR-15 | manual | Power on at the venue. | online within 30 s |
| SYS-08 | telemetry every 500 ms, ≤ 512 bytes, QoS 0 | FR-16, NFR-12, NFR-13 | manual | Subscribe to the topic for 2 minutes. | one frame per 500 ms |
| SYS-09 | **Superseded 2026-11-05 (TEAM.md §7):** remote E-stop and RESUME removed. Was: remote E_STOP and RESUME | FR-17 | manual | Publish E_STOP then RESUME. | motors stop / restart |
| SYS-10 | heartbeat when quiet | FR-18 | manual | Stop telemetry. | heartbeat every 1 s |
| SYS-11 | recovery from sensor failure without restart | FR-19 | manual | Unplug the IMU, cover the IR sensors, unplug an encoder, one at a time. | car recovers or stops safely; no restart needed |
| SYS-12 | code builds with `-Wall -Wextra -Werror` and follows Barr-C | NFR-01, NFR-02 | manual | Build with those flags; run a Barr-C check. | no warnings |
| SYS-20 | demo: barcodes scanned in either direction decode correctly, are stored, and each next cross junction takes the movement mapped over MQTT; repeated over the course | FR-02, FR-03, NFR-05 | manual | Instructor sends the mapping, then run the course. | every barcode decoded, every junction correct |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| *(none yet)* | | | | | | the car has not run on the board since the port |
