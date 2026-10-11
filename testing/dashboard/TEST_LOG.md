# Dashboard — test log (test IDs S4-…)

**Subsystem:** Online dashboard (WiFi / MQTT)  ·  **Owner:** Jun Kai  ·  **Code:** `app_program/dashboard/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic: `cd testing/host && make run`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | telemetry JSON has the expected fields | FR-16 | auto, host | `dashboard_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | worst-case frame ≤ 512 bytes (measured 200) | NFR-13 | auto, host | `dashboard_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | too-small buffer is refused, not overrun | NFR-13 | auto, host | `dashboard_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | E_STOP / RESUME parsed, junk ignored | FR-17 | auto, host | `dashboard_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | connection statechart: WIFI → MQTT → ONLINE → HEARTBEAT → RECONNECTING | 6.5, FR-14, FR-15, FR-18 | auto, host | `dashboard_test.c` | `[TEST] S4-U05 PASS` |
| S4-I01 | Telemetry sends a frame every 500 ms | FR-16 | auto | `dashboard_test.c` | `[TEST] S4-I01 PASS` |
| S4-M01 | WiFi joins the venue SSID (needs `NET=lwip MQTT=1` and `config/wifi_secrets.h`) | FR-14 | manual | Boot, watch console / `cyw43_utk_get_status()`. | joined and IP within 30 s |
| S4-M02 | **blocked until IF-12 is agreed:** broker receives a frame every 500 ms at QoS 0 | FR-15, FR-16, NFR-12 | manual | Subscribe to the topic with an MQTT client. | one frame per 500 ms |
| S4-M03 | **blocked until IF-12 is agreed:** E_STOP from the dashboard halts the car; RESUME re-enters LINE_FOLLOW | FR-17, NFR-12 | manual | Publish E_STOP then RESUME. | motors stop / restart |
| S4-M04 | heartbeat when no telemetry is sent | FR-18 | manual | Stop Telemetry (or pull the car's data) for 3 s. | heartbeat every 1 s |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S4-U01, S4-U02, S4-U03, S4-U04, S4-U05 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
