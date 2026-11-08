# Dashboard — test log (test IDs S4-…)

**Subsystem:** Online dashboard (WiFi / MQTT)  ·  **Owner:** Jun Kai  ·  **Code:** `app_program/dashboard/`

See `testing/README.md` for how to run tests and record results.
Host (PC) runs of the pure logic: `cd testing/host && make run`.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | **Superseded 2026-11-05 (TEAM.md §7):** telemetry becomes the 11-byte binary frame, see S4-U10. Was: telemetry JSON has the expected fields | FR-16 | auto, host | `dashboard_test.c` | `[TEST] S4-U01 PASS` |
| S4-U02 | worst-case frame ≤ 512 bytes (measured 200) | NFR-13 | auto, host | `dashboard_test.c` | `[TEST] S4-U02 PASS` |
| S4-U03 | too-small buffer is refused, not overrun | NFR-13 | auto, host | `dashboard_test.c` | `[TEST] S4-U03 PASS` |
| S4-U04 | **Superseded 2026-11-05 (TEAM.md §7):** remote E-stop and RESUME removed. Was: E_STOP / RESUME parsed, junk ignored | FR-17 | auto, host | `dashboard_test.c` | `[TEST] S4-U04 PASS` |
| S4-U05 | connection statechart: WIFI → MQTT → ONLINE → HEARTBEAT → RECONNECTING | 6.5, FR-14, FR-15, FR-18 | auto, host | `dashboard_test.c` | `[TEST] S4-U05 PASS` |
| S4-I01 | Telemetry sends a frame every 500 ms | FR-16 | auto | `dashboard_test.c` | `[TEST] S4-I01 PASS` |
| S4-M01 | WiFi joins the venue SSID (needs `NET=lwip MQTT=1` and `config/wifi_secrets.h`) | FR-14 | manual | Boot, watch console / `cyw43_utk_get_status()`. | joined and IP within 30 s |
| S4-M02 | **blocked until IF-12 is agreed:** broker receives a frame every 500 ms at QoS 0 | FR-15, FR-16, NFR-12 | manual | Subscribe to the topic with an MQTT client. | one frame per 500 ms |
| S4-M03 | **Superseded 2026-11-05 (TEAM.md §7):** remote E-stop and RESUME removed. Was: **blocked until IF-12 is agreed:** E_STOP from the dashboard halts the car; RESUME re-enters LINE_FOLLOW | FR-17, NFR-12 | manual | Publish E_STOP then RESUME. | motors stop / restart |
| S4-M04 | heartbeat when no telemetry is sent | FR-18 | manual | Stop Telemetry (or pull the car's data) for 3 s. | heartbeat every 1 s |
| S4-B01 | range-based binary bit packing primitive | Minimum-bit groundwork; no final schema | auto, host | Compile/run `protocol_bits_test.c` as described in `BRINGUP_REVIEW.md` | Known bytes, boundaries, invalid input and widths/offsets pass under address/undefined sanitizers |
| S4-B02 | binary packing on the RP2040 | Minimum-bit groundwork; no final schema | auto, target | Build `SMP=0 WIFI=none NET=none MQTT=0 TEST=1`; flash and capture USB console | `[TEST] S4-B02 PASS` for known bytes, decode, invalid range and short-buffer rejection; hardware execution pending |
| S4-B03 | opt-in MQTT transport scheduling/callback validation | Transport groundwork, not IF-12 integration | auto, host | `sh testing/dashboard/broker/test_mqtt_transport.sh` | Subscription gating, binary fragment/length checks, retry cap, resubscription and clock-wrap checks pass |
| S4-M05 | Pico binary transport in both directions | MQTT diagnostic, not real telemetry/commands | manual, target | Follow `broker/PICO_MQTT_TEST.md` | Mac receives five `8d230100ff` messages; Pico logs RX PASS for exact reply and REJECT for invalid length |
| S4-M06 | Broker-restart recovery of diagnostic transport | MQTT reconnect/resubscribe groundwork | manual, target | Stop/restart broker; receive five fresh outbound fixtures, then send valid binary reply | Connection/subscription restored, both directions resume |
| S4-W01 | Laptop webpage to broker/Pico connection | Dashboard connection demonstration | manual, browser + target | Run local `web/serve.py`; verify live fixture; click Send test reply; check Pico RX PASS; disconnect/reconnect | Broker/subscription indicators, exact binary messages and physical RX PASS; no Mission commands |
| S4-U10 | binary telemetry: pack a known frame into 11 bytes and unpack it, every field equal; out-of-range values clamped | FR-16, NFR-13 | auto, host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S4-U12 | debug frame: pack a known frame into 14 bytes and unpack it, every field equal; out-of-range values clamped | debugging aid | auto, host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S4-U13 | barcode trace frame (`csc11/dev/car/barcode`): edge times packed as widths in 0.1 ms, long widths clamped to 65535, too many edges or a short buffer refused | debugging aid | auto, host | `host_tests.c` | `PASS` from `make -C testing/host run` |
| S4-W02 | car dashboard page decodes telemetry and debug frames packed by the firmware code | FR-16 | auto, host + manual | `node --test app_program/dashboard/web/car_model.test.mjs`; then `testing/dashboard/CAR_DASHBOARD.md` | tests pass; page shows live values from the car |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console
(`make TEST=1`); **host** = the same check also runs on a PC (`testing/host`);
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-11 | S4-U01, S4-U02, S4-U03, S4-U04, S4-U05 | *(uncommitted)* | *(Claude, port)* | PASS | `make -C testing/host run` | host (PC) run of testing/host; not yet run on the board |
| *(none yet)* | | | | | | on-target `TEST=1` run and manual tests still to do |
| 2026-10-23 | S4-U01..S4-U05 | c1cb8d9 unchanged baseline | Codex for Jun Kai | PASS | `make -C testing/host run`: 80 passed, 0 failed overall | Current JSON/text logic only; not binary protocol compliance or hardware evidence |
| 2026-10-23 | S4-B01 | c1cb8d9 + local protocol_bits files | Codex for Jun Kai | PASS | `protocol_bits: known vector, invalid inputs, all widths/offsets PASS` | C99 warnings-as-errors; address/undefined sanitizers; no board |
| 2026-10-28 | S4-B02 | 99417c6 | Jun Kai | PASS | User supplied `[TEST] S4-B02 PASS` and `dashboard SUMMARY 16 passed, 0 failed` | Physical Pico execution; unrelated S3-H01 IMU failure reported |
| 2026-10-28 | S4-M01 (join/IP only) | 99417c6 + local test-only WiFi reporting | Jun Kai | PASS (partial) | User supplied `[wifi-test] UP ip=172.20.10.2` | SSID JK; WiFi/DHCP confirmed; join deadline not measured; MQTT disabled |
| 2026-10-28 | S4-B03 | 99417c6 + local MQTT diagnostic | Codex for Jun Kai | PASS | Host callback tests under address/undefined sanitizers | Mocked network I/O; real SDK declarations; hardware MQTT results pending |
| 2026-10-28 | S4-M05 (positive paths) | 99417c6 + local MQTT diagnostic | Jun Kai; log checked by Codex | PASS (partial) | `logs/2026-10-28-mqtt-transport.txt`: five exact outbound fixtures and Pico RX PASS | Hardware binary delivery confirmed both ways; invalid-length rejection and reconnect tests still pending |
| 2026-10-28 | S4-M05 (invalid inputs), S4-M06 | 99417c6 + local MQTT diagnostic | Codex for Jun Kai | PASS | `logs/2026-10-28-mqtt-rejection-reconnect.txt`: three RX REJECTs, reconnect/ONLINE and RX PASS after restart | Exact fixture protocol only; retry timing, message freshness and motor behavior not validated; previous publish failure count remained 1 |
| 2026-10-28 | S4-W01 | 6b7b3ba + local web files | Codex for Jun Kai | PASS | `logs/2026-10-28-dashboard-connection.txt`, screenshot and actual Pico RX PASS count=3 | Browser ↔ broker ↔ Pico verified using fixture; sensor telemetry and motor commands remain unimplemented |
| 2026-10-28 | S4-W01 (visual revision regression) | 6b7b3ba + local web revision | Codex for Jun Kai | PASS | `logs/2026-10-28-dashboard-restyle.txt` and monitor screenshot | Live receive, exact binary reply, disconnect/reconnect and log clearing checked; three model tests pass; no browser console errors |
