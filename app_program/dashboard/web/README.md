# CSC11 local connection dashboard

This webpage demonstrates a real Pico W → MQTT broker → browser connection
before final sensor telemetry is implemented. It runs on Jun Kai's laptop.
It is not a hosted website or a firmware web server. No project changes were
committed/pushed by the agent.

## Open the dashboard

With the previously tested Pico transport firmware running and the local broker
running, start the page server in a terminal:

```sh
cd /Users/junkai/Embedded/Embedded-System-Project
python3 app_program/dashboard/web/serve.py
```

Open **http://127.0.0.1:8765/** in your browser. Keep the terminal open while
demonstrating the page. Stop with Ctrl+C. Only this `web/` directory is served;
the ignored WiFi settings file is outside it.

The webpage connects automatically to `ws://127.0.0.1:9001/mqtt`. This address
refers to the laptop's WebSocket listener. The Pico uses the laptop's WiFi IPv4
address on TCP port 1883, not this WebSocket address.

If the broker is stopped, start it in a separate terminal:

```sh
python3 testing/dashboard/broker/start_lan_broker.py
```

Do not start a second copy if it is already running. If the laptop changes IP,
restart the LAN broker launcher, update `MQTT_BROKER_IP` in ignored
`config/wifi_secrets.h`, rebuild with
`sh testing/dashboard/broker/build_mqtt_test.sh`, and reflash the Pico.

## Show Felix

1. **MQTT broker → Connected** confirms browser MQTT/WebSocket connectivity and
   the acknowledged subscription.
2. **Pico signal → Receiving**, the increasing message count and exact bytes
   **8D 23 01 00 FF** demonstrate incoming data on `csc11/dev/pico/tx`.
3. Click **Send test reply**. It publishes those five binary bytes to
   `csc11/dev/pico/rx`, QoS 1, non-retained. The browser reports the broker's ACK;
   the Pico's `[mqtt-test] RX PASS` console line is the end-to-end confirmation.
4. **Disconnect** disables sending; **Reconnect** reconnects and resubscribes.
   Loss of broker connection also triggers automatic reconnect.

The fixed bytes are artificial test data, not sensor readings or real commands.
No E_STOP/RESUME or motor actuation is connected to this page. Sensor fields are
shown as unavailable, not invented values. Final ranges/precision can be agreed
after this demonstration.

## What the indicators mean

- Receiving means a matching, non-retained fixture was received within the last
  three seconds. It does not prove the entire robot is healthy or safe to move.
- The interval is the mean of up to 40 measured inter-arrival times in the
  browser, using its monotonic clock. It is not sensor measurement age, latency,
  round-trip time, firmware execution time or a deadline guarantee.
- Retained fixtures and unexpected payloads are ignored and recorded in the
  activity log. Only fresh exact fixtures update the receiving indicator.
- Counts apply to the current page session; there is no sequence number or
  packet-loss measurement in the fixed fixture. The log is bounded to 35 entries.
- No reply is published automatically when connecting or reconnecting. The send
  button publishes only the fixed test fixture and is disabled while disconnected
  or waiting for its previous acknowledgement.

## Dependencies and checks

The pinned MQTT.js **5.16.1** browser bundle and MIT license are in `vendor/`.
No CDN, external font, npm installation or build is required to run this page.
The bundle was obtained via `npm pack mqtt@5.16.1 --ignore-scripts` and only the
browser bundle/license were copied. Library source/docs:
https://github.com/mqttjs/MQTT.js

Local model checks:

```sh
node --check app_program/dashboard/web/app.mjs
node --test app_program/dashboard/web/model.test.mjs
```

Verified on 2026-10-28:

- HTTP page returned 200 and browser console had no errors.
- Browser connected to the real local broker and received the attached Pico's
  fixture stream, including exact binary values. The displayed short-window
  arrival average was about 501 ms during observation; not a timing guarantee.
- The webpage's reply button received a broker ACK and the physical Pico logged
  `[mqtt-test] RX PASS count=3 bytes=8d230100ff (log only)`.
- Disconnect disabled the reply button and changed indicators to offline;
  reconnect restored subscription and fresh messages.
- Layout inspected in the in-app browser at narrow and wider panel widths.
  No separate full desktop browser/device matrix was tested.
- Three model tests passed for exact binary validation, freshness vs broker-only
  connection, and missing/measured inter-arrival intervals.

Evidence: `../../../testing/dashboard/logs/2026-10-28-dashboard-connection.txt`
and `../../../testing/dashboard/logs/2026-10-28-dashboard-preview.png` (paths
relative to the project root are `testing/dashboard/logs/...`).

## Visual revision, 2026-10-28

Restyled as a compact connection monitor after Jun Kai requested removal of the
generic generated-dashboard aesthetic. Removed the sidebar, logo glyphs,
decorative badges, oversized metric tiles, rounded containers and promotional
headings. Replaced them with a status strip, labelled receive/transmit sections,
a telemetry table, conventional controls and a timestamped log. Colours are
neutral except for operational state. Text remains readable at narrow widths.

MQTT topics, binary validation, subscription behaviour, reply payload/QoS,
freshness thresholds and connection handlers are unchanged. The only JavaScript
edits are payload-status wording. Model checks pass; browser checks verified live
Pico fixtures, exact transmitted reply bytes, disconnect disabling transmission
and reconnect restoring the subscription. Browser console contained no errors.

Design references consulted:
- https://blog.interfacekit.io/what-makes-a-website-look-ai-generated
- https://github.com/febbhav/signs-of-ai-design

These describe recurring conventions, not reliable tests of AI authorship.
Revision evidence: `testing/dashboard/logs/2026-10-28-dashboard-restyle.txt`
and `testing/dashboard/logs/2026-10-28-dashboard-monitor-preview.png`.
