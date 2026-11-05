# Pico binary MQTT transport test

This is an opt-in diagnostic, not the final robot telemetry/command bridge.
The spare Pico W can be used. No sensors are needed. Do not enable motor power
when running these diagnostics on the car.

## What it does

- Uses the existing radio/lwIP service task and micro T-Kernel. No student task
  calls lwIP and no extra task/timer chain is created.
- Connects to the ignored local `MQTT_BROKER_IP`/port with `MQTT_CLIENT_ID`.
- Subscribes to `csc11/dev/pico/rx` at QoS 1, and reports ONLINE after SUBACK.
- Publishes five artificial bytes `8d 23 01 00 ff` on `csc11/dev/pico/tx` at
  QoS 0, non-retained, with a 500 ms target interval. No missed-period burst.
- Accepts only the same five bytes on the RX topic, assembles fragments, validates
  total length/content and logs PASS or REJECT. No input is sent to Mission.
- Retries failed MQTT connections with 1/2/4/8/16-second capped backoff, and
  resubscribes on reconnect. The existing WiFi service owns WiFi reconnection.

The fixture tests transport of NUL/non-ASCII bytes and defines no sensor ranges,
telemetry schema, heartbeat schema or command format. It cannot authorize RESUME.
RX PASS establishes only that the service received this fixture. It does not
prove command freshness, timing deadlines, system health or physical stopping.

## Build

Enter actual WiFi settings in the ignored `config/wifi_secrets.h`. The confirmed
WiFi test used SSID `JK` and Pico IP `172.20.10.2`; the Mac was `172.20.10.4` on
2026-10-28. Recheck the Mac address if the network changes.

```sh
cd /Users/junkai/Embedded/Embedded-System-Project
sh testing/dashboard/broker/build_mqtt_test.sh
```

This excludes the shared demo's `MQTT_PORT_OBJS` and compiles the opt-in
`app_program/dashboard/mqtt_transport_test.c` implementation of `mqtt_utk_start`.
No shared library/build files are edited. Its distinct profile invalidates stale
objects, and normal builds compile this file to an empty unit. The final IF-12
bridge remains unimplemented and subject to the team's recorded review process.

Output: `build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_transport_test.uf2`.
**Use the wrapper**, not just `make MQTT=1`, which would build the old text demo.

## Broker

One broker must be running, bound to the current Mac WiFi address:

```sh
python3 testing/dashboard/broker/start_lan_broker.py
```

The agent restarted the broker on `172.20.10.4` for this test. No Homebrew login
service is enabled. If it is already running, don't start a second broker.
The LAN launcher permits only development topics, including the two above.
If macOS asks about incoming connections for Mosquitto, allow them for this test
network. Connection across the hotspot must still be verified from the board.

## Hardware check, two terminals

In terminal 1, begin listening before flashing:

```sh
mosquitto_sub -h 127.0.0.1 -p 1883 -t csc11/dev/pico/tx -q 0 -R -C 5 -W 90 -F '%x'
```

Put the Pico in BOOTSEL. In terminal 2:

```sh
cp -X /Users/junkai/Embedded/Embedded-System-Project/build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_mqtt_transport_test.uf2 /Volumes/RPI-RP2/
```

Then open serial in terminal 2 (rediscover the port if needed):

```sh
screen -L /dev/cu.usbmodemMTK3PICO1 115200
```

Terminal 1 should print `8d230100ff` five times and return to its prompt. This
proves actual Pico-to-broker payload delivery if these messages are observed
during this run; a TX accepted console counter alone does not prove delivery.
In terminal 1, send a binary return fixture:

```sh
python3 -c 'import sys; sys.stdout.buffer.write(bytes.fromhex("8d230100ff"))' | mosquitto_pub -h 127.0.0.1 -p 1883 -t csc11/dev/pico/rx -q 1 -s
```

Terminal 2 should report:

```text
[mqtt-test] RX PASS count=1 bytes=8d230100ff (log only)
```

To test rejection, send a short fixture on that same development topic:

```sh
python3 -c 'import sys; sys.stdout.buffer.write(bytes.fromhex("8d23"))' | mosquitto_pub -h 127.0.0.1 -p 1883 -t csc11/dev/pico/rx -q 1 -s
```

Expect `[mqtt-test] RX REJECT count=...`, with no Mission interaction. Log counts
can differ after previous messages. Do not set retained publishing (`-r`).

Later reconnect check: stop/restart the test broker, observe delayed retries,
ONLINE after a fresh subscription, outgoing fixtures and a successful RX fixture.
This reconnect sequence still needs physical execution, not just mock tests.

## Validation recorded on 2026-10-28

- WiFi physical result supplied by Jun Kai: `[wifi-test] UP ip=172.20.10.2`.
- Mac reached the Pico with ICMP while WiFi-only firmware was running.
- MQTT diagnostic cross-build passed without compiler warnings.
- Host callback tests use real SDK MQTT declarations and mocked I/O. PASS under
  C99 warnings-as-errors and address/undefined sanitizers: no duplicate connect
  while pending, subscription-before-publication, 500 ms scheduling, no catch-up
  bursts, publish-error counting, exact binary/fragment handling, malformed/short/
  oversized/unknown-topic rejection, failed SUBACK, reconnect resubscription,
  capped retry backoff and clock rollover. Run:

  ```sh
  sh testing/dashboard/broker/test_mqtt_transport.sh
  ```

- Local broker binary TCP/WebSocket tests passed again at Mac IP `172.20.10.4`.
- Normal WiFi-only build was also checked to ensure the opt-in diagnostic does
  not replace the normal networking implementation.
- Pico MQTT delivery and valid RX were subsequently confirmed on hardware by
  Jun Kai: Mac subscriber received five `8d230100ff` fixtures; the saved Pico
  console contains `[mqtt-test] RX PASS count=1 bytes=8d230100ff (log only)`.
  Evidence extract: `../logs/2026-10-28-mqtt-transport.txt`.
- Hardware rejection of short/oversized/wrong-content fixtures and broker-restart
  recovery were subsequently verified by Codex on the attached Pico. Five fresh
  outbound messages and RX PASS after restart confirm both directions resumed.
  Evidence: `../logs/2026-10-28-mqtt-rejection-reconnect.txt`. One earlier publish
  failure was counted; transport continued, so loss-free delivery is not claimed.
  No timing or
  minimum-bit RAM-reduction claim is made; the existing console telemetry is
  still JSON and is not published by this isolated transport test.
