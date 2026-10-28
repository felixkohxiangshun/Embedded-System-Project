# Laptop broker: first bring-up stage

This is a development setup for the one-Pico car architecture. The car will
run communication tasks alongside Mission/sensors/motors; the webpage and broker
run on the laptop. No second Pico relay is needed.

## Installed and verified on 2026-10-28

Mosquitto 2.1.2 was installed with Homebrew (`brew install mosquitto`). No login
service was enabled and Homebrew's broker configuration was not edited. The
repository-local configuration exposes two **loopback-only** listeners:

| Client | Address | Transport |
|---|---|---|
| Local MQTT test client | `127.0.0.1:1883` | MQTT over TCP |
| Future local webpage | `ws://127.0.0.1:9001/mqtt` | MQTT over WebSockets |

Port 9001 carries MQTT WebSocket connections; opening it in a browser does not
show a dashboard. The webpage has not been implemented yet.

Anonymous clients are allowed only on loopback for this stage. Persistence is
disabled. This does not prevent clients retaining messages during a broker run,
and does not implement command freshness or validation.

## Start and stop

Run in a dedicated terminal:

```sh
mosquitto -c /Users/junkai/Embedded/Embedded-System-Project/testing/dashboard/broker/mosquitto.conf -v
```

Keep that terminal open; Ctrl+C stops the broker. If `mosquitto` is not on PATH
on Apple Silicon Homebrew, use `/opt/homebrew/sbin/mosquitto`.
An "address already in use" error means another process already owns a port;
inspect with `lsof -nP -iTCP:1883 -iTCP:9001 -sTCP:LISTEN` before starting another.

## Verify both directions

In a second terminal:

```sh
cd /Users/junkai/Embedded/Embedded-System-Project
python3 testing/dashboard/broker/smoke_test.py
```

Expected and observed output:

```text
PASS TCP -> WebSocket, QoS 0: 8d230100ff
PASS WebSocket -> TCP, QoS 1: 8d230100ff
Broker smoke test PASS; no Pico or physical stopping behavior tested.
```

The script requires the Mosquitto 2.1 command-line clients (`--ws` support),
Python 3 and macOS `/usr/bin/script` for promptly captured subscription output.
It waits for subscription acknowledgement before publishing, sets timeouts,
uses unique `csc11/dev/broker-test/...` topics and checks exact received bytes.
The five-byte fixture deliberately includes `00` and `ff` to catch text handling.
It is artificial test data, not the proposed telemetry format or a car command.
No commands are sent to Mission and no sensors are simulated by this test.

## Next stage: connect the Pico

The current listener binds **only to the laptop**, so a Pico cannot reach it
yet. For Pico bring-up, first choose the shared WiFi network and the Mac's LAN
IPv4 address. Then explicitly bind the TCP listener to the appropriate LAN
address (or prepare a separate LAN configuration) and verify firewall/client
isolation behavior. Keep the WebSocket listener local while the webpage is
only used on this Mac. Decide LAN authentication/access policy at that stage.

Firmware must use the Mac's LAN address, never `127.0.0.1`, for the broker.
Enter WiFi credentials only into the ignored local `config/wifi_secrets.h`.
No credentials or final application topics were set in this stage. The shared
MQTT bridge remains unimplemented and its interface needs team agreement.

## Stage 2: LAN launcher and Pico WiFi test

`start_lan_broker.py` now prepares a temporary configuration using en0's current
IPv4 address. It exposes MQTT on that address and loopback, and WebSockets on
loopback only. Anonymous access is limited by an ACL to `csc11/dev/#`. This is a
temporary development setup for the selected home WiFi/phone hotspot.

Stop any earlier broker before starting this launcher:

```sh
cd /Users/junkai/Embedded/Embedded-System-Project
python3 testing/dashboard/broker/start_lan_broker.py
```

Keep the terminal open. Ctrl+C stops the broker. It prints the current Pico-facing
address. If the Mac changes WiFi networks or IP addresses, restart this launcher
and update the Pico's local `MQTT_BROKER_IP` setting before the next MQTT build.

Observed on 2026-10-28: Mac address changed from `10.132.2.29` to `172.20.10.3`.
The launcher detected the new address. With that broker running, this passed:

```sh
python3 testing/dashboard/broker/smoke_test.py --tcp-host 172.20.10.3
```

Both binary directions passed. This connects from the Mac to its own LAN address;
it does not prove hotspot peer connectivity or a successful Pico connection.

An ignored local `config/wifi_secrets.h` was prepared from the example with
`MQTT_BROKER_IP` set to `172.20.10.3`, client ID `csc11-pico-junkai-dev`, and
development topic `csc11/dev/heartbeat`. The WiFi name/password placeholders need
to be replaced locally by Jun Kai. Do not commit or share that file.

After entering credentials, rebuild the **WiFi-only** test profile:

```sh
cd /Users/junkai/Embedded/Embedded-System-Project/build_make
make -j8 SMP=0 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=0 TEST=1
```

Compilation was verified with placeholder WiFi settings; no new image was flashed
by the agent. Rebuild after entering the real settings. Output is
`mtk3pico_smp0_usb_cdc_wifi_lwip_test.uf2` (different from the earlier radio-disabled
image). With motor power disconnected, flash this image and capture USB serial.
Look for `wifi: link up, ip=...`. Association alone is insufficient; an IP address
is needed. Capture early `wifi:` lines as well if the join fails.

MQTT remains disabled in this profile. Its existing demo publishes text; it has
not yet been converted into the required binary application bridge. No dashboard
page, remote command path or physical stopping behavior is tested in this stage.

## Scope of evidence

### Diagnostic network scan

For SSID-not-found troubleshooting, use the existing radio scan code without
editing shared library or Make files:

```sh
cd /Users/junkai/Embedded/Embedded-System-Project
sh testing/dashboard/broker/build_wifi_scan.sh
```

This sets the compiler macro `WIFI_DIAGNOSTIC_SCAN=1` and uses a distinct build
profile and image name to avoid stale objects. It compiled successfully on
2026-10-28. Output:
`build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_scan_test.uf2`.
Normal builds do not enable the scan. MQTT remains disabled.

The scan runs before association and can delay boot by around 10 seconds. It is
diagnostic firmware, not a timing-validation or deployment image. To capture
early output, put the Pico in BOOTSEL first. In a terminal in a log directory,
arm the serial monitor before copying the UF2 from a second terminal:

```sh
mkdir -p /tmp/inf2004-wifi-scan
cd /tmp/inf2004-wifi-scan
until [ -e /dev/cu.usbmodemMTK3PICO1 ]; do sleep 0.2; done
screen -L /dev/cu.usbmodemMTK3PICO1 115200
```

In the other terminal:

```sh
cp -X /Users/junkai/Embedded/Embedded-System-Project/build_make/mtk3pico_smp0_usb_cdc_wifi_lwip_scan_test.uf2 /Volumes/RPI-RP2/
```

After around 30 seconds, exit screen (Ctrl+A, release, K, Y) and inspect the
newest `wifi:` lines in `/tmp/inf2004-wifi-scan/screenlog.0`. The scan prints SSID,
RSSI, channel and authentication code, then attempts the configured connection.
Some very early results can still precede the host opening USB serial. The
presence of an SSID does not prove authentication or DHCP success; only the
later link-up/IP message establishes a working IP connection. Scan results may
contain other nearby network names; share only the relevant lines.

The two PASS results verify local binary routing through the broker on both
transports. They do not verify a browser client, Pico connection, 500 ms
telemetry, reconnect behavior, stale-command rejection, motor timing, or stopping
distance. No firmware was modified, flashed or built for this broker-only step.

Reference: https://mosquitto.org/man/mosquitto-conf-5.html
