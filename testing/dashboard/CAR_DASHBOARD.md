# Car dashboard: live telemetry and debug in the browser

Shows the car's binary telemetry (TEAM.md section 7) and a debug stream on a
web page, so you can debug without the USB serial monitor.

```
car (Pico W) --WiFi, MQTT--> Mosquitto on the Mac --WebSocket--> car.html
```

| Topic | Frame | Rate | Contents |
|---|---|---|---|
| `csc11/dev/car/telemetry` | 11 bytes | 500 ms | state, speed, distance, heading, line seen, obstacle, last and stored character, tallest hump, shock, faults |
| `csc11/dev/car/barcode` | 3 + 2 per width (≤ 81 bytes) | once per barcode capture | seq, decode result, edge count, then each bar/space width as u16 LE in 0.1 ms, bar first |
| `csc11/dev/car/debug` | 24 bytes | 100 ms | darkC, darkR, line error, steering, line mode, barcode edges/decoded/failed/last, last decode result and edge count, ultrasonic mm, motor state, scan state, last sweep mm at all five positions, ultrasonic echo start delay and edge count and chosen side |
| `csc11/dev/car/heartbeat` | 1 byte | 1 s, only when telemetry stops | system state |
| `csc11/dev/car/cmd` | | | subscribed, for the movement mapping later |

Frames are packed by `dash_pack_telem()` / `dash_pack_debug()` in
`app_program/dashboard/dash_logic.c` with `protocol_bits`: each field uses the
bits its maximum needs, in order, **least significant bit first**. The page's
decoder is `app_program/dashboard/web/car_model.mjs`; both have tests.
Note: TEAM.md section 7 still says "most significant bit first"; this code uses
Jun Kai's `protocol_bits` order (least significant first). Agree one and update
the other.

## One-time setup

1. `brew install mosquitto` (already done on Felix's Mac).
2. Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h` and fill in:
   - `WIFI_SSID`, `WIFI_PASSWORD`: the network both the Mac and the car join
     (a phone hotspot works).
   - `MQTT_BROKER_IP`: the Mac's address on that network. The broker launcher
     below prints it. Never `127.0.0.1`.
   - `MQTT_CLIENT_ID`: any unique name, e.g. `csc11-car-felix`.

   `wifi_secrets.h` is ignored by git. Never commit it.

## Every session

Terminal 1, the broker (keep open; it prints the address for `MQTT_BROKER_IP`):

```sh
python3 testing/dashboard/broker/start_lan_broker.py
```

Terminal 2, build and flash the dashboard firmware:

```sh
sh testing/dashboard/broker/build_dashboard.sh
cp build_make/mtk3pico_dashboard.uf2 /Volumes/RPI-RP2/
```

(or with the Debug Probe, `program build_make/mtk3pico_dashboard.elf`.)
The script builds single-core (`SMP=0`): in the dual-core build the motor task
never ran, so the car would not drive (seen 2026-11-07).
If the Mac's address changed, update `MQTT_BROKER_IP` and rebuild first.

Terminal 3, the web page:

```sh
python3 app_program/dashboard/web/serve.py
```

Open http://127.0.0.1:8765/car.html. The badge turns green ("Car live") once
frames arrive. Without WiFi the car still drives: Mission leaves INIT after 20 s.

## If nothing arrives

| Symptom | Check |
|---|---|
| Page says "Broker offline" | Terminal 1 is running; the page's broker field is `ws://127.0.0.1:9001/mqtt` |
| "Waiting for car" | USB console shows `[mqtt] connecting to ...`: is `MQTT_BROKER_IP` the Mac's current address? Same WiFi? Mac firewall allows incoming 1883? |
| Console shows no `wifi:` link up | wrong `WIFI_SSID` / `WIFI_PASSWORD`, or 5 GHz-only network (the Pico W needs 2.4 GHz) |
| `[mqtt] connection status=5` | broker refused the client: restart the launcher |

Watch raw frames without the page:

```sh
mosquitto_sub -h 127.0.0.1 -t 'csc11/dev/car/#' -v -F '%t %x'
```
