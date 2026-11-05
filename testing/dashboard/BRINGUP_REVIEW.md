# Dashboard bring-up and team review

Prepared 2026-10-23 for Jun Kai. Repository baseline: `c1cb8d9`.
This is a review proposal, not an approved interface or completed integration.

## What was inspected and measured

- A clean local checkout exists at `/Users/junkai/Embedded/Embedded-System-Project`.
  The GitHub connector still returns 404; local inspection is available.
- This checkout uses GNU Make, C (`gnu11`), and a micro T-Kernel 3.0 RP2040
  port with optional SMP. Default `SMP=1` is an existing build setting, not
  evidence that the lecturer requires CSC11 to migrate to SMP.
- Installed SDK is 2.3.0; README specifies 2.2.0. Confirm the team's supported
  SDK before treating results as reproducible against its environment.
- Unchanged default build passed: `make -j8` in `build_make`.
  This profile has USB/radio support, `NET=none`, `MQTT=0`; it does not join WiFi.
- Unchanged network build passed with `NET=lwip MQTT=1` and a temporary include
  directory containing the example credentials. No real credentials were added.
  These images use placeholder settings and were not flashed.
- Existing host tests: **80 passed, 0 failed**. These test the current JSON and
  text-command implementation, not compliance with the binary change requirement.
- No Pico USB serial port or RPI-RP2 volume was present. Network and motor
  runtime behavior have not been tested.

ELF section sizes from `arm-none-eabi-size`:

| Unchanged profile | text | data | bss |
|---|---:|---:|---:|
| SMP1, USB, radio, no IP/MQTT | 292956 | 96 | 27116 |
| SMP1, USB, radio, lwIP, MQTT | 346084 | 104 | 84892 |

Sizes are bytes. These profiles differ in enabled networking features; this is
**not** a before/after compression comparison. Section totals do not establish
peak runtime RAM consumption, heap use, stack high-water marks or timing.

## Existing code gaps

1. `dash_logic.c` formats JSON and parses commands by substring. Unknown text
   containing `RESUME` can be accepted. No freshness/session checks exist.
2. `mqtt_utk.c` is a demo: it publishes text `hb N` every 5 s and subscribes
   to that same topic at QoS 0. It does not service the dashboard buffers.
3. Its receive callback replaces nonprintable bytes with `.`. Binary commands
   require preserving original bytes and rejecting oversized/incomplete messages.
4. Dashboard's connection statechart does not itself connect or reconnect the
   network. Its two-second state delay is not proof of network retry backoff.
5. Telemetry queues four frames. Draining to the newest received frame cannot
   replace a newer snapshot that was rejected when the buffer was full.
6. Command sends ignore full-buffer failures; STOP precedence is undefined.
7. Mission currently allows INIT to reach IDLE after 20 s offline. This differs
   from the handover's startup connection policy; Felix must resolve it.
8. `TEAM.md` restricts networking calls to the radio service task and says the
   IF-12 library bridge needs team/instructor agreement. Implementation should
   use that single-owner service, rather than call lwIP from Dashboard.

## Small code component added

`app_program/dashboard/protocol_bits.c/.h` is a bounded, allocation-free
range-based bit encoder/decoder. For an unsigned code in `0..maximum`, its
width is the number of significant bits in `maximum`. Thus codes `0..10`
need 4 bits; codes `0..4` need 3. A constant consumes zero bits.

The **proposed** convention is least-significant bit first within fields and
bytes. It uses explicit shifts/masks; it never transmits a C struct. Rejected
fields leave the cursor and output unchanged. Unused bit patterns above the
declared maximum are rejected. Callers must zero packet storage first and must
own it exclusively while encoding. This helper provides no synchronization.

This component is not connected to live telemetry or commands. Signed/scaled
fields, message boundaries, padding, versioning, replay checks and the final
packet layout still need the schema below. The linker removes the unused helper,
so the network image section sizes remain unchanged. No RAM saving is claimed.

Run host verification from `testing/dashboard`:

```sh
cc -std=c99 -Wall -Wextra -Werror -fsanitize=address,undefined -g \
  protocol_bits_test.c ../../app_program/dashboard/protocol_bits.c \
  -o /tmp/inf2004-protocol-bits-test
/tmp/inf2004-protocol-bits-test
```

Observed PASS: independent known byte vector `8d 23 01`, malformed field values,
short buffers, cursor overflow, null arguments, zero-bit constants, clearing
previous bits, and round trips for widths 1–32 at offsets 0–7. Sanitizers reported
no errors. Test fields are artificial and do not define robot sensor ranges.

## Decisions to bring to the team

### 1. Ownership and network bridge

Propose retaining the existing single-owner radio/lwIP service task. Add a
bounded, nonblocking poll for outgoing binary packets and an incoming-command
path in that service. Preserve all received bytes, validate exact topic and
advertised/assembled lengths, subscribe at QoS 1 after every successful connect,
and report ONLINE only once the command subscription succeeds. Define retry
limits/backoff and counters for connection, subscription and queue failures.

The existing IF-12 `MQTT_MSG` is a 512-byte text-oriented envelope. Revise its
capacity only after calculating the final maximum binary packet size, including
command metadata. A smaller MQTT payload alone does not shrink these buffers.
Shared library/config/interface changes require the agreement recorded in TEAM.md.

### 2. Telemetry handoff

Propose replacing IF-09's message buffer with one mutex-protected latest-unsent
slot, owned by Telemetry/Dashboard. Producer copies the new snapshot and counts
replacement of a pending one. Consumer copies to private storage and clears pending
under the same lock. It releases the lock before encoding or networking.

Agree separately how shock/fault events are latched or counted and acknowledged;
do not clear an event just because a newer periodic sample has no event. Mission
owns event semantics. Keep commands on a separate reliable handling path.
Record lock durations, replacements, send failures and received sample age.
This proposal is not implemented and cannot remove downstream TCP/broker backlog.

### 3. Telemetry schema worksheet

For **each** quantity, agree minimum/maximum, unit, resolution, signedness,
invalid/unavailable representation, overflow action and owner. Then calculate
the number of codes (including sentinels) and `ceil(log2(code_count))` bits.

| Field | What the checkout establishes | Still needs agreement |
|---|---|---|
| Mission state | 11 codes, SYS_INIT through SYS_E_STOP | 4 bits if these remain the only codes; reserve/invalid policy |
| Heading | Encoder-derived in task_telemetry.c, degrees, positive right | Normalization, range, resolution, validity and calibration |
| Speed | mm/s | Signed range, precision and invalid policy |
| Cumulative distance | mm | Run length limit, reset/wrap/saturation policy |
| Obstacle distance | mm | Sensor limit, no echo/stale/invalid codes |
| Last barcode | none plus A–D | Proposed mapping 0=none, 1–4=A–D; 3 bits if accepted |
| Peak hump height | mm | Range, precision, validity and reset policy |
| Shock | Current flag | 1-bit value; event persistence/counting semantics |
| Fault flags | Four named faults | 4 bits for those faults; event persistence |
| IR state | Current single line-seen flag | Whether the team needs individual sensors instead |
| Encoder counts | Present in current frame | Inclusion, signed range, wrap/reset policy |
| Time/sequence | Uptime present, no sequence | Age measurement, clock relationship, sequence width/wrap |

No final packet size is claimed. Agree packet type/version, field order, bit
order, length and padding rules as part of the schema. Heartbeat must be binary
too; agree contents and interval independently of MQTT's keepalive.

### 4. Commands and stopping behavior

Agree a binary command envelope with exact length, opcode mapping and session /
freshness information. E_STOP/RESUME are two operations and can use one opcode
bit if both bit values are assigned; the rest of the envelope still needs enough
information to reject stale RESUME. Define duplicate handling, session changes,
sequence wrap and expiration before choosing widths. Do not infer freshness from
exact command matching or QoS 1.

Propose non-retained command publishing at QoS 1, a fresh session handshake after
reconnect, and rejection of commands from older sessions. Verify what MQTT
metadata this lwIP client exposes before choosing how retained deliveries are
handled. Mission must gate RESUME on E_STOP and required fault conditions.

Agree STOP precedence when the command channel is full; report failures rather
than silently dropping. A separate STOP latch/event may be appropriate but needs
Felix/Dave agreement. Initial development must log decoded commands without
forwarding to motors. Network loss policy and the 20-second startup fallback need
explicit acceptance. Physical stopping distance requires a later board/car test.

### 5. Broker and dashboard

Agree broker host/software, TCP port, authentication if used, exact telemetry,
status and command topics, and dashboard runtime. MQTT_TOPIC in the example is
a demo heartbeat topic, not a project decision. A browser dashboard also needs
an MQTT-over-WebSocket listener or a backend bridge; plain MQTT TCP port 1883
alone is insufficient for a browser client.

## Next implementation sequence

1. Agree the worksheet and bridge ownership above.
2. Implement one shared packet specification with known byte vectors and
   matching firmware/dashboard decoders; reject malformed packets atomically.
3. Simulate telemetry at 500 ms and decode binary commands into logs only.
4. Implement agreed latest-slot/event behavior and network-service bridge.
5. Test broker restart, reconnection, resubscription, duplicate/stale commands,
   full channels and bounded callback work before enabling Mission forwarding.
6. With the board: verify WiFi join/DHCP, then MQTT, then integrate with Felix
   and Dave for command-to-stop timing, distance and Motor's 10 ms deadlines.
7. Measure equivalent-feature before/after RAM, payload sizes and performance;
   do not present today's baseline builds as runtime or minimum-bit compliance.
