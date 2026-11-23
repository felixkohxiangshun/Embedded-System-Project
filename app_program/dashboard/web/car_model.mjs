// Decoders for the car's binary frames (TEAM.md section 7).
// Must match dash_pack_telem() / dash_pack_debug() in dash_logic.c:
// protocol_bits packs each field into the bits its maximum needs, in order,
// least significant bit first, starting at bit 0 of byte 0.

export const TOPICS = Object.freeze({
  telemetry: 'csc11/dev/car/telemetry',
  debug: 'csc11/dev/car/debug',
  heartbeat: 'csc11/dev/car/heartbeat',
  barcode: 'csc11/dev/car/barcode',
  command: 'csc11/dev/car/cmd',
});
// Commands on TOPICS.command: exactly one byte, bit 0 = 1 for START, 0 for
// STOP, bits 1..7 zero (dash_parse_cmd() in dash_logic.c).
export const COMMANDS = Object.freeze({ start: 1, stop: 0 });
export function encodeCommand(name) {
  if (!(name in COMMANDS)) throw new Error(`unknown command ${name}`);
  return Uint8Array.of(COMMANDS[name]);
}

// Which drive button the car would act on (mission_on_mqtt()): Start only
// from IDLE, Stop only while running or recovering.  Both off while the
// broker or the car is not live, or the state is unknown.
const RUNNING = ['LINE_FOLLOW', 'BARCODE_SCAN', 'BARCODE_CMD', 'HUMP_TRAVERSE', 'OBSTACLE_SCAN',
  'OBSTACLE_BYPASS', 'LINE_REACQUIRE', 'FAULT_RECOVERY'];
export function driveButtons(stateName, live, connected) {
  const ok = Boolean(live && connected);
  return { start: ok && stateName === 'IDLE', stop: ok && RUNNING.includes(stateName) };
}

export const TELEM_BYTES = 11;
export const DEBUG_BYTES = 24;
export const CHAR_NONE = 31;

export const SYS_STATES = ['INIT', 'IDLE', 'LINE_FOLLOW', 'BARCODE_SCAN', 'BARCODE_CMD',
  'HUMP_TRAVERSE', 'OBSTACLE_SCAN', 'OBSTACLE_BYPASS', 'LINE_REACQUIRE', 'FAULT_RECOVERY'];
export const MOTOR_STATES = ['STOPPED', 'DRIVING', 'TURNING', 'BYPASS', 'ENCODER_FAULT'];
export const LINE_MODES = ['TRACK', 'COAST', 'SWEEP'];
export const SCAN_STATES = ['MONITORING', 'OBSTACLE_DETECTED', 'MOVE_SERVO', 'MEASURE', 'EVALUATE', 'POST_RESULT', 'SCAN_FAULT'];
export const SCAN_SIDES = ['none yet', 'go LEFT', 'go RIGHT', 'no way round'];
export const BC_EXITS = ['none yet', 'decoded', 'failed (forward and reverse)', 'distance limit',
  'time limit', 'back-up / reverse never finished', 'stopped (button / fault)', 'decoded on the reverse pass'];
export const BC_RESULTS = ['OK', 'NULL', 'COUNT (not 30 edges)', 'WIDTH (no clear narrow/wide)', 'FRAME (start/stop not *)',
  'GAP (character gap wrong)', 'SPEED (speed changed)', 'COMMAND (not a letter)', 'REVERSED (refused)',
  'SHORT (under 6 edges, ignored)', '', '', '', '', '', 'none yet'];
export const BYP_RESULTS = ['none yet', 'stopped on the line', 'ran full length, line not seen', 'curved back on odometry'];
export const FAULTS = ['IMU', 'ENCODER', 'LINE', 'SCAN', 'MOTOR_STALL', 'WIFI', 'HUMP_RANGE', 'MISSION_TMO'];

const bitsFor = maximum => (maximum === 0 ? 0 : 32 - Math.clz32(maximum));

class BitReader {
  constructor(bytes) { this.bytes = bytes; this.cursor = 0; }
  get(maximum) {
    const bits = bitsFor(maximum);
    if (this.cursor + bits > this.bytes.length * 8) throw new Error('frame too short');
    let value = 0;
    for (let i = 0; i < bits; i += 1) {
      const position = this.cursor + i;
      value |= ((this.bytes[position >> 3] >> (position & 7)) & 1) << i;
    }
    this.cursor += bits;
    if (value > maximum) throw new Error('field out of range');
    return value >>> 0;
  }
}

export const charName = code => (code < 26 ? String.fromCharCode(65 + code) : null);
const stateName = (list, value) => list[value] ?? `#${value}`;

export function decodeTelemetry(bytes) {
  if (bytes.length !== TELEM_BYTES) throw new Error(`telemetry must be ${TELEM_BYTES} bytes, got ${bytes.length}`);
  const r = new BitReader(bytes);
  const seq = r.get(255);
  const state = r.get(15);
  const speed = r.get(511) - 256;
  const distance = r.get(65535);
  const heading = r.get(359);
  const lineSeen = r.get(1) === 1;
  const obstacleCm = r.get(255);
  const lastChar = r.get(31);
  const storedChar = r.get(31);
  const hump = r.get(127);
  const lifted = r.get(1) === 1;	// was shock (IMU collision, dropped)
  const faults = r.get(255);
  return {
    seq, state, stateName: stateName(SYS_STATES, state), speedMmps: speed, distanceMm: distance,
    headingDeg: heading, lineSeen, obstacleCm: obstacleCm === 0 ? null : obstacleCm,
    lastChar: charName(lastChar), storedChar: charName(storedChar), humpMaxMm: hump, lifted,
    faults, faultNames: FAULTS.filter((_, i) => faults & (1 << i)),
  };
}

export function decodeDebug(bytes) {
  if (bytes.length !== DEBUG_BYTES) throw new Error(`debug must be ${DEBUG_BYTES} bytes, got ${bytes.length}`);
  const r = new BitReader(bytes);
  const seq = r.get(255);
  const darkC = r.get(1023);
  const darkR = r.get(1023);
  const err = r.get(1023) - 512;
  const steer = r.get(2047) - 1024;
  const mode = r.get(3);
  const bcEdges = r.get(255);
  const bcOk = r.get(255);
  const bcFail = r.get(255);
  const bcLast = r.get(31);
  const ultraMm = r.get(4095);
  const motor = r.get(7);
  const bcStatus = r.get(15);
  const bcCount = r.get(63);
  const scanState = r.get(7);
  // far left, near left, centre, near right, far right
  const scanMm = [r.get(4095), r.get(4095), r.get(4095), r.get(4095), r.get(4095)].map(v => (v === 0 ? null : v));
  const scanSide = r.get(3);
  const bcExit = r.get(7);
  const echoRiseUs = r.get(4095);
  const echoEdges = r.get(15);
  const bypResult = r.get(3);
  return {
    echoRiseUs: echoRiseUs === 0 ? null : echoRiseUs, echoEdges,
    bypResult, bypResultName: BYP_RESULTS[bypResult],
    bcExit, bcExitName: stateName(BC_EXITS, bcExit),
    scanState, scanStateName: stateName(SCAN_STATES, scanState), scanMm,
    scanSide, scanSideName: SCAN_SIDES[scanSide],
    bcStatus, bcResult: stateName(BC_RESULTS, bcStatus), bcCount,
    seq, darkC, darkR, err, steer, mode, modeName: stateName(LINE_MODES, mode),
    bcEdges, bcOk, bcFail, bcLast: charName(bcLast), ultraMm: ultraMm === 0 ? null : ultraMm,
    motor, motorName: stateName(MOTOR_STATES, motor),
  };
}

export function decodeHeartbeat(bytes) {
  if (bytes.length !== 1) throw new Error('heartbeat must be 1 byte');
  return { state: bytes[0], stateName: stateName(SYS_STATES, bytes[0]) };
}

// Barcode trace: seq, status, edge count n, then n-1 widths (u16 LE, 0.1 ms),
// bar first.  Sent once per finished capture.
export function decodeBcTrace(bytes) {
  if (bytes.length < 3) throw new Error('barcode trace needs at least 3 bytes');
  const count = bytes[2];
  const widths = Math.max(count - 1, 0);
  if (bytes.length !== 3 + 2 * widths) throw new Error('barcode trace length does not match its edge count');
  const widthsMs = [];
  for (let i = 0; i < widths; i++) widthsMs.push((bytes[3 + 2 * i] | (bytes[4 + 2 * i] << 8)) / 10);
  return { seq: bytes[0], status: bytes[1], result: stateName(BC_RESULTS, bytes[1]), count, widthsMs };
}

// Frames lost between two sequence numbers (both wrap at 256)
export const seqGap = (previous, current) => (previous === null ? 0 : (current - previous - 1 + 256) % 256);
