// Decoders for the car's binary frames (TEAM.md section 7).
// Must match dash_pack_telem() / dash_pack_debug() in dash_logic.c:
// protocol_bits packs each field into the bits its maximum needs, in order,
// least significant bit first, starting at bit 0 of byte 0.

export const TOPICS = Object.freeze({
  telemetry: 'csc11/dev/car/telemetry',
  debug: 'csc11/dev/car/debug',
  heartbeat: 'csc11/dev/car/heartbeat',
  command: 'csc11/dev/car/cmd',
});
export const TELEM_BYTES = 11;
export const DEBUG_BYTES = 12;
export const CHAR_NONE = 31;

export const SYS_STATES = ['INIT', 'IDLE', 'LINE_FOLLOW', 'BARCODE_SCAN', 'BARCODE_CMD',
  'HUMP_TRAVERSE', 'OBSTACLE_SCAN', 'OBSTACLE_BYPASS', 'LINE_REACQUIRE', 'FAULT_RECOVERY', 'E_STOP'];
export const MOTOR_STATES = ['STOPPED', 'DRIVING', 'TURNING', 'BYPASS', 'ENCODER_FAULT'];
export const LINE_MODES = ['TRACK', 'COAST', 'SWEEP'];
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
  const shock = r.get(1) === 1;
  const faults = r.get(255);
  return {
    seq, state, stateName: stateName(SYS_STATES, state), speedMmps: speed, distanceMm: distance,
    headingDeg: heading, lineSeen, obstacleCm: obstacleCm === 0 ? null : obstacleCm,
    lastChar: charName(lastChar), storedChar: charName(storedChar), humpMaxMm: hump, shock,
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
  return {
    seq, darkC, darkR, err, steer, mode, modeName: stateName(LINE_MODES, mode),
    bcEdges, bcOk, bcFail, bcLast: charName(bcLast), ultraMm: ultraMm === 0 ? null : ultraMm,
    motor, motorName: stateName(MOTOR_STATES, motor),
  };
}

export function decodeHeartbeat(bytes) {
  if (bytes.length !== 1) throw new Error('heartbeat must be 1 byte');
  return { state: bytes[0], stateName: stateName(SYS_STATES, bytes[0]) };
}

// Frames lost between two sequence numbers (both wrap at 256)
export const seqGap = (previous, current) => (previous === null ? 0 : (current - previous - 1 + 256) % 256);
