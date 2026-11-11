import test from 'node:test';
import assert from 'node:assert/strict';
import { decodeTelemetry, decodeDebug, decodeHeartbeat, seqGap } from './car_model.mjs';

// Same LSB-first packer as protocol_bits.c, to build test frames.
function pack(fields, bytes) {
  const out = new Uint8Array(bytes);
  let cursor = 0;
  for (const [value, maximum] of fields) {
    const bits = maximum === 0 ? 0 : 32 - Math.clz32(maximum);
    for (let i = 0; i < bits; i += 1) {
      if ((value >> i) & 1) out[(cursor + i) >> 3] |= 1 << ((cursor + i) & 7);
    }
    cursor += bits;
  }
  return out;
}

test('telemetry: every field round-trips, 11 bytes', () => {
  const frame = pack([[7, 255], [2, 15], [120 + 256, 511], [54321, 65535], [359, 359], [1, 1],
    [30, 255], [0, 31], [31, 31], [42, 127], [1, 1], [0x85, 255]], 11);
  const t = decodeTelemetry(frame);
  assert.equal(t.seq, 7);
  assert.equal(t.stateName, 'LINE_FOLLOW');
  assert.equal(t.speedMmps, 120);
  assert.equal(t.distanceMm, 54321);
  assert.equal(t.headingDeg, 359);
  assert.equal(t.lineSeen, true);
  assert.equal(t.obstacleCm, 30);
  assert.equal(t.lastChar, 'A');
  assert.equal(t.storedChar, null);
  assert.equal(t.humpMaxMm, 42);
  assert.equal(t.shock, true);
  assert.deepEqual(t.faultNames, ['IMU', 'LINE', 'MISSION_TMO']);
});

test('telemetry: negative speed and wrong length', () => {
  const frame = pack([[0, 255], [1, 15], [-40 + 256, 511]], 11);
  assert.equal(decodeTelemetry(frame).speedMmps, -40);
  assert.throws(() => decodeTelemetry(new Uint8Array(10)));
});

test('debug: signed fields and counters round-trip, 22 bytes', () => {
  const frame = pack([[200, 255], [600, 1023], [480, 1023], [-120 + 512, 1023], [-700 + 1024, 2047],
    [2, 3], [60, 255], [1, 255], [3, 255], [1, 31], [289, 4095], [1, 7], [6, 15], [30, 63],
    [3, 7], [1200, 4095], [900, 4095], [250, 4095], [700, 4095], [0, 4095], [2, 3], [3, 7]], 22);
  const g = decodeDebug(frame);
  assert.deepEqual([g.darkC, g.darkR, g.err, g.steer], [600, 480, -120, -700]);
  assert.equal(g.modeName, 'SWEEP');
  assert.deepEqual([g.bcEdges, g.bcOk, g.bcFail, g.bcLast], [60, 1, 3, 'B']);
  assert.equal(g.ultraMm, 289);
  assert.equal(g.motorName, 'DRIVING');
  assert.equal(g.bcResult, 'SPEED (speed changed)');
  assert.equal(g.bcCount, 30);
  assert.equal(g.scanStateName, 'MEASURE');
  assert.deepEqual(g.scanMm, [1200, 900, 250, 700, null]);
  assert.equal(g.scanSideName, 'go RIGHT');
  assert.equal(g.bcExitName, 'distance limit');
});

test('heartbeat and sequence gaps', () => {
  assert.equal(decodeHeartbeat(new Uint8Array([1])).stateName, 'IDLE');
  assert.equal(seqGap(null, 5), 0);
  assert.equal(seqGap(254, 1), 2);
  assert.equal(seqGap(9, 10), 0);
});

test('barcode trace: widths in 0.1 ms, bar first', async () => {
  const { decodeBcTrace } = await import('./car_model.mjs');
  const t = decodeBcTrace(Uint8Array.from([5, 2, 3, 0x7b, 0x00, 0x10, 0x27]));
  assert.equal(t.seq, 5);
  assert.equal(t.result, 'COUNT (not 30 edges)');
  assert.deepEqual(t.widthsMs, [12.3, 1000]);
  assert.throws(() => decodeBcTrace(Uint8Array.from([5, 2, 3, 1])));
});
