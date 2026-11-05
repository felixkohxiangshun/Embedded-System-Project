import test from 'node:test';
import assert from 'node:assert/strict';
import { EXPECTED, isFixture, hexBytes, meanInterval, picoStatus } from './model.mjs';

test('binary fixture is exact, including NUL and non-ASCII bytes', () => {
  assert.equal(isFixture(new Uint8Array(EXPECTED)), true);
  assert.equal(isFixture(new TextEncoder().encode('8d230100ff')), false);
  assert.equal(isFixture(new Uint8Array([0x8d, 0x23])), false);
  assert.equal(isFixture(new Uint8Array([...EXPECTED, 0])), false);
  assert.equal(isFixture(new Uint8Array([0x8d, 0x23, 1, 0, 0])), false);
  assert.deepEqual(hexBytes(new Uint8Array(EXPECTED)), ['8D', '23', '01', '00', 'FF']);
});
test('broker connection alone does not imply a live Pico', () => {
  assert.equal(picoStatus(false, 100, 200), 'offline');
  assert.equal(picoStatus(true, null, 200), 'waiting');
  assert.equal(picoStatus(true, 100, 3100), 'live');
  assert.equal(picoStatus(true, 100, 3101), 'stale');
});
test('arrival interval is unavailable until measured', () => {
  assert.equal(meanInterval([]), null);
  assert.equal(meanInterval([490, 510, 500]), 500);
  assert.equal(meanInterval([NaN, Infinity, -1]), null);
});
