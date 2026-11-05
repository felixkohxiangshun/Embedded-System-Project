export const TX_TOPIC = 'csc11/dev/pico/tx';
export const RX_TOPIC = 'csc11/dev/pico/rx';
export const EXPECTED = Object.freeze([0x8d, 0x23, 0x01, 0x00, 0xff]);
export const STALE_AFTER_MS = 3000;

export function isFixture(bytes) {
  return bytes.length === EXPECTED.length && EXPECTED.every((value, i) => bytes[i] === value);
}

export function hexBytes(bytes) {
  return Array.from(bytes, byte => byte.toString(16).padStart(2, '0').toUpperCase());
}

export function meanInterval(intervals) {
  const valid = intervals.filter(value => Number.isFinite(value) && value >= 0);
  return valid.length ? valid.reduce((sum, value) => sum + value, 0) / valid.length : null;
}

export function picoStatus(connected, lastReceived, now) {
  if (!connected) return 'offline';
  if (lastReceived === null) return 'waiting';
  return now - lastReceived <= STALE_AFTER_MS ? 'live' : 'stale';
}
