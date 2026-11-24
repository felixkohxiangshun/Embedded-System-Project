import { TOPICS, encodeCommand, driveButtons, decodeTelemetry, decodeDebug, decodeHeartbeat, decodeBcTrace, decodeImu, encodeMap, decodeMap, MAP_TURNS } from './car_model.mjs';

const $ = id => document.getElementById(id);
const STALE_MS = 2000;
let client = null;
let lastTelemAt = 0;
let lastDebugAt = 0;
let telemTimes = [];
let carState = null;	// last SYS_xxx name from telemetry or heartbeat
let previous = { state: null, faults: 0, bcOk: null, bcFail: null, stored: null, scanState: null, scanSide: null, bcExit: null, bypResult: null, widthMm: null, imuState: null, humps: null };

function event(text) {
  const item = document.createElement('li');
  const time = document.createElement('time');
  time.textContent = new Date().toLocaleTimeString('en-GB');
  item.append(time, text);
  $('events').prepend(item);
  while ($('events').children.length > 200) $('events').lastChild.remove();
}

function rate(times) {
  if (times.length < 2) return '—';
  const span = times[times.length - 1] - times[0];
  return `${Math.round(span / (times.length - 1))} ms`;
}

function remember(times, now) {
  times.push(now);
  while (times.length > 20) times.shift();
}

function showTelemetry(t) {
  const now = performance.now();
  lastTelemAt = now;
  remember(telemTimes, now);
  $('sys-state').textContent = t.stateName;
  carState = t.stateName;
  $('speed').textContent = `${t.speedMmps} mm/s`;
  $('distance').textContent = `${(t.distanceMm / 1000).toFixed(3)} m`;
  $('heading').textContent = `${t.headingDeg}°`;
  $('line-seen').textContent = t.lineSeen ? 'yes' : 'no';
  $('obstacle').textContent = t.obstacleCm === null ? 'none' : `${t.obstacleCm} cm`;
  $('hump').textContent = `${t.humpMaxMm} mm`;
  $('lifted').textContent = t.lifted ? 'yes: motors stopped' : 'no';
  $('faults').textContent = t.faultNames.length ? t.faultNames.join(', ') : 'none';
  $('last-char').textContent = t.lastChar ?? '—';
  $('stored-char').textContent = t.storedChar ?? '—';
  if (previous.state !== null && previous.state !== t.stateName) event(`state ${previous.state} → ${t.stateName}`);
  if (t.faults !== previous.faults && t.faults) event(`faults: ${t.faultNames.join(', ')}`);
  if (previous.stored !== null && t.storedChar !== previous.stored && t.storedChar) event(`stored character ${t.storedChar}`);
  previous.state = t.stateName;
  previous.faults = t.faults;
  previous.stored = t.storedChar;
}

// Widths of the last capture, bars and spaces on separate rows (ms)
function showBcTrace(t) {
  const bars = t.widthsMs.filter((_, i) => i % 2 === 0).map(w => w.toFixed(1));
  const spaces = t.widthsMs.filter((_, i) => i % 2 === 1).map(w => w.toFixed(1));
  $('bc-trace').textContent = `${t.count} edges, ${t.result}\nbars   ${bars.join(' ')}\nspaces ${spaces.join(' ')}`;
}

function showDebug(g) {
  const now = performance.now();
  lastDebugAt = now;
  $('dark-c').textContent = g.darkC;
  $('dark-r').textContent = g.darkR;
  $('bar-c').style.width = `${Math.min(100, g.darkC / 10)}%`;
  $('bar-r').style.width = `${Math.min(100, g.darkR / 10)}%`;
  $('steer').textContent = g.steer;
  $('steer-mark').style.left = `${50 + Math.max(-50, Math.min(50, g.steer / 20))}%`;
  $('err').textContent = g.err;
  $('mode').textContent = g.modeName;
  $('motor').textContent = g.motorName;
  $('scan-ultra').textContent = g.ultraMm === null ? 'no echo' : `${g.ultraMm} mm`;
  $('scan-state').textContent = g.scanStateName;
  const w = g.width;
  $('obs-width').textContent = w === null ? 'not measured' : `${w.mm} mm`;
  $('obs-halves').textContent = w === null ? '—' : `${w.leftMm} / ${w.rightMm} mm`;
  if (w !== null && (previous.widthMm === null || w.mm !== previous.widthMm)) event(`obstacle width ${$('obs-width').textContent} (left ${w.leftMm}, right ${w.rightMm})`);
  if (w !== null) $('last-width').textContent = `${w.mm} mm`;
  previous.widthMm = w === null ? null : w.mm;
  $('scan-side').textContent = g.scanSideName;
  $('byp-result').textContent = g.bypResultName;
  if (previous.bypResult !== null && g.bypResult !== previous.bypResult && g.bypResult !== 0) event(`bypass return leg: ${g.bypResultName}`);
  previous.bypResult = g.bypResult;
  if (previous.scanState !== null && g.scanState !== previous.scanState) event(`scan: ${g.scanStateName}`);
  if (previous.scanSide !== null && g.scanSide !== previous.scanSide && g.scanSide !== 0) {
    event(`sweep decision: ${g.scanSideName}`);
  }
  previous.scanState = g.scanState;
  previous.scanSide = g.scanSide;
  $('bc-edges').textContent = g.bcEdges;
  $('bc-ok').textContent = g.bcOk;
  $('bc-fail').textContent = g.bcFail;
  $('bc-result').textContent = g.bcResult;
  $('bc-count').textContent = `${g.bcCount} / 30`;
  $('bc-exit').textContent = g.bcExitName;
  if (previous.bcExit !== null && g.bcExit !== previous.bcExit && g.bcExit !== 0) event(`barcode pass ended: ${g.bcExitName}`);
  previous.bcExit = g.bcExit;
  if (previous.bcOk !== null && g.bcOk !== previous.bcOk) event(`barcode read: ${g.bcLast ?? '?'}`);
  if (previous.bcFail !== null && g.bcFail !== previous.bcFail) event(`barcode failed: ${g.bcResult}, ${g.bcCount} edges`);
  previous.bcOk = g.bcOk;
  previous.bcFail = g.bcFail;
}

function showImu(m) {
  $('imu-state').textContent = m.stateName;
  $('imu-cal').textContent = m.calibrated ? 'yes' : 'no: keep the car still';
  $('imu-lean').textContent = `${m.leanMm.toFixed(1)} mm`;
  $('imu-last').textContent = m.humps ? `${m.lastHumpMm} mm` : 'none yet';
  $('imu-max').textContent = m.humps ? `${m.maxHumpMm} mm` : 'none yet';
  $('imu-humps').textContent = m.humps;
  $('imu-rise').textContent = `${m.riseMps.toFixed(2)} m/s (${Math.max(m.risePercent, 0)} %)`;
  $('imu-rise').title = 'Upward speed from the accelerometer; the car counts as lifted at 100 % (about 0.39 m/s)';
  if (previous.imuState !== null && m.stateName !== previous.imuState &&
      (m.stateName === 'LIFTED' || m.stateName === 'SENSOR_FAULT' || previous.imuState === 'SENSOR_FAULT')) {
    event(`IMU ${previous.imuState} → ${m.stateName}`);
  }
  if (previous.humps !== null && m.humps !== previous.humps) event(`hump measured: ${m.lastHumpMm} mm (tallest ${m.maxHumpMm} mm)`);
  previous.imuState = m.stateName;
  previous.humps = m.humps;
}

function tick() {
  const now = performance.now();
  $('clock').textContent = new Date().toLocaleTimeString('en-GB');
  $('telem-rate').textContent = rate(telemTimes);
  const live = now - Math.max(lastTelemAt, lastDebugAt) < STALE_MS;
  const badge = $('overall');
  badge.classList.toggle('live', live);
  badge.lastChild.textContent = client?.connected ? (live ? 'Car live' : 'Waiting for car') : 'Broker offline';
  $('sys-state').classList.toggle('stale', !live);
  const drive = driveButtons(carState, live, client?.connected);
  $('btn-start').disabled = !drive.start;
  $('btn-stop').disabled = !drive.stop;
  requestAnimationFrame(tick);
}

function connect() {
  if (client) client.end(true);
  const address = $('broker').value.trim();
  $('broker-state').textContent = `Connecting to ${address}…`;
  client = window.mqtt.connect(address, {
    clientId: `csc11-car-web-${crypto.randomUUID().slice(0, 8)}`,
    protocolVersion: 4, clean: true, keepalive: 15, reconnectPeriod: 1500, connectTimeout: 5000,
  });
  client.on('connect', () => {
    $('broker-state').textContent = 'Broker connected';
    client.subscribe([TOPICS.telemetry, TOPICS.debug, TOPICS.heartbeat, TOPICS.barcode, TOPICS.imu, TOPICS.map], { qos: 0 });
    event('broker connected');
  });
  client.on('close', () => { $('broker-state').textContent = 'Broker disconnected, retrying…'; });
  client.on('error', error => { $('broker-state').textContent = `Broker error: ${error.message}`; });
  client.on('message', (topic, payload) => {
    const bytes = new Uint8Array(payload);
    try {
      if (topic === TOPICS.telemetry) showTelemetry(decodeTelemetry(bytes));
      else if (topic === TOPICS.debug) showDebug(decodeDebug(bytes));
      else if (topic === TOPICS.barcode) showBcTrace(decodeBcTrace(bytes));
      else if (topic === TOPICS.imu) showImu(decodeImu(bytes));
      else if (topic === TOPICS.map) showMap(decodeMap(bytes));
      else if (topic === TOPICS.heartbeat) {
        const hb = decodeHeartbeat(bytes);
        lastTelemAt = performance.now();
        $('sys-state').textContent = hb.stateName;
        carState = hb.stateName;
      }
    } catch (error) {
      event(`rejected ${topic}: ${error.message}`);
    }
  });
}

// QoS 1: the car subscribes to the command topic at QoS 1, so the broker
// redelivers a command lost on the WiFi link.
function sendCommand(name) {
  if (!client?.connected) return;
  const label = name.toUpperCase();
  client.publish(TOPICS.command, encodeCommand(name), { qos: 1 }, error => {
    event(error ? `${label} not sent: ${error.message}` : `${label} sent to the car`);
  });
}

// The mapping held on the broker (retained), i.e. what the car gets
function showMap(map) {
  MAP_TURNS.forEach(turn => { $(`map-${turn}`).value = map[turn]; });
  const text = MAP_TURNS.map(turn => `${turn === 'uturn' ? 'U-turn' : turn} ${map[turn] || '-'}`).join(', ');
  $('map-state').textContent = `On the broker: ${text}`;
}

function sendMap(e) {
  e.preventDefault();
  if (!client?.connected) return;
  let payload;
  const map = {};
  MAP_TURNS.forEach(turn => { map[turn] = $(`map-${turn}`).value; });
  try {
    payload = encodeMap(map);
  } catch (error) {
    $('map-state').textContent = `Not sent: ${error.message}`;
    return;
  }
  // QoS 1 and retained: the car gets it now, and again whenever it reconnects
  client.publish(TOPICS.map, payload, { qos: 1, retain: true }, error => {
    event(error ? `mapping not sent: ${error.message}` : `junction mapping sent: ${$('map-state').textContent}`);
  });
}

$('map-form').addEventListener('submit', sendMap);
$('connect').addEventListener('click', connect);
$('btn-start').addEventListener('click', () => sendCommand('start'));
$('btn-stop').addEventListener('click', () => sendCommand('stop'));
window.addEventListener('load', () => { connect(); requestAnimationFrame(tick); });
