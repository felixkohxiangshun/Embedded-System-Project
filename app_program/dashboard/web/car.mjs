import { TOPICS, decodeTelemetry, decodeDebug, decodeHeartbeat, seqGap } from './car_model.mjs';

const $ = id => document.getElementById(id);
const STALE_MS = 2000;
const TRACE_MS = 15000;
let client = null;
let lastTelemSeq = null;
let lastDebugSeq = null;
let lost = 0;
let lastTelemAt = 0;
let lastDebugAt = 0;
let telemTimes = [];
let debugTimes = [];
let trace = [];
let previous = { state: null, faults: 0, bcOk: null, bcFail: null, stored: null };

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
  lost += seqGap(lastTelemSeq, t.seq);
  lastTelemSeq = t.seq;
  lastTelemAt = now;
  remember(telemTimes, now);
  $('sys-state').textContent = t.stateName;
  $('speed').textContent = `${t.speedMmps} mm/s`;
  $('distance').textContent = `${(t.distanceMm / 1000).toFixed(3)} m`;
  $('heading').textContent = `${t.headingDeg}°`;
  $('line-seen').textContent = t.lineSeen ? 'yes' : 'no';
  $('obstacle').textContent = t.obstacleCm === null ? 'none' : `${t.obstacleCm} cm`;
  $('hump').textContent = `${t.humpMaxMm} mm`;
  $('shock').textContent = t.shock ? 'yes' : 'no';
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

function showDebug(g) {
  const now = performance.now();
  lost += seqGap(lastDebugSeq, g.seq);
  lastDebugSeq = g.seq;
  lastDebugAt = now;
  remember(debugTimes, now);
  $('dark-c').textContent = g.darkC;
  $('dark-r').textContent = g.darkR;
  $('bar-c').style.width = `${Math.min(100, g.darkC / 10)}%`;
  $('bar-r').style.width = `${Math.min(100, g.darkR / 10)}%`;
  $('steer').textContent = g.steer;
  $('steer-mark').style.left = `${50 + Math.max(-50, Math.min(50, g.steer / 20))}%`;
  $('err').textContent = g.err;
  $('mode').textContent = g.modeName;
  $('motor').textContent = g.motorName;
  $('ultra').textContent = g.ultraMm === null ? 'no echo' : `${g.ultraMm} mm`;
  $('bc-edges').textContent = g.bcEdges;
  $('bc-ok').textContent = g.bcOk;
  $('bc-fail').textContent = g.bcFail;
  if (previous.bcOk !== null && g.bcOk !== previous.bcOk) event(`barcode read: ${g.bcLast ?? '?'}`);
  if (previous.bcFail !== null && g.bcFail !== previous.bcFail) event('barcode failed');
  previous.bcOk = g.bcOk;
  previous.bcFail = g.bcFail;
  trace.push({ t: now, c: g.darkC, r: g.darkR, s: g.steer });
  while (trace.length && trace[0].t < now - TRACE_MS) trace.shift();
}

function drawTrace() {
  const canvas = $('trace');
  const width = canvas.clientWidth;
  const height = canvas.height;
  if (canvas.width !== width) canvas.width = width;
  const ctx = canvas.getContext('2d');
  ctx.clearRect(0, 0, width, height);
  ctx.strokeStyle = '#e1e4e6';
  ctx.beginPath();
  ctx.moveTo(0, height / 2);
  ctx.lineTo(width, height / 2);
  ctx.stroke();
  const now = performance.now();
  const x = t => width - ((now - t) / TRACE_MS) * width;
  const line = (key, scale, offset, colour) => {
    ctx.strokeStyle = colour;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    trace.forEach((p, i) => {
      const y = height - ((p[key] + offset) / scale) * height;
      if (i === 0) ctx.moveTo(x(p.t), y); else ctx.lineTo(x(p.t), y);
    });
    ctx.stroke();
  };
  line('c', 1000, 0, '#2f3b45');
  line('r', 1000, 0, '#3f7cac');
  line('s', 2000, 1000, '#9c3d14');
}

function tick() {
  const now = performance.now();
  $('clock').textContent = new Date().toLocaleTimeString('en-GB');
  $('telem-rate').textContent = rate(telemTimes);
  $('debug-rate').textContent = rate(debugTimes);
  $('lost').textContent = lost;
  const live = now - Math.max(lastTelemAt, lastDebugAt) < STALE_MS;
  const badge = $('overall');
  badge.classList.toggle('live', live);
  badge.lastChild.textContent = client?.connected ? (live ? 'Car live' : 'Waiting for car') : 'Broker offline';
  $('sys-state').classList.toggle('stale', !live);
  drawTrace();
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
    client.subscribe([TOPICS.telemetry, TOPICS.debug, TOPICS.heartbeat], { qos: 0 });
    event('broker connected');
  });
  client.on('close', () => { $('broker-state').textContent = 'Broker disconnected, retrying…'; });
  client.on('error', error => { $('broker-state').textContent = `Broker error: ${error.message}`; });
  client.on('message', (topic, payload) => {
    const bytes = new Uint8Array(payload);
    try {
      if (topic === TOPICS.telemetry) showTelemetry(decodeTelemetry(bytes));
      else if (topic === TOPICS.debug) showDebug(decodeDebug(bytes));
      else if (topic === TOPICS.heartbeat) {
        const hb = decodeHeartbeat(bytes);
        lastTelemAt = performance.now();
        $('sys-state').textContent = hb.stateName;
      }
    } catch (error) {
      event(`rejected ${topic}: ${error.message}`);
    }
  });
}

$('connect').addEventListener('click', connect);
window.addEventListener('load', () => { connect(); requestAnimationFrame(tick); });
