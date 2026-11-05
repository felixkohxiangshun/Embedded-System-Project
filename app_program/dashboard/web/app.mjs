import { TX_TOPIC, RX_TOPIC, EXPECTED, isFixture, hexBytes, meanInterval, picoStatus } from './model.mjs';

const $ = id => document.getElementById(id);
let client = null;
let generation = 0;
let connected = false;
let subscribed = false;
let lastReceived = null;
let lastWallTime = null;
let count = 0;
let rejected = 0;
let intervals = [];
let replyPending = false;
let events = [];
let connectionState = 'connecting';

function resetStream() {
  lastReceived = null;
  lastWallTime = null;
  intervals = [];
  const emptyBytes = EXPECTED.map(() => {
    const item = document.createElement('span');
    item.textContent = '—';
    return item;
  });
  $('hex-bytes').replaceChildren(...emptyBytes);
  $('hex-bytes').classList.remove('received');
  $('payload-validation').textContent = 'Expected fixture: 5 bytes';
  renderChart();
}

function event(message, kind = '') {
  events.unshift({ time: new Date().toLocaleTimeString('en-GB'), message, kind });
  events = events.slice(0, 35);
  const rows = events.map(entry => {
    const row = document.createElement('li');
    row.className = entry.kind;
    const time = document.createElement('time');
    time.textContent = entry.time;
    const text = document.createElement('span');
    text.textContent = entry.message;
    row.append(time, text);
    return row;
  });
  $('event-list').replaceChildren(...rows);
}

function render() {
  const now = performance.now();
  const state = picoStatus(connected && subscribed, lastReceived, now);
  const isLive = state === 'live';
  const brokerText = connected && subscribed ? 'Connected' : connected ? 'Subscribing' : connectionState === 'offline' ? 'Disconnected' : connectionState === 'retrying' ? 'Reconnecting' : 'Connecting';
  $('broker-state').replaceChildren(document.createTextNode(brokerText));
  const dot = document.createElement('span');
  dot.className = 'state-dot' + (connected && subscribed ? ' live' : '');
  $('broker-state').append(dot);
  $('broker-detail').textContent = connected && subscribed ? 'WebSocket connected · subscription ready' : connected ? 'Waiting for subscription acknowledgement' : connectionState === 'offline' ? 'Select Reconnect to open the connection' : 'Opening local WebSocket connection';
  const picoText = { live: 'Receiving', stale: 'No recent data', waiting: 'Waiting', offline: 'Offline' }[state];
  $('pico-state').replaceChildren(document.createTextNode(picoText));
  const picoDot = document.createElement('span');
  picoDot.className = 'state-dot' + (isLive ? ' live' : '');
  $('pico-state').append(picoDot);
  $('pico-detail').textContent = isLive ? 'Fresh binary test messages received' : state === 'stale' ? 'No valid test message in the last 3 seconds' : 'Waiting for a fresh test message';
  const overall = !connected || !subscribed ? brokerText : isLive ? 'Live connection' : 'Waiting for Pico';
  $('overall-badge').className = 'badge' + (isLive ? ' live' : connected ? ' warn' : '');
  $('overall-badge').replaceChildren(document.createElement('i'), document.createTextNode(overall));
  $('stream-label').textContent = isLive ? 'Receiving live' : state === 'stale' ? 'Stream paused' : 'Waiting for data';
  $('stream-label').className = 'pill' + (isLive ? ' live' : state === 'stale' ? ' warn' : '');
  $('packet-count').textContent = count.toLocaleString();
  const average = meanInterval(intervals);
  $('interval-value').textContent = average === null ? '—' : `${Math.round(average)} ms`;
  const age = lastReceived === null ? null : Math.max(0, now - lastReceived);
  $('last-seen').textContent = age === null ? 'Last received: —' : `Last received: ${(age / 1000).toFixed(1)} s ago`;
  $('last-time').textContent = lastWallTime === null ? 'No message yet' : lastWallTime;
  $('send-reply').disabled = !connected || !subscribed || replyPending;
  $('disconnect-button').disabled = client === null;
  $('pico-node').classList.toggle('live', isLive);
  $('broker-node').classList.toggle('live', connected);
  $('browser-node').classList.toggle('live', connected && subscribed);
  $('clock').textContent = new Date().toLocaleString('en-SG', { day: '2-digit', month: 'short', hour: '2-digit', minute: '2-digit', hour12: false });
}

function renderChart() {
  const bars = intervals.slice(-40).map(interval => {
    const bar = document.createElement('div');
    bar.className = 'arrival-bar' + (interval > 1000 ? ' delayed' : '');
    bar.style.height = `${Math.max(5, Math.min(100, interval / 1000 * 100))}%`;
    bar.title = `${Math.round(interval)} ms between browser arrivals`;
    return bar;
  });
  if (bars.length) $('arrival-chart').replaceChildren(...bars);
  else {
    const empty = document.createElement('span');
    empty.className = 'empty-chart';
    empty.textContent = 'Fresh messages will appear here';
    $('arrival-chart').replaceChildren(empty);
  }
}

function connect() {
  const address = $('broker-url').value.trim();
  try {
    const parsed = new URL(address);
    if (!['ws:', 'wss:'].includes(parsed.protocol) || parsed.username || parsed.password) throw new Error();
  } catch {
    $('connection-error').textContent = 'Enter a WebSocket address, such as ws://127.0.0.1:9001/mqtt.';
    return;
  }
  if (!window.mqtt) {
    $('connection-error').textContent = 'The local MQTT library could not load. Restart the page server and reload.';
    connectionState = 'offline';
    render();
    return;
  }
  $('connection-error').textContent = '';
  const current = ++generation;
  if (client) client.end(true);
  connected = false;
  subscribed = false;
  resetStream();
  replyPending = false;
  connectionState = 'connecting';
  $('reply-status').textContent = 'Connect to the broker to send a reply.';
  $('reply-status').className = 'reply-status';
  event(`Connecting to ${address}`);
  try {
    client = window.mqtt.connect(address, {
      clientId: `csc11-web-${crypto.randomUUID().slice(0, 8)}`,
      protocolVersion: 4, clean: true, keepalive: 15,
      reconnectPeriod: 1500, connectTimeout: 5000,
      resubscribe: false, queueQoSZero: false,
    });
  } catch (error) {
    client = null;
    connectionState = 'offline';
    $('connection-error').textContent = error.message;
    event('Could not open the connection.', 'warning');
    render();
    return;
  }
  const active = client;
  active.on('connect', () => {
    if (current !== generation) return;
    connected = true;
    subscribed = false;
    $('connection-error').textContent = '';
    resetStream();
    event('Broker connected over WebSockets.', 'success');
    active.subscribe(TX_TOPIC, { qos: 0 }, (error, grants) => {
      if (current !== generation || !active.connected) return;
      if (error || !grants?.length || grants.some(grant => grant.qos > 2)) {
        $('connection-error').textContent = 'The broker rejected the test-topic subscription. Check its development-topic access.';
        event('Subscription failed.', 'warning');
      } else {
        subscribed = true;
        $('reply-status').textContent = 'Ready to send a binary test reply.';
        event('Subscribed to the Pico’s test messages.', 'success');
      }
      render();
    });
    render();
  });
  active.on('message', (topic, payload, packet) => {
    if (current !== generation || topic !== TX_TOPIC) return;
    if (packet.retain) {
      event('Ignored a retained test message; waiting for fresh data.', 'warning');
      return;
    }
    if (!isFixture(payload)) {
      rejected++;
      event(`Unexpected test payload ignored (${payload.length} bytes, ${rejected} rejected).`, 'warning');
      return;
    }
    const now = performance.now();
    if (lastReceived !== null) intervals.push(now - lastReceived);
    intervals = intervals.slice(-40);
    lastReceived = now;
    lastWallTime = new Date().toLocaleTimeString('en-GB');
    count++;
    const bytes = hexBytes(payload).map(byte => {
      const item = document.createElement('span');
      item.textContent = byte;
      return item;
    });
    $('hex-bytes').replaceChildren(...bytes);
    $('hex-bytes').classList.add('received');
    $('payload-validation').textContent = 'Valid fixture: 5 bytes';
    if (count === 1 || count % 20 === 0) event(`Received ${count} valid binary test ${count === 1 ? 'message' : 'messages'}.`, 'success');
    renderChart();
    render();
  });
  active.on('reconnect', () => {
    if (current !== generation) return;
    connectionState = 'retrying';
    render();
  });
  active.on('close', () => {
    if (current !== generation) return;
    const wasConnected = connected;
    connected = false;
    subscribed = false;
    replyPending = false;
    connectionState = 'retrying';
    $('reply-status').textContent = 'Broker disconnected. Waiting to reconnect.';
    $('reply-status').className = 'reply-status';
    if (wasConnected) event('Broker connection closed. Reconnecting automatically.', 'warning');
    render();
  });
  active.on('error', error => {
    if (current !== generation) return;
    $('connection-error').textContent = `Connection error: ${error.message}. Check that the local broker is running on port 9001.`;
    event('Broker connection error.', 'warning');
  });
  render();
}

$('connection-form').addEventListener('submit', event => { event.preventDefault(); connect(); });
$('disconnect-button').addEventListener('click', () => {
  generation++;
  if (client) client.end(true);
  client = null;
  connected = false;
  subscribed = false;
  replyPending = false;
  connectionState = 'offline';
  $('reply-status').textContent = 'Connect to the broker to send a reply.';
  $('reply-status').className = 'reply-status';
  event('Disconnected by you.');
  render();
});
$('send-reply').addEventListener('click', () => {
  if (!client || !connected || !subscribed || replyPending) return;
  const current = generation;
  replyPending = true;
  $('reply-status').textContent = 'Sending five binary bytes…';
  $('reply-status').className = 'reply-status';
  client.publish(RX_TOPIC, new Uint8Array(EXPECTED), { qos: 1, retain: false }, error => {
    if (current !== generation) return;
    replyPending = false;
    $('reply-status').textContent = error ? `Reply failed: ${error.message}` : 'Broker acknowledged. Check the Pico console for RX PASS.';
    $('reply-status').className = 'reply-status' + (error ? '' : ' success');
    event(error ? 'Test reply failed.' : 'Binary reply acknowledged by broker (QoS 1).', error ? 'warning' : 'success');
    render();
  });
  render();
});
$('clear-log').addEventListener('click', () => {
  events = [];
  event('Activity log cleared.');
});

setInterval(render, 250);
connect();
