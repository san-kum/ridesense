const WS_URL = 'ws://localhost:9001';

let ws = null;
let reconnectTimer = null;

const $ = id => document.getElementById(id);

const statusDot = $('status-dot');
const statusText = $('status-text');
const speedValue = $('speed-value');
const leanValue = $('lean-value');
const leanIndicator = $('lean-indicator');
const latValue = $('lat-value');
const lonValue = $('lon-value');
const headingValue = $('heading-value');
const eventLog = $('event-log');
const maxSpeedValue = $('max-speed');
const avgSpeedValue = $('avg-speed');
const hardBrakesValue = $('hard-brakes');
const leanWarningsValue = $('lean-warnings');
const heartbeatTime = $('heartbeat-time');

const LEAN_MAX = 60;

function connect() {
    if (ws && ws.readyState === WebSocket.OPEN) return;

    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        setConnected(true);
        clearTimeout(reconnectTimer);
    };

    ws.onclose = () => {
        setConnected(false);
        scheduleReconnect();
    };

    ws.onerror = () => { };

    ws.onmessage = (event) => {
        try {
            handleMessage(JSON.parse(event.data));
        } catch (e) { }
    };
}

function setConnected(connected) {
    statusDot.classList.toggle('connected', connected);
    statusText.textContent = connected ? 'live' : 'offline';
}

function scheduleReconnect() {
    clearTimeout(reconnectTimer);
    reconnectTimer = setTimeout(connect, 2000);
}

function handleMessage(msg) {
    switch (msg.type) {
        case 'heartbeat':
            handleHeartbeat(msg);
            break;
        case 'state_update':
            handleStateUpdate(msg);
            break;
        case 'event':
            handleEvent(msg);
            break;
        case 'metrics':
            handleMetrics(msg);
            break;
    }
}

function handleHeartbeat(msg) {
    const date = new Date(msg.ts);
    heartbeatTime.textContent = date.toLocaleTimeString('en-US', {
        hour12: false,
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    });
}

function handleStateUpdate(msg) {
    const speed = msg.speed || 0;
    speedValue.textContent = Math.round(speed);

    const lean = msg.lean || 0;
    const sign = lean >= 0 ? '+' : '';
    leanValue.textContent = `${sign}${lean.toFixed(1)}°`;

    const leanPercent = (lean + LEAN_MAX) / (2 * LEAN_MAX);
    leanIndicator.style.left = `${Math.max(0, Math.min(100, leanPercent * 100))}%`;

    latValue.textContent = (msg.lat || 0).toFixed(6);
    lonValue.textContent = (msg.lon || 0).toFixed(6);
    headingValue.textContent = `${Math.round(msg.heading || 0)}°`;
}

function handleEvent(msg) {
    const item = document.createElement('div');
    item.className = 'event-item';

    const time = new Date(msg.ts).toLocaleTimeString('en-US', {
        hour12: false,
        hour: '2-digit',
        minute: '2-digit',
        second: '2-digit'
    });

    const eventName = msg.event.replace(/_/g, ' ');
    const value = typeof msg.value === 'number' ? msg.value.toFixed(1) : msg.value;

    item.innerHTML = `
        <span class="event-time">${time}</span>
        <span class="event-type">${eventName}</span>
        <span class="event-value">${value}</span>
    `;

    eventLog.insertBefore(item, eventLog.firstChild);

    while (eventLog.children.length > 20) {
        eventLog.removeChild(eventLog.lastChild);
    }
}

function handleMetrics(msg) {
    maxSpeedValue.textContent = Math.round(msg.max_speed || 0);
    avgSpeedValue.textContent = Math.round(msg.avg_speed || 0);
    hardBrakesValue.textContent = msg.hard_brakes || 0;
    leanWarningsValue.textContent = msg.lean_warnings || 0;
}

document.addEventListener('DOMContentLoaded', connect);
