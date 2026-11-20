/*
 * =====================================================
 * RoboBot - webserver.cpp
 * Web Server Implementation
 * =====================================================
 * ESP32 runs in WiFi Access Point mode.
 * DNS captive portal redirects all domains to 192.168.4.1.
 * HTTP server serves the dashboard HTML and handles JSON API.
 * =====================================================
 */

#include "robot_webserver.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>

// ------------------------------------------------
// SERVER INSTANCES
// ------------------------------------------------
static const byte DNS_PORT = 53;
static DNSServer  dnsServer;
static WebServer  httpServer(80);

// ------------------------------------------------
// SHARED STATE
// ------------------------------------------------
static WebCommand* g_cmd = nullptr;

static struct {
    float       angle;
    float       distance;
    char        expression[24];
    RobotMode   mode;
    bool        obsEnabled;
    bool        buzzEnabled;
    int         motorL;
    int         motorR;
    float       kp;
    float       ki;
    float       kd;
    float       trim;
    float       minSpeed;
} g_telem = {0.0f, -1.0f, "EYE_NORMAL", MODE_BALANCE_SEGWAY, true, true, 0, 0, 24.0f, 0.3f, 1.2f, 0.0f, 28.0f};

// ============================================================
// EMBEDDED HTML DASHBOARD
// Single-page dashboard with zero external CDN dependencies.
// ============================================================
static const char DASHBOARD_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>RoboBot Dashboard</title>
<style>
  * { box-sizing:border-box; margin:0; padding:0; -webkit-tap-highlight-color:transparent; }
  :root {
    --bg:#0a0a0f; --card:#12121a; --border:#1e1e2e;
    --green:#00ff88; --blue:#00cfff; --red:#ff4466;
    --text:#e0e0f0; --muted:#555577;
    --glow-green:0 0 12px rgba(0,255,136,0.35);
    --glow-blue:0 0 12px rgba(0,207,255,0.35);
  }
  body {
    background:var(--bg); color:var(--text);
    font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;
    user-select:none; -webkit-user-select:none;
    display:flex; justify-content:center;
    min-height:100vh; padding:12px;
  }
  .app { width:100%; max-width:440px; display:flex; flex-direction:column; gap:10px; }

  .header {
    display:flex; justify-content:space-between; align-items:center;
    padding:8px 4px;
  }
  .header h1 { font-size:1.25rem; font-weight:700; color:var(--green); letter-spacing:.05em; }
  .mode-badge {
    padding:4px 10px; border-radius:999px; font-size:.72rem; font-weight:700;
    letter-spacing:.05em; text-transform:uppercase; border:1px solid transparent;
  }
  .mode-badge.balance { background:rgba(0,255,136,.12); color:var(--green); border-color:var(--green); box-shadow:var(--glow-green); }
  .mode-badge.drive   { background:rgba(0,207,255,.12); color:var(--blue);  border-color:var(--blue);  box-shadow:var(--glow-blue); }

  .card {
    background:var(--card); border:1px solid var(--border);
    border-radius:12px; padding:12px;
  }
  .card-title {
    font-size:.7rem; font-weight:600; text-transform:uppercase;
    letter-spacing:.08em; color:var(--muted); margin-bottom:8px;
  }

  .row1 { display:grid; grid-template-columns:1fr 1fr; gap:10px; }
  .eye-wrap { display:flex; justify-content:center; align-items:center; }
  #eyeCanvas { width:100%; height:auto; max-width:180px; border-radius:8px; background:#000; }

  .telem-grid { display:flex; flex-direction:column; gap:6px; }
  .telem-row  { display:flex; justify-content:space-between; align-items:center; }
  .telem-label{ font-size:.78rem; color:var(--muted); }
  .telem-val  { font-size:.85rem; font-weight:700; font-family:'Courier New',monospace; color:var(--blue); }

  .tabs { display:flex; gap:6px; margin-bottom:10px; }
  .tab {
    flex:1; padding:7px; border:1px solid var(--border); border-radius:8px;
    background:transparent; color:var(--muted); font-size:.8rem; font-weight:600;
    cursor:pointer; transition:all .15s;
  }
  .tab.active { background:rgba(0,255,136,.12); color:var(--green); border-color:var(--green); }

  .joy-wrap { display:flex; justify-content:center; align-items:center; padding:8px 0; }
  #joyBase {
    position:relative; width:170px; height:170px; border-radius:50%;
    background:rgba(255,255,255,.03); border:2px solid var(--border);
    touch-action:none; cursor:grab;
  }
  #joyBase.active { border-color:var(--green); box-shadow:var(--glow-green); }
  #joyKnob {
    position:absolute; width:56px; height:56px; border-radius:50%;
    background:var(--green); top:50%; left:50%;
    transform:translate(-50%,-50%); box-shadow:var(--glow-green);
    pointer-events:none; transition:transform .05s;
  }

  .dpad {
    display:grid; grid-template-columns:repeat(3,1fr);
    grid-template-rows:repeat(3,52px); gap:6px; max-width:210px; margin:0 auto;
  }
  .dpad-btn {
    display:flex; align-items:center; justify-content:center;
    background:rgba(255,255,255,.05); border:1px solid var(--border);
    border-radius:10px; font-size:1.1rem; color:var(--text);
    cursor:pointer; -webkit-user-select:none; user-select:none;
    touch-action:none; transition:background .1s, transform .05s;
  }
  .dpad-btn:active, .dpad-btn.pressed {
    background:rgba(0,255,136,.25); border-color:var(--green);
    color:var(--green); transform:scale(.95);
  }
  .dpad-stop { color:var(--red); border-color:rgba(255,68,102,.3); }
  .dpad-stop:active, .dpad-stop.pressed { background:rgba(255,68,102,.25); border-color:var(--red); color:var(--red); }

  .slider-row { display:flex; align-items:center; gap:10px; }
  .slider-label { font-size:.78rem; color:var(--muted); min-width:32px; }
  input[type=range] {
    flex:1; -webkit-appearance:none; height:6px;
    border-radius:3px; background:var(--border); outline:none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance:none; width:20px; height:20px;
    border-radius:50%; background:var(--green);
    box-shadow:var(--glow-green); cursor:pointer;
  }
  .slider-val {
    font-family:'Courier New',monospace; font-size:.85rem;
    color:var(--green); min-width:36px; text-align:right;
  }

  .buzzer-row { display:flex; align-items:center; justify-content:space-between; padding:4px 0; }
  .buzzer-label { font-size:.85rem; }
  .toggle { position:relative; width:52px; height:28px; cursor:pointer; }
  .toggle input { opacity:0; width:0; height:0; }
  .toggle-track {
    position:absolute; inset:0; background:#222; border-radius:999px;
    transition:background .2s; border:1px solid var(--border);
  }
  .toggle input:checked + .toggle-track { background:#00ff8833; border-color:var(--green); }
  .toggle-thumb {
    position:absolute; top:3px; left:3px; width:20px; height:20px;
    border-radius:50%; background:#555; transition:all .2s; pointer-events:none;
  }
  .toggle input:checked ~ .toggle-thumb { left:27px; background:var(--green); box-shadow:var(--glow-green); }

  .mode-btn-row { display:flex; gap:8px; }
  .btn-mode {
    flex:1; padding:10px 8px; border:1px solid var(--border); border-radius:10px;
    background:rgba(255,255,255,.05); color:var(--muted); font-size:.82rem; font-weight:700;
    cursor:pointer; transition:all .2s; text-align:center;
  }
  .btn-mode.active.balance { background:rgba(0,255,136,.18); color:var(--green); border-color:var(--green); box-shadow:var(--glow-green); }
  .btn-mode.active.drive   { background:rgba(0,207,255,.18); color:var(--blue);  border-color:var(--blue);  box-shadow:var(--glow-blue); }

  .btn-action {
    display:block; width:100%; padding:10px; margin-top:8px;
    border:1px solid var(--border); border-radius:8px;
    background:rgba(255,255,255,.06); color:var(--text); font-size:.82rem; font-weight:600;
    cursor:pointer; text-align:center; transition:background .15s, border-color .15s;
  }
  .btn-action:active { background:rgba(0,255,136,.2); border-color:var(--green); }

  .conn-dot {
    width:8px; height:8px; border-radius:50%; background:#333;
    display:inline-block; margin-right:6px; transition:background .3s;
  }
  .conn-dot.on { background:var(--green); box-shadow:var(--glow-green); }
</style>
</head>
<body>
<div class="app">

  <div class="header">
    <h1>RoboBot</h1>
    <div>
      <span class="conn-dot" id="connDot"></span>
      <span id="modeBadge" class="mode-badge balance" onclick="toggleRobotMode()" style="cursor:pointer">BALANCE</span>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Robot Operating Mode (Tap to Switch)</div>
    <div class="mode-btn-row">
      <button class="btn-mode active balance" id="btnModeSegway" onclick="setRobotMode(0)">BALANCE (SEGWAY)</button>
      <button class="btn-mode" id="btnModeDrive" onclick="setRobotMode(1)">REGULAR DRIVE</button>
    </div>
  </div>

  <div class="row1">
    <div class="card">
      <div class="card-title">Robot Eye</div>
      <div class="eye-wrap">
        <canvas id="eyeCanvas" width="200" height="100"></canvas>
      </div>
    </div>
    <div class="card">
      <div class="card-title">Telemetry</div>
      <div class="telem-grid">
        <div class="telem-row">
          <span class="telem-label">Angle</span>
          <span class="telem-val" id="tAngle">-- deg</span>
        </div>
        <div class="telem-row">
          <span class="telem-label">Distance</span>
          <span class="telem-val" id="tDist">-- cm</span>
        </div>
        <div class="telem-row">
          <span class="telem-label">Speed</span>
          <span class="telem-val" id="tSpeed">--%</span>
        </div>
        <div class="telem-row">
          <span class="telem-label">Motor L</span>
          <span class="telem-val" id="tMotL">--</span>
        </div>
        <div class="telem-row">
          <span class="telem-label">Motor R</span>
          <span class="telem-val" id="tMotR">--</span>
        </div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="tabs">
      <button class="tab" id="tabJoy" onclick="switchTab('joy')">Joystick</button>
      <button class="tab" id="tabBtn" onclick="switchTab('btn')">Buttons</button>
    </div>

    <div id="panelJoy">
      <div class="joy-wrap">
        <div id="joyBase">
          <div id="joyKnob"></div>
        </div>
      </div>
    </div>

    <div id="panelBtn" style="display:none">
      <div class="dpad">
        <div></div>
        <div class="dpad-btn" id="dUp"    data-dx="0"  data-dy="1">UP</div>
        <div></div>
        <div class="dpad-btn" id="dLeft"  data-dx="-1" data-dy="0">LEFT</div>
        <div class="dpad-btn dpad-stop" id="dStop" data-dx="0" data-dy="0">STOP</div>
        <div class="dpad-btn" id="dRight" data-dx="1"  data-dy="0">RIGHT</div>
        <div></div>
        <div class="dpad-btn" id="dDown"  data-dx="0"  data-dy="-1">DOWN</div>
        <div></div>
      </div>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Speed</div>
    <div class="slider-row">
      <span class="slider-label">0%</span>
      <input type="range" id="speedSlider" min="0" max="100" value="70">
      <span class="slider-val" id="speedVal">70%</span>
    </div>
  </div>

  <div class="card">
    <div class="card-title">Hardware & Balance Settings</div>
    <div class="buzzer-row" style="margin-bottom:8px">
      <span class="buzzer-label">Invert Motor Direction</span>
      <label class="toggle">
        <input type="checkbox" id="invertToggle">
        <span class="toggle-track"></span>
        <span class="toggle-thumb"></span>
      </label>
    </div>
    <div class="buzzer-row" style="margin-bottom:8px">
      <span class="buzzer-label">Buzzer</span>
      <label class="toggle">
        <input type="checkbox" id="buzzerToggle" checked>
        <span class="toggle-track"></span>
        <span class="toggle-thumb"></span>
      </label>
    </div>
    <button class="btn-action" onclick="triggerCalibrate()">Zero / Calibrate Upright Balance</button>
  </div>

  <div class="card">
    <div class="card-title">Live Balance PID & Angle Trim</div>
    <div style="font-size:12px;color:var(--text-dim);margin-bottom:8px">
      Adjust PID power, damping and angle trim in real-time.
    </div>

    <div style="margin-bottom:12px">
      <div style="display:flex;justify-content:space-between;margin-bottom:4px;font-size:13px">
        <span>Balance Angle Trim:</span>
        <strong id="trimVal">0.0&deg;</strong>
      </div>
      <div style="display:flex;gap:6px">
        <button class="btn-action" style="flex:1;padding:8px 4px;margin-top:0" onclick="adjustTrim(-0.2)">&larr; Back 0.2&deg;</button>
        <button class="btn-action" style="flex:1;padding:8px 4px;margin-top:0;background:#334155" onclick="setTrim(0.0)">0.0&deg;</button>
        <button class="btn-action" style="flex:1;padding:8px 4px;margin-top:0" onclick="adjustTrim(0.2)">Fwd 0.2&deg; &rarr;</button>
      </div>
    </div>

    <div style="margin-bottom:10px">
      <div style="display:flex;justify-content:space-between;font-size:13px;margin-bottom:2px">
        <span>Kp (Stiffness / Power):</span>
        <strong id="kpVal">24.0</strong>
      </div>
      <input type="range" id="kpSlider" min="5" max="60" step="0.5" value="24" oninput="onPidInput()">
    </div>

    <div style="margin-bottom:10px">
      <div style="display:flex;justify-content:space-between;font-size:13px;margin-bottom:2px">
        <span>Kd (Damping / Smoothness):</span>
        <strong id="kdVal">1.2</strong>
      </div>
      <input type="range" id="kdSlider" min="0" max="4.0" step="0.1" value="1.2" oninput="onPidInput()">
    </div>

    <div style="margin-bottom:10px">
      <div style="display:flex;justify-content:space-between;font-size:13px;margin-bottom:2px">
        <span>Ki (Drift / Offset Fix):</span>
        <strong id="kiVal">0.30</strong>
      </div>
      <input type="range" id="kiSlider" min="0" max="2.0" step="0.05" value="0.3" oninput="onPidInput()">
    </div>

    <div style="margin-bottom:12px">
      <div style="display:flex;justify-content:space-between;font-size:13px;margin-bottom:2px">
        <span>Min Motor Torque (Deadband):</span>
        <strong id="minSpdVal">28</strong>
      </div>
      <input type="range" id="minSpdSlider" min="10" max="60" step="1" value="28" oninput="onPidInput()">
    </div>

    <button class="btn-action" style="margin-top:0;background:var(--accent)" onclick="sendPidNow()">Apply PID Settings</button>
  </div>

</div>

<script>
// RoboBot Dashboard Client Script
const state = {
  joyX: 0, joyY: 0,
  speed: 70,
  buzzer: true,
  invertMotors: false,
  lastSent: { joyX: null, joyY: null, speed: null, buzzer: null },
  expression: 'EYE_NORMAL',
  mode: 'BALANCE',
  connected: false,
};

function setRobotMode(m) {
  state.mode = m === 0 ? 'BALANCE' : 'DRIVE';
  updateModeBtns(m);
  sendImmediate({ mode: m });
}

function toggleRobotMode() {
  setRobotMode(state.mode === 'BALANCE' ? 1 : 0);
}

function updateModeBtns(m) {
  const isBal = m === 0;
  const badge = document.getElementById('modeBadge');
  if (badge) {
    badge.textContent = isBal ? 'BALANCE' : 'DRIVE';
    badge.className = 'mode-badge ' + (isBal ? 'balance' : 'drive');
  }
  const bBal = document.getElementById('btnModeSegway');
  const bDrv = document.getElementById('btnModeDrive');
  if (bBal && bDrv) {
    bBal.className = 'btn-mode' + (isBal ? ' active balance' : '');
    bDrv.className = 'btn-mode' + (!isBal ? ' active drive' : '');
  }
}

function triggerCalibrate() {
  if (confirm('Hold the robot vertically upright at its balance point, then press OK.')) {
    sendImmediate({ calibrate: true });
  }
}

let currentTrim = 0.0;
let pidLoaded = false;

function adjustTrim(delta) {
  currentTrim = Math.round((currentTrim + delta) * 10) / 10;
  document.getElementById('trimVal').innerHTML = currentTrim.toFixed(1) + '&deg;';
  sendPidNow();
}

function setTrim(val) {
  currentTrim = val;
  document.getElementById('trimVal').innerHTML = currentTrim.toFixed(1) + '&deg;';
  sendPidNow();
}

function onPidInput() {
  document.getElementById('kpVal').textContent = parseFloat(document.getElementById('kpSlider').value).toFixed(1);
  document.getElementById('kdVal').textContent = parseFloat(document.getElementById('kdSlider').value).toFixed(1);
  document.getElementById('kiVal').textContent = parseFloat(document.getElementById('kiSlider').value).toFixed(2);
  document.getElementById('minSpdVal').textContent = parseInt(document.getElementById('minSpdSlider').value);
}

function sendPidNow() {
  const kp = parseFloat(document.getElementById('kpSlider').value);
  const kd = parseFloat(document.getElementById('kdSlider').value);
  const ki = parseFloat(document.getElementById('kiSlider').value);
  const minSpd = parseInt(document.getElementById('minSpdSlider').value);
  sendImmediate({ kp: kp, kd: kd, ki: ki, trim: currentTrim, minSpeed: minSpd });
}

async function sendImmediate(extra) {
  try {
    await fetch('/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(Object.assign({
        joyX: state.joyX,
        joyY: state.joyY,
        speed: state.speed,
        buzzer: state.buzzer,
        invertMotors: state.invertMotors
      }, extra))
    });
  } catch (_) {}
}

const invertToggle = document.getElementById('invertToggle');
if (invertToggle) {
  invertToggle.addEventListener('change', () => {
    state.invertMotors = invertToggle.checked;
    sendImmediate({ invertMotors: state.invertMotors });
  });
}

function switchTab(which) {
  const isJoy = which === 'joy';
  document.getElementById('tabJoy').classList.toggle('active', isJoy);
  document.getElementById('tabBtn').classList.toggle('active', !isJoy);
  document.getElementById('panelJoy').style.display = isJoy ? '' : 'none';
  document.getElementById('panelBtn').style.display = isJoy ? 'none' : '';
  localStorage.setItem('rbtab', which);
  if (!isJoy) { state.joyX = 0; state.joyY = 0; }
}

(function initTab() {
  const saved = localStorage.getItem('rbtab') || 'joy';
  switchTab(saved);
})();

const speedSlider = document.getElementById('speedSlider');
const speedValEl  = document.getElementById('speedVal');
speedSlider.addEventListener('input', () => {
  state.speed = parseInt(speedSlider.value);
  speedValEl.textContent = state.speed + '%';
});

const buzzerToggle = document.getElementById('buzzerToggle');
buzzerToggle.addEventListener('change', () => { state.buzzer = buzzerToggle.checked; });

// Virtual Joystick
(function initJoystick() {
  const base   = document.getElementById('joyBase');
  const knob   = document.getElementById('joyKnob');
  const radius = 50;
  let dragging = false;
  let bRect;

  function getPos(e) {
    const src = e.touches ? e.touches[0] : e;
    return { x: src.clientX, y: src.clientY };
  }

  function move(e) {
    if (!dragging) return;
    e.preventDefault();
    const { x, y } = getPos(e);
    let dx = x - (bRect.left + bRect.width / 2);
    let dy = y - (bRect.top  + bRect.height / 2);
    const dist = Math.sqrt(dx * dx + dy * dy);
    if (dist > radius) { dx = dx / dist * radius; dy = dy / dist * radius; }
    knob.style.left = (50 + (dx / bRect.width  * 100)) + '%';
    knob.style.top  = (50 + (dy / bRect.height * 100)) + '%';
    state.joyX =  Math.round(dx / radius * 100);
    state.joyY = -Math.round(dy / radius * 100);
  }

  function start(e) {
    dragging = true;
    bRect = base.getBoundingClientRect();
    base.classList.add('active');
    move(e);
  }

  function end() {
    dragging = false;
    base.classList.remove('active');
    knob.style.left = '50%';
    knob.style.top  = '50%';
    state.joyX = 0;
    state.joyY = 0;
  }

  base.addEventListener('mousedown',  start);
  base.addEventListener('touchstart', start, { passive: false });
  window.addEventListener('mousemove',  move);
  window.addEventListener('touchmove',  move, { passive: false });
  window.addEventListener('mouseup',  end);
  window.addEventListener('touchend', end);
})();

// D-Pad Buttons
(function initDPad() {
  const dpadBtns = document.querySelectorAll('.dpad-btn');
  let activeBtn = null;

  function setDir(btn) {
    if (!btn) { state.joyX = 0; state.joyY = 0; return; }
    const dx = parseInt(btn.dataset.dx || 0);
    const dy = parseInt(btn.dataset.dy || 0);
    state.joyX = dx * 100;
    state.joyY = dy * 100;
  }

  function press(e) {
    e.preventDefault();
    if (activeBtn) activeBtn.classList.remove('pressed');
    activeBtn = e.currentTarget;
    activeBtn.classList.add('pressed');
    setDir(activeBtn);
  }

  function release() {
    if (activeBtn) activeBtn.classList.remove('pressed');
    activeBtn = null;
    setDir(null);
  }

  dpadBtns.forEach(btn => {
    btn.addEventListener('mousedown',  press);
    btn.addEventListener('touchstart', press, { passive: false });
  });
  window.addEventListener('mouseup',  release);
  window.addEventListener('touchend', release);
})();

// Periodic control command sender (100 ms interval)
setInterval(async () => {
  const payload = {
    joyX:  state.joyX,
    joyY:  state.joyY,
    speed: state.speed,
    buzzer: state.buzzer,
  };
  const changed = payload.joyX !== state.lastSent.joyX ||
                  payload.joyY !== state.lastSent.joyY ||
                  payload.speed !== state.lastSent.speed ||
                  payload.buzzer !== state.lastSent.buzzer;
  if (!changed && payload.joyX === 0 && payload.joyY === 0 &&
      state.lastSent.joyX === 0 && state.lastSent.joyY === 0) return;

  try {
    await fetch('/control', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload),
    });
    Object.assign(state.lastSent, payload);
  } catch (_) {}
}, 100);

// Telemetry polling (200 ms interval)
async function pollTelemetry() {
  try {
    const r = await fetch('/status');
    if (!r.ok) throw new Error();
    const d = await r.json();

    document.getElementById('tAngle').textContent = d.angle.toFixed(1) + ' deg';
    document.getElementById('tDist').textContent  = d.distance >= 0
        ? d.distance.toFixed(0) + ' cm' : '-- cm';
    document.getElementById('tSpeed').textContent = state.speed + '%';
    document.getElementById('tMotL').textContent  = d.motorL;
    document.getElementById('tMotR').textContent  = d.motorR;

    const badge = document.getElementById('modeBadge');
    if (d.mode !== undefined) {
      state.mode = d.mode === 0 ? 'BALANCE' : 'DRIVE';
      updateModeBtns(d.mode);
    }

    if (d.buzzerEnabled !== undefined && buzzerToggle.checked !== d.buzzerEnabled) {
      buzzerToggle.checked = d.buzzerEnabled;
      state.buzzer = d.buzzerEnabled;
    }

    state.expression = d.expression || 'EYE_NORMAL';
    state.connected = true;
    document.getElementById('connDot').classList.add('on');

    if (!pidLoaded && d.kp !== undefined) {
      pidLoaded = true;
      document.getElementById('kpSlider').value = d.kp;
      document.getElementById('kdSlider').value = d.kd;
      document.getElementById('kiSlider').value = d.ki;
      if (d.minSpeed !== undefined) {
        document.getElementById('minSpdSlider').value = d.minSpeed;
        document.getElementById('minSpdVal').textContent = parseInt(d.minSpeed);
      }
      currentTrim = d.trim || 0.0;
      document.getElementById('kpVal').textContent = d.kp.toFixed(1);
      document.getElementById('kdVal').textContent = d.kd.toFixed(1);
      document.getElementById('kiVal').textContent = d.ki.toFixed(2);
      document.getElementById('trimVal').innerHTML = currentTrim.toFixed(1) + '&deg;';
    }
  } catch (_) {
    state.connected = false;
    document.getElementById('connDot').classList.remove('on');
  }
}
setInterval(pollTelemetry, 200);
pollTelemetry();

// Robot eye canvas renderer
(function initEye() {
  const canvas = document.getElementById('eyeCanvas');
  const ctx    = canvas.getContext('2d');
  const W = 200, H = 100;

  const EYE = {
    left:  { x: 20,  y: 20, w: 70, h: 60 },
    right: { x: 110, y: 20, w: 70, h: 60 },
  };

  let spinAngle = 0;
  let zzzPhase  = 0;
  let heartBeat = 0;
  let sweatY    = 0;

  function clearCanvas() {
    ctx.fillStyle = '#000';
    ctx.fillRect(0, 0, W, H);
  }

  function drawEyeBox(e, fillColor) {
    ctx.fillStyle   = fillColor || '#fff';
    ctx.strokeStyle = '#fff';
    ctx.lineWidth   = 2;
    ctx.beginPath();
    ctx.roundRect(e.x, e.y, e.w, e.h, 6);
    ctx.fill();
  }

  function drawPupil(e, offX, offY, r) {
    const cx = e.x + e.w / 2 + (offX || 0);
    const cy = e.y + e.h / 2 + (offY || 0);
    ctx.fillStyle = '#000';
    ctx.beginPath();
    ctx.arc(cx, cy, r || 14, 0, Math.PI * 2);
    ctx.fill();
  }

  function drawNormal() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      drawPupil(e, 0, 0, 14);
    });
  }

  function drawHappy() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      drawPupil(e, 0, 6, 14);
      ctx.fillStyle = '#000';
      ctx.fillRect(e.x + 2, e.y + 2, e.w - 4, e.h / 2 - 2);
      ctx.strokeStyle = '#fff';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(e.x + e.w / 2, e.y + e.h / 2 + 8, 18, 0.1 * Math.PI, 0.9 * Math.PI);
      ctx.stroke();
    });
  }

  function drawSurprised() {
    [EYE.left, EYE.right].forEach(e => {
      const big = { x: e.x - 4, y: e.y - 6, w: e.w + 8, h: e.h + 10 };
      drawEyeBox(big);
      drawPupil(big, 0, 0, 18);
    });
  }

  function drawAngry() {
    [EYE.left, EYE.right].forEach((e, i) => {
      drawEyeBox(e);
      drawPupil(e, 0, 10, 12);
      ctx.strokeStyle = '#000';
      ctx.lineWidth = 5;
      ctx.beginPath();
      if (i === 0) {
        ctx.moveTo(e.x + 2, e.y + 2);
        ctx.lineTo(e.x + e.w - 2, e.y + 16);
      } else {
        ctx.moveTo(e.x + e.w - 2, e.y + 2);
        ctx.lineTo(e.x + 2, e.y + 16);
      }
      ctx.stroke();
    });
  }

  function drawSleepy() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      drawPupil(e, 0, 0, 12);
      ctx.fillStyle = '#000';
      ctx.fillRect(e.x, e.y, e.w, e.h / 2);
    });
  }

  function drawLookLeft() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      drawPupil(e, -18, 0, 14);
    });
  }

  function drawLookRight() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      drawPupil(e, 18, 0, 14);
    });
  }

  function drawDizzy() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      const cx = e.x + e.w / 2, cy = e.y + e.h / 2;
      ctx.strokeStyle = '#000';
      ctx.lineWidth = 4;
      const s = 16;
      ctx.beginPath(); ctx.moveTo(cx - s, cy - s); ctx.lineTo(cx + s, cy + s); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(cx + s, cy - s); ctx.lineTo(cx - s, cy + s); ctx.stroke();
    });
  }

  function drawCool() {
    ctx.fillStyle = '#fff';
    ctx.beginPath();
    ctx.roundRect(10, 30, 180, 40, 8);
    ctx.fill();
    ctx.fillStyle = '#000';
    ctx.fillRect(12, 32, 176, 36);
    ctx.fillStyle = 'rgba(255,255,255,0.15)';
    ctx.fillRect(14, 33, 80, 10);
    ctx.fillRect(104, 33, 80, 10);
  }

  function drawSmirk() {
    drawEyeBox(EYE.left);
    drawPupil(EYE.left, 0, 0, 14);
    const e = EYE.right;
    drawEyeBox(e);
    drawPupil(e, 0, 8, 12);
    ctx.fillStyle = '#000';
    ctx.fillRect(e.x, e.y, e.w, e.h * 0.4);
  }

  function drawZzz() {
    drawSleepy();
    const zy = 10 + Math.sin(zzzPhase) * 4;
    ctx.fillStyle = '#00cfff';
    ctx.font = 'bold 14px system-ui';
    ctx.fillText('z', 155, zy + 20);
    ctx.font = 'bold 18px system-ui';
    ctx.fillText('Z', 168, zy + 10);
    ctx.font = 'bold 22px system-ui';
    ctx.fillText('Z', 183, zy);
    zzzPhase += 0.05;
  }

  function drawHeart() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      const cx = e.x + e.w / 2, cy = e.y + e.h / 2;
      const scale = 1 + 0.15 * Math.sin(heartBeat);
      ctx.save();
      ctx.translate(cx, cy);
      ctx.scale(scale, scale);
      ctx.fillStyle = '#ff4466';
      ctx.beginPath();
      ctx.moveTo(0, 6);
      ctx.bezierCurveTo(-16, -6, -18, 12, 0, 20);
      ctx.bezierCurveTo( 18, 12,  16, -6, 0, 6);
      ctx.fill();
      ctx.restore();
    });
    heartBeat += 0.12;
  }

  function drawSweat() {
    drawNormal();
    const sx = EYE.right.x + EYE.right.w + 6;
    const sy = EYE.right.y + sweatY;
    ctx.fillStyle = '#00cfff';
    ctx.beginPath();
    ctx.arc(sx, sy + 8, 5, 0, Math.PI * 2);
    ctx.fill();
    ctx.beginPath();
    ctx.moveTo(sx, sy);
    ctx.lineTo(sx - 4, sy + 8);
    ctx.lineTo(sx + 4, sy + 8);
    ctx.closePath();
    ctx.fill();
    sweatY = (sweatY + 0.5) % (EYE.right.h - 4);
  }

  function drawSpinning() {
    [EYE.left, EYE.right].forEach(e => {
      drawEyeBox(e);
      const cx = e.x + e.w / 2, cy = e.y + e.h / 2;
      ctx.save();
      ctx.translate(cx, cy);
      ctx.rotate(spinAngle);
      ctx.strokeStyle = '#00cfff';
      ctx.lineWidth = 4;
      for (let i = 0; i < 4; i++) {
        ctx.beginPath();
        ctx.moveTo(0, 0);
        ctx.lineTo(0, -18);
        ctx.stroke();
        ctx.rotate(Math.PI / 2);
      }
      ctx.restore();
    });
    spinAngle += 0.08;
  }

  function renderEye(expr) {
    clearCanvas();
    switch (expr) {
      case 'EYE_HAPPY':      drawHappy();     break;
      case 'EYE_SURPRISED':  drawSurprised(); break;
      case 'EYE_ANGRY':      drawAngry();     break;
      case 'EYE_SLEEPY':     drawSleepy();    break;
      case 'EYE_LOOK_LEFT':  drawLookLeft();  break;
      case 'EYE_LOOK_RIGHT': drawLookRight(); break;
      case 'EYE_DIZZY':      drawDizzy();     break;
      case 'EYE_COOL':       drawCool();      break;
      case 'EYE_SMIRK':      drawSmirk();     break;
      case 'EYE_ZZZ':        drawZzz();       break;
      case 'EYE_HEART':      drawHeart();     break;
      case 'EYE_SWEAT':      drawSweat();     break;
      case 'EYE_SPINNING':   drawSpinning();  break;
      default:               drawNormal();    break;
    }
  }

  function animate() {
    renderEye(state.expression);
    requestAnimationFrame(animate);
  }
  animate();
})();
</script>
</body>
</html>
)rawliteral";

// ============================================================
// TELEMETRY UPDATE
// ============================================================
void webserver_set_telemetry(float angle, float distance,
                              const char* expression,
                              RobotMode mode,
                              bool obsEnabled, bool buzzEnabled,
                              int motorL, int motorR,
                              float kp, float ki, float kd, float trim, float minSpeed)
{
    g_telem.angle       = angle;
    g_telem.distance    = distance;
    g_telem.mode        = mode;
    g_telem.obsEnabled  = obsEnabled;
    g_telem.buzzEnabled = buzzEnabled;
    g_telem.motorL      = motorL;
    g_telem.motorR      = motorR;
    g_telem.kp          = kp;
    g_telem.ki          = ki;
    g_telem.kd          = kd;
    g_telem.trim        = trim;
    g_telem.minSpeed    = minSpeed;
    if (expression) {
        strncpy(g_telem.expression, expression, sizeof(g_telem.expression) - 1);
        g_telem.expression[sizeof(g_telem.expression) - 1] = '\0';
    }
}

// ============================================================
// HTTP ROUTE HANDLERS
// ============================================================

static void handleRoot() {
    httpServer.send_P(200, "text/html", DASHBOARD_HTML);
}

static void handleStatus() {
    char buf[384];
    snprintf(buf, sizeof(buf),
        "{\"angle\":%.2f,\"distance\":%.1f,\"expression\":\"%s\","
        "\"mode\":%d,\"obsEnabled\":%s,\"buzzerEnabled\":%s,"
        "\"motorL\":%d,\"motorR\":%d,\"kp\":%.2f,\"ki\":%.2f,\"kd\":%.2f,\"trim\":%.2f,\"minSpeed\":%.0f}",
        g_telem.angle,
        g_telem.distance,
        g_telem.expression,
        (int)g_telem.mode,
        g_telem.obsEnabled  ? "true" : "false",
        g_telem.buzzEnabled ? "true" : "false",
        g_telem.motorL,
        g_telem.motorR,
        g_telem.kp,
        g_telem.ki,
        g_telem.kd,
        g_telem.trim,
        g_telem.minSpeed
    );
    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.send(200, "application/json", buf);
}

static void handleControl() {
    if (!g_cmd) { httpServer.send(500, "text/plain", "not init"); return; }

    String body = httpServer.arg("plain");

    auto getInt = [&](const char* key) -> int {
        String k = "\""; k += key; k += "\":";
        int idx = body.indexOf(k);
        if (idx < 0) return 0;
        idx += k.length();
        while (idx < (int)body.length() && body[idx] == ' ') idx++;
        return body.substring(idx).toInt();
    };
    auto getBool = [&](const char* key) -> bool {
        String k = "\""; k += key; k += "\":";
        int idx = body.indexOf(k);
        if (idx < 0) return false;
        idx += k.length();
        while (idx < (int)body.length() && body[idx] == ' ') idx++;
        return body.substring(idx).startsWith("true");
    };
    auto getFloat = [&](const char* key, float def) -> float {
        String k = "\""; k += key; k += "\":";
        int idx = body.indexOf(k);
        if (idx < 0) return def;
        idx += k.length();
        while (idx < (int)body.length() && body[idx] == ' ') idx++;
        return body.substring(idx).toFloat();
    };

    if (body.indexOf("\"joyX\":") >= 0) {
        g_cmd->joyX = constrain(getInt("joyX"), -100, 100);
        g_cmd->joyY = constrain(getInt("joyY"), -100, 100);
    }
    if (body.indexOf("\"speed\":") >= 0) {
        g_cmd->speed = constrain(getInt("speed"), 0, 100);
    }
    if (body.indexOf("\"buzzer\":") >= 0) {
        g_cmd->buzzerEnabled = getBool("buzzer");
    }
    if (body.indexOf("\"mode\":") >= 0) {
        g_cmd->requestedMode = getInt("mode");
    }
    if (body.indexOf("\"invertMotors\":") >= 0) {
        g_cmd->invertMotors = getBool("invertMotors");
    }
    if (body.indexOf("\"calibrate\":") >= 0) {
        g_cmd->triggerCalib = getBool("calibrate");
    }
    if (body.indexOf("\"kp\":") >= 0) {
        g_cmd->pidKp       = getFloat("kp", 24.0f);
        g_cmd->pidKi       = getFloat("ki", 0.3f);
        g_cmd->pidKd       = getFloat("kd", 1.2f);
        g_cmd->pidTrim     = getFloat("trim", 0.0f);
        g_cmd->pidMinSpeed = getFloat("minSpeed", 28.0f);
        g_cmd->updatePid   = true;
    }
    g_cmd->updated = true;

    httpServer.sendHeader("Access-Control-Allow-Origin", "*");
    httpServer.send(200, "application/json", "{\"ok\":true}");
}

// Captive portal probes for Android, iOS, Windows
static void handleGenerate204() {
    httpServer.send(204, "text/plain", "");
}
static void handleHotspot() {
    httpServer.send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
}
static void handleNcsi() {
    httpServer.send(200, "text/plain", "Microsoft NCSI");
}

static void handleNotFound() {
    httpServer.sendHeader("Location", "/", true);
    httpServer.send(302, "text/plain", "");
}

// ============================================================
// PUBLIC API IMPLEMENTATION
// ============================================================

void webserver_init(WebCommand* cmd) {
    g_cmd = cmd;
    if (g_cmd) {
        g_cmd->joyX          = 0;
        g_cmd->joyY          = 0;
        g_cmd->speed         = 70;
        g_cmd->buzzerEnabled = true;
        g_cmd->requestedMode = -1;
        g_cmd->invertMotors  = false;
        g_cmd->triggerCalib  = false;
        g_cmd->updatePid     = false;
        g_cmd->pidKp         = PID_KP;
        g_cmd->pidKi         = PID_KI;
        g_cmd->pidKd         = PID_KD;
        g_cmd->pidTrim       = 0.0f;
        g_cmd->pidMinSpeed   = MOTOR_MIN_SPEED;
        g_cmd->updated       = false;
    }
}

void webserver_start() {
    Serial.println("[WiFi] Starting Access Point: " WIFI_SSID);
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD[0] == '\0' ? nullptr : WIFI_PASSWORD);

    IPAddress apIP = WiFi.softAPIP();
    Serial.print("[WiFi] AP IP: ");
    Serial.println(apIP);

    dnsServer.start(DNS_PORT, "*", apIP);
    Serial.println("[DNS] Captive portal DNS started");

    httpServer.on("/",                   HTTP_GET,  handleRoot);
    httpServer.on("/status",             HTTP_GET,  handleStatus);
    httpServer.on("/control",            HTTP_POST, handleControl);
    httpServer.on("/generate_204",       HTTP_GET,  handleGenerate204);
    httpServer.on("/hotspot-detect.html",HTTP_GET,  handleHotspot);
    httpServer.on("/ncsi.txt",           HTTP_GET,  handleNcsi);
    httpServer.onNotFound(handleNotFound);

    httpServer.begin();
    Serial.println("[HTTP] Web server started on port 80");
}

void webserver_update() {
    dnsServer.processNextRequest();
    httpServer.handleClient();
}

bool webserver_is_connected() {
    return WiFi.softAPgetStationNum() > 0;
}
