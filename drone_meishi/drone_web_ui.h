#pragma once

// ============================================================
// Web UI
// ============================================================

static const char kIndexHtml[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport"
        content="width=device-width,initial-scale=1,viewport-fit=cover,user-scalable=no">
  <title>ESP32 Drone</title>
  <style>
    :root{
      --gap: 8px;
      --card-pad: 10px;

      /* portrait fallback */
      --pad-size: min(30vh, 30vw, 320px);
      --center-width: min(360px, 96vw);
    }

    html, body { height: 100%; }

    body {
      font-family: sans-serif;
      margin: 0;
      padding: calc(6px + env(safe-area-inset-top))
               calc(6px + env(safe-area-inset-right))
               calc(6px + env(safe-area-inset-bottom))
               calc(6px + env(safe-area-inset-left));
      display: flex;
      flex-direction: column;
      gap: var(--gap);
      box-sizing: border-box;
      overflow: hidden;
      background: #fff;
    }

    * { box-sizing: border-box; }

    .card {
      border: 1px solid #ccc;
      border-radius: 10px;
      padding: var(--card-pad);
      background: #fff;
    }

    .row {
      display: flex;
      gap: 10px;
      align-items: center;
      flex-wrap: wrap;
    }

    .danger { font-weight: 700; }

    button {
      font-size: 16px;
      padding: 10px 14px;
    }

    input[type="number"] {
      width: 90px;
      font-size: 16px;
    }

    small { color: #555; }

    .kv {
      display: grid;
      grid-template-columns: 110px 1fr;
      gap: 6px 10px;
    }

    #modeBar {
      display: flex;
      gap: 6px;
      align-items: center;
      justify-content: center;
    }

    .modeBtn {
      font-size: 14px;
      padding: 6px 10px;
      border: 1px solid #888;
      border-radius: 8px;
      background: #f4f4f4;
    }

    .modeBtn.active {
      background: #222;
      color: #fff;
      border-color: #222;
    }

    #controlView,
    #settingsView {
      flex: 1 1 auto;
      width: 100%;
      min-height: 0;
    }

    #settingsView {
      display: none;
      overflow: auto;
    }

    body[data-mode="control"] #controlView { display: block; }
    body[data-mode="control"] #settingsView { display: none; }
    body[data-mode="settings"] #controlView { display: none; }
    body[data-mode="settings"] #settingsView { display: block; }

    #settingsLayout {
      height: 100%;
      display: flex;
      flex-direction: column;
      gap: var(--gap);
      align-items: center;
      justify-content: flex-start;
    }

    #attCard {
      width: min(320px, 96vw);
      display: flex;
      flex-direction: column;
      gap: 8px;
    }

    #attCanvas {
      width: 100%;
      max-width: 280px;
      aspect-ratio: 1;
      display: block;
      margin: 0 auto;
      background: #f7f7f7;
      border-radius: 10px;
    }

    #tuneCard {
      width: min(420px, 96vw);
      display: flex;
      flex-direction: column;
      gap: 10px;
    }

    .section { margin-top: 8px; }

    .section .row { margin-top: 6px; }

    .muted { color: #666; font-size: 12px; }

    #app {
      height: 100%;
      width: 100%;
      display: flex;
      flex-direction: column;
      gap: var(--gap);
      align-items: center;
      justify-content: flex-start;
    }

    .padCol {
      display: flex;
      align-items: center;
      justify-content: center;
      width: 100%;
    }

    .joy {
      width: var(--pad-size);
      height: var(--pad-size);
      border: 2px solid #444;
      border-radius: 12px;
      position: relative;
      touch-action: none;
      user-select: none;
      overflow: hidden;
    }

    .joyLabel {
      position: absolute;
      left: 10px;
      top: 10px;
      font-size: 14px;
    }

    .stick {
      width: 60px;
      height: 60px;
      border-radius: 999px;
      background: #444;
      opacity: 0.6;
      position: absolute;
      left: 50%;
      top: 50%;
      transform: translate(-50%,-50%);
    }

    #centerCol {
      width: var(--center-width);
      display: flex;
      flex-direction: column;
      gap: var(--gap);
    }

    #telemetryCard { flex: 1; overflow: hidden; }

    @media (orientation:landscape) {
      :root{
        /* iPhone横持ちを想定して、中央パネルの幅を確保できるサイズ */
        --pad-size: min(60vh, 32vw, 260px);
        --center-width: min(360px, 42vw, 380px);
        --card-pad: 6px;
      }

      #app {
        height: 100%;
        flex-direction: row;
        align-items: stretch;
        justify-content: center;
        gap: 6px;
      }

      .padCol {
        width: auto;
        flex: 0 0 auto;
      }

      #centerCol {
        height: 100%;
        max-height: 100%;
        min-height: 0;
        font-size: 14px;
      }

      #controlCard small { display: none; }

      #telemetryCard {
        overflow: auto;
        min-height: 0;
      }

      #telemetryCard .kv {
        grid-template-columns: 86px minmax(0, 1fr) 70px minmax(0, 1fr);
        gap: 4px 8px;
        font-size: 13px;
      }

      #telemetryCard .kv div {
        overflow: hidden;
        text-overflow: ellipsis;
        white-space: nowrap;
      }

      #settingsLayout {
        flex-direction: row;
        align-items: stretch;
        justify-content: center;
      }

      #attCard {
        flex: 0 0 260px;
      }

      #attCanvas {
        max-width: 240px;
      }

      #tuneCard {
        flex: 1 1 360px;
        max-width: 520px;
        min-height: 0;
        overflow: auto;
      }

      .stick {
        width: 56px;
        height: 56px;
      }

      button {
        padding: 8px 12px;
      }
    }
  </style>
</head>
<body data-mode="control">
  <div id="modeBar" class="card">
    <button id="modeControl" class="modeBtn active">CONTROL</button>
    <button id="modeSettings" class="modeBtn">SETTINGS</button>
  </div>

  <div id="controlView">
    <div id="app">
    <div class="padCol">
      <div class="joy" id="leftJoy">
        <div class="joyLabel">Yaw / Throttle</div>
        <div class="stick" id="leftStick"></div>
      </div>
    </div>

    <div id="centerCol">
      <div class="card" id="controlCard">
        <div class="row">
          <label><input id="armSwitch" type="checkbox"> ARM request</label>
          <button id="disarmBtn" class="danger">DISARM</button>
        </div>

        <div class="row" style="margin-top:6px;">
          <div>Throttle: <b id="thv">0.00</b></div>
          <div>WS: <span id="ws">-</span></div>
          <div>State: <b id="state">-</b></div>
        </div>

        <small>Left Y: bottom=0, top=1 (hold). DISARM always stops.</small>
      </div>

      <div class="card" id="telemetryCard">
        <div class="kv">
          <div>Att (deg)</div><div id="att">-</div>
          <div>dt / Hz</div><div id="dtHz">-</div>
          <div>Cmd age</div><div id="cmdAge">-</div>
          <div>Motors</div><div id="mot">-</div>
          <div>VBatt</div><div id="vbatt">-</div>
          <div>FS</div><div id="fs">-</div>
        </div>
      </div>

    </div>

    <div class="padCol">
      <div class="joy" id="rightJoy">
        <div class="joyLabel">Roll / Pitch</div>
        <div class="stick" id="rightStick"></div>
      </div>
    </div>
    </div>
  </div>

  <div id="settingsView">
    <div id="settingsLayout">
      <div class="card" id="attCard">
        <div class="row">
          <b>Attitude</b>
          <span class="muted" id="attNums">-</span>
        </div>
        <canvas id="attCanvas"></canvas>
        <div class="row">
          <button id="levelBtn">LEVEL</button>
          <button id="gyroCalBtn">GYRO CAL</button>
          <span class="muted" id="calState">CAL: -</span>
        </div>
        <small class="muted">Keep still on a flat surface.</small>
        <div class="kv">
          <div>Roll</div><div id="attRoll">-</div>
          <div>Pitch</div><div id="attPitch">-</div>
          <div>Yaw</div><div id="attYaw">-</div>
        </div>
      </div>

      <div class="card" id="tuneCard">
        <div class="row">
          <button id="syncBtn">GET</button>
          <button id="saveBtn">SAVE</button>
          <span class="muted">PID + config</span>
          <span class="muted" id="actionStatus">-</span>
        </div>
        <div class="row">
          <b>Status</b>
          <span id="statusMsg">-</span>
        </div>

        <div class="section">
          <b>Angle (Roll/Pitch)</b>
          <div class="row">
            Kp <input id="aKp" type="number" step="0.001">
            Ki <input id="aKi" type="number" step="0.001">
            Kd <input id="aKd" type="number" step="0.001">
            <button id="applyAngle">Apply</button>
          </div>
        </div>

        <div class="section">
          <b>Rate (Roll/Pitch)</b>
          <div class="row">
            Kp <input id="rKp" type="number" step="0.0001">
            Ki <input id="rKi" type="number" step="0.0001">
            Kd <input id="rKd" type="number" step="0.0001">
            <button id="applyRate">Apply</button>
          </div>
        </div>

        <div class="section">
          <b>Yaw Rate</b>
          <div class="row">
            Kp <input id="yKp" type="number" step="0.0001">
            Ki <input id="yKi" type="number" step="0.0001">
            Kd <input id="yKd" type="number" step="0.0001">
            <button id="applyYaw">Apply</button>
          </div>
        </div>

        <div class="section">
          <b>Limits</b>
          <div class="row">
            Max angle <input id="maxAngle" type="number" step="1">
            Max yaw <input id="maxYawRate" type="number" step="1">
          </div>
          <div class="row">
            Tilt disarm <input id="tiltDisarm" type="number" step="1">
            Cmd timeout <input id="cmdTimeout" type="number" step="10">
          </div>
          <div class="row">
            Telem ms <input id="telemMs" type="number" step="10">
            <button id="applyLimits">Apply Limits</button>
          </div>
        </div>
      </div>
    </div>
  </div>

<script>
(() => {
  const wsLabel = document.getElementById('ws');
  const stateLabel = document.getElementById('state');
  const attLabel = document.getElementById('att');
  const dtHzLabel = document.getElementById('dtHz');
  const cmdAgeLabel = document.getElementById('cmdAge');
  const motLabel = document.getElementById('mot');
  const vbattLabel = document.getElementById('vbatt');
  const fsLabel = document.getElementById('fs');

  const attNums = document.getElementById('attNums');
  const attRollLabel = document.getElementById('attRoll');
  const attPitchLabel = document.getElementById('attPitch');
  const attYawLabel = document.getElementById('attYaw');
  const attCanvas = document.getElementById('attCanvas');
  const attCtx = attCanvas ? attCanvas.getContext('2d') : null;
  const levelBtn = document.getElementById('levelBtn');
  const gyroCalBtn = document.getElementById('gyroCalBtn');
  const calState = document.getElementById('calState');
  const actionStatus = document.getElementById('actionStatus');
  const statusMsg = document.getElementById('statusMsg');

  const modeControl = document.getElementById('modeControl');
  const modeSettings = document.getElementById('modeSettings');

  const armSwitch = document.getElementById('armSwitch');
  const disarmBtn = document.getElementById('disarmBtn');
  const thv = document.getElementById('thv');

  const syncBtn = document.getElementById('syncBtn');
  const saveBtn = document.getElementById('saveBtn');

  const aKp = document.getElementById('aKp');
  const aKi = document.getElementById('aKi');
  const aKd = document.getElementById('aKd');
  const rKp = document.getElementById('rKp');
  const rKi = document.getElementById('rKi');
  const rKd = document.getElementById('rKd');
  const yKp = document.getElementById('yKp');
  const yKi = document.getElementById('yKi');
  const yKd = document.getElementById('yKd');

  const applyAngle = document.getElementById('applyAngle');
  const applyRate = document.getElementById('applyRate');
  const applyYaw = document.getElementById('applyYaw');

  const maxAngle = document.getElementById('maxAngle');
  const maxYawRate = document.getElementById('maxYawRate');
  const tiltDisarm = document.getElementById('tiltDisarm');
  const cmdTimeout = document.getElementById('cmdTimeout');
  const telemMs = document.getElementById('telemMs');
  const applyLimits = document.getElementById('applyLimits');

  const leftJoy = document.getElementById('leftJoy');
  const leftStick = document.getElementById('leftStick');
  const rightJoy = document.getElementById('rightJoy');
  const rightStick = document.getElementById('rightStick');

  let ws = null;

  let roll = 0;
  let pitch = 0;
  let yaw = 0;
  let thr = 0;      // 0..1 (left stick Y)
  let armReq = 0;

  let lastAtt = { roll: 0, pitch: 0, yaw: 0 };
  let statusTimer = null;

  const fsMap = {
    0: 'NONE',
    1: 'CMD_TIMEOUT',
    2: 'WS_DISCONNECT',
    3: 'IMU_FAIL',
    4: 'TILT_LIMIT',
    5: 'DT_LIMIT',
    6: 'KILL',
    7: 'MANUAL',
    8: 'MOTOR_FAIL'
  };

  function clamp(v, vmin, vmax) {
    return Math.max(vmin, Math.min(vmax, v));
  }

  function setStatus(text, holdMs = 2000) {
    if (!actionStatus) return;
    actionStatus.textContent = text;
    if (statusTimer) {
      clearTimeout(statusTimer);
      statusTimer = null;
    }
    if (holdMs > 0) {
      statusTimer = setTimeout(() => {
        actionStatus.textContent = '-';
        statusTimer = null;
      }, holdMs);
    }
  }

  function joyMaxR(areaEl) {
    return Math.min(areaEl.clientWidth, areaEl.clientHeight) * 0.35;
  }

  function setStickNorm(areaEl, stickEl, nx, ny) {
    const mr = joyMaxR(areaEl);
    stickEl.style.transform =
      `translate(calc(-50% + ${nx * mr}px), calc(-50% + ${ny * mr}px))`;
  }

  function nyToThrottle(ny) {
    return clamp((1 - ny) / 2, 0, 1);
  }

  function throttleToNy(t) {
    const tt = clamp(t, 0, 1);
    return 1 - 2 * tt;
  }

  function updateThrottleUi() {
    if (thv) thv.textContent = thr.toFixed(2);
  }

  function setMode(mode) {
    document.body.dataset.mode = mode;
    if (modeControl) modeControl.classList.toggle('active', mode === 'control');
    if (modeSettings) modeSettings.classList.toggle('active', mode === 'settings');
    if (mode === 'settings') {
      armReq = 0;
      if (armSwitch) armSwitch.checked = false;
      thr = 0;
      yaw = 0;
      roll = 0;
      pitch = 0;
      updateThrottleUi();
      if (leftJoy && leftStick) {
        setStickNorm(leftJoy, leftStick, 0, throttleToNy(thr));
      }
      if (rightJoy && rightStick) {
        setStickNorm(rightJoy, rightStick, 0, 0);
      }
      requestAnimationFrame(() => {
        drawAttitude(lastAtt.roll, lastAtt.pitch, lastAtt.yaw);
      });
    }
  }

  function resizeAttCanvas() {
    if (!attCanvas || !attCtx) return;
    const rect = attCanvas.getBoundingClientRect();
    if (rect.width < 10 || rect.height < 10) return;
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(rect.width * dpr));
    const h = Math.max(1, Math.round(rect.height * dpr));
    if (attCanvas.width !== w || attCanvas.height !== h) {
      attCanvas.width = w;
      attCanvas.height = h;
    }
  }

  function drawAttitude(rollDeg, pitchDeg, yawDeg) {
    lastAtt = { roll: rollDeg, pitch: pitchDeg, yaw: yawDeg };
    if (!attCanvas || !attCtx) return;
    const rect = attCanvas.getBoundingClientRect();
    if (rect.width < 10 || rect.height < 10) return;
    resizeAttCanvas();

    const w = attCanvas.width;
    const h = attCanvas.height;
    const dpr = window.devicePixelRatio || 1;
    const ctx = attCtx;
    ctx.clearRect(0, 0, w, h);

    const cx = w / 2;
    const cy = h / 2;
    const radius = Math.min(w, h) * 0.45;
    const rollRad = rollDeg * Math.PI / 180;
    const pitch = clamp(pitchDeg, -45, 45);
    const offset = (pitch / 45) * radius;

    ctx.save();
    ctx.translate(cx, cy);
    ctx.beginPath();
    ctx.arc(0, 0, radius, 0, Math.PI * 2);
    ctx.clip();
    ctx.rotate(-rollRad);
    ctx.fillStyle = '#cfe8ff';
    ctx.fillRect(-radius * 2, -radius * 2 + offset, radius * 4, radius * 2);
    ctx.fillStyle = '#d2a679';
    ctx.fillRect(-radius * 2, offset, radius * 4, radius * 2);
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    ctx.moveTo(-radius * 2, offset);
    ctx.lineTo(radius * 2, offset);
    ctx.stroke();
    ctx.restore();

    ctx.strokeStyle = '#333';
    ctx.lineWidth = 2 * dpr;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, 0, Math.PI * 2);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(cx - 10 * dpr, cy);
    ctx.lineTo(cx + 10 * dpr, cy);
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(cx, cy - 10 * dpr);
    ctx.lineTo(cx, cy + 10 * dpr);
    ctx.stroke();

    ctx.save();
    ctx.translate(cx, cy);
    ctx.rotate(-yawDeg * Math.PI / 180);
    ctx.fillStyle = '#333';
    ctx.beginPath();
    ctx.moveTo(0, -radius);
    ctx.lineTo(-6 * dpr, -radius + 12 * dpr);
    ctx.lineTo(6 * dpr, -radius + 12 * dpr);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }

  function connectWs() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => {
      wsLabel.textContent = 'OPEN';
      if (ws && ws.readyState === 1) ws.send('GET');
    };
    ws.onclose = () => { wsLabel.textContent = 'CLOSED'; setTimeout(connectWs, 500); };
    ws.onerror = () => { wsLabel.textContent = 'ERR'; };
    ws.onmessage = (ev) => {
      const s = String(ev.data || '');
      try {
        const obj = JSON.parse(s);
        if (obj.type === 'tel') {
          const rollDeg = Number(obj.roll);
          const pitchDeg = Number(obj.pitch);
          const yawDeg = Number(obj.yaw);
          const rollTxt = Number.isFinite(rollDeg) ? rollDeg.toFixed(1) : '-';
          const pitchTxt = Number.isFinite(pitchDeg) ? pitchDeg.toFixed(1) : '-';
          const yawTxt = Number.isFinite(yawDeg) ? yawDeg.toFixed(1) : '-';

          stateLabel.textContent = obj.armed ? 'ARMED' : 'DISARMED';
          attLabel.textContent = `${rollTxt}, ${pitchTxt}, ${yawTxt}`;
          if (attNums) attNums.textContent = `R ${rollTxt} P ${pitchTxt} Y ${yawTxt}`;
          if (attRollLabel) attRollLabel.textContent = rollTxt;
          if (attPitchLabel) attPitchLabel.textContent = pitchTxt;
          if (attYawLabel) attYawLabel.textContent = yawTxt;
          if (calState && obj.cal) calState.textContent = `CAL: ${obj.cal}`;

          dtHzLabel.textContent = `${obj.dt_ms.toFixed(2)} ms / ${obj.loop_hz.toFixed(0)} Hz`;
          cmdAgeLabel.textContent = `${obj.cmd_age} ms`;
          motLabel.textContent = `${obj.m0.toFixed(2)} ${obj.m1.toFixed(2)} ${obj.m2.toFixed(2)} ${obj.m3.toFixed(2)}`;
          vbattLabel.textContent = (Number.isFinite(obj.vbatt) ? `${obj.vbatt.toFixed(2)} V` : '-');
          fsLabel.textContent = fsMap[obj.fs] || String(obj.fs);
          if (statusMsg) statusMsg.textContent = obj.msg ? String(obj.msg) : '-';

          if (Number.isFinite(rollDeg) && Number.isFinite(pitchDeg) && Number.isFinite(yawDeg)) {
            drawAttitude(rollDeg, pitchDeg, yawDeg);
          }
        } else if (obj.type === 'ack') {
          const op = String(obj.op || '');
          const tag = obj.tag ? String(obj.tag) : '';
          const ok = obj.ok ? true : false;
          const msg = obj.msg ? String(obj.msg) : '';
          let text = op || 'ACK';
          if (tag) text += ` ${tag}`;
          text += ok ? ' OK' : ' FAIL';
          if (msg) text += ` (${msg})`;
          setStatus(text, ok ? 2000 : 3000);
        } else if (obj.type === 'cfg') {
          if (aKp) aKp.value = obj.angle.kp;
          if (aKi) aKi.value = obj.angle.ki;
          if (aKd) aKd.value = obj.angle.kd;
          if (rKp) rKp.value = obj.rate.kp;
          if (rKi) rKi.value = obj.rate.ki;
          if (rKd) rKd.value = obj.rate.kd;
          if (yKp) yKp.value = obj.yaw.kp;
          if (yKi) yKi.value = obj.yaw.ki;
          if (yKd) yKd.value = obj.yaw.kd;
          if (obj.limits) {
            if (maxAngle) maxAngle.value = obj.limits.max_angle;
            if (maxYawRate) maxYawRate.value = obj.limits.max_yaw_rate;
            if (tiltDisarm) tiltDisarm.value = obj.limits.tilt_disarm;
            if (cmdTimeout) cmdTimeout.value = obj.limits.cmd_timeout;
            if (telemMs) telemMs.value = obj.limits.telem_ms;
          }
          setStatus('CFG updated', 2000);
        }
      } catch (e) {
        // ignore
      }
    };
  }

  if (modeControl) {
    modeControl.addEventListener('click', () => setMode('control'));
  }
  if (modeSettings) {
    modeSettings.addEventListener('click', () => setMode('settings'));
  }

  if (armSwitch) {
    armSwitch.addEventListener('change', () => {
      armReq = armSwitch.checked ? 1 : 0;
    });
  }

  if (disarmBtn) {
    disarmBtn.addEventListener('click', () => {
      armReq = 0;
      if (armSwitch) armSwitch.checked = false;

      thr = 0;
      yaw = 0;
      roll = 0;
      pitch = 0;

      updateThrottleUi();
      if (leftJoy && leftStick) {
        setStickNorm(leftJoy, leftStick, 0, throttleToNy(thr));
      }
      if (rightJoy && rightStick) {
        setStickNorm(rightJoy, rightStick, 0, 0);
      }

      if (ws && ws.readyState === 1) {
        ws.send('K');
      }
    });
  }

  function sendWs(text, statusText) {
    if (!ws || ws.readyState !== 1) {
      setStatus('WS not open', 2000);
      return false;
    }
    if (statusText) {
      setStatus(statusText, 1500);
    }
    ws.send(text);
    return true;
  }

  function sendPid(tag, kp, ki, kd) {
    if (!ws || ws.readyState !== 1) return;
    ws.send(`PID,${tag},${kp},${ki},${kd}`);
  }

  if (applyAngle) {
    applyAngle.addEventListener('click', () => {
      sendPid('ANGLE', Number(aKp.value), Number(aKi.value), Number(aKd.value));
    });
  }
  if (applyRate) {
    applyRate.addEventListener('click', () => {
      sendPid('RATE', Number(rKp.value), Number(rKi.value), Number(rKd.value));
    });
  }
  if (applyYaw) {
    applyYaw.addEventListener('click', () => {
      sendPid('YAW', Number(yKp.value), Number(yKi.value), Number(yKd.value));
    });
  }
  if (syncBtn) {
    syncBtn.addEventListener('click', () => {
      sendWs('GET', 'GET sent');
    });
  }
  if (saveBtn) {
    saveBtn.addEventListener('click', () => {
      sendWs('SAVE', 'SAVE sent');
    });
  }

  if (levelBtn) {
    levelBtn.addEventListener('click', () => {
      sendWs('CAL,LEVEL', 'CAL LEVEL sent');
    });
  }
  if (gyroCalBtn) {
    gyroCalBtn.addEventListener('click', () => {
      sendWs('CAL,GYRO', 'CAL GYRO sent');
    });
  }

  function sendCfg(key, value) {
    if (!ws || ws.readyState !== 1) return;
    ws.send(`CFG,${key},${value}`);
  }

  function readNumberInput(input) {
    if (!input) return null;
    const s = String(input.value || '').trim();
    if (s === '') return null;
    const v = Number(s);
    return Number.isFinite(v) ? v : null;
  }

  if (applyLimits) {
    applyLimits.addEventListener('click', () => {
      const maxAngleVal = readNumberInput(maxAngle);
      const maxYawVal = readNumberInput(maxYawRate);
      const tiltVal = readNumberInput(tiltDisarm);
      const cmdVal = readNumberInput(cmdTimeout);
      const telemVal = readNumberInput(telemMs);

      if (maxAngleVal !== null) sendCfg('MAX_ANGLE', maxAngleVal);
      if (maxYawVal !== null) sendCfg('MAX_YAW_RATE', maxYawVal);
      if (tiltVal !== null) sendCfg('TILT_DISARM', tiltVal);
      if (cmdVal !== null) sendCfg('CMD_TIMEOUT', Math.round(cmdVal));
      if (telemVal !== null) sendCfg('TELEM_MS', Math.round(telemVal));
    });
  }

  function makeJoystick(areaEl, stickEl, cfg) {
    const maxR = () => joyMaxR(areaEl);
    let activeId = null;
    let origin = null;
    let curNx = cfg.initialNx || 0;
    let curNy = cfg.initialNy || 0;

    function setStick(nx, ny) {
      curNx = nx;
      curNy = ny;
      setStickNorm(areaEl, stickEl, nx, ny);
    }

    requestAnimationFrame(() => {
      setStick(curNx, curNy);
    });

    areaEl.addEventListener('pointerdown', (e) => {
      activeId = e.pointerId;
      areaEl.setPointerCapture(activeId);

      const rect = areaEl.getBoundingClientRect();
      origin = { x: rect.left + rect.width / 2, y: rect.top + rect.height / 2 };

      const mr = maxR();
      const nx = clamp((e.clientX - origin.x) / mr, -1, 1);
      const ny = clamp((e.clientY - origin.y) / mr, -1, 1);
      setStick(nx, ny);
      cfg.onMove(nx, ny);
    });

    areaEl.addEventListener('pointermove', (e) => {
      if (activeId === null || e.pointerId !== activeId || !origin) return;
      const mr = maxR();
      const nx = clamp((e.clientX - origin.x) / mr, -1, 1);
      const ny = clamp((e.clientY - origin.y) / mr, -1, 1);
      setStick(nx, ny);
      cfg.onMove(nx, ny);
    });

    function release(e) {
      if (activeId === null || e.pointerId !== activeId) return;
      activeId = null;
      origin = null;

      const nx = cfg.centerX ? 0 : curNx;
      const ny = cfg.centerY ? 0 : curNy;
      setStick(nx, ny);

      if (cfg.onRelease) {
        cfg.onRelease(nx, ny);
      }
    }

    areaEl.addEventListener('pointerup', release);
    areaEl.addEventListener('pointercancel', release);
  }

  // Left stick: X=Yaw (spring), Y=Throttle (sticky), bottom=0 top=1
  makeJoystick(leftJoy, leftStick, {
    centerX: true,
    centerY: false,
    initialNx: 0,
    initialNy: 1,
    onMove: (nx, ny) => {
      yaw = clamp(nx, -1, 1);
      thr = nyToThrottle(ny);
      updateThrottleUi();
    },
    onRelease: (nx, ny) => {
      yaw = 0;
      thr = nyToThrottle(ny);
      updateThrottleUi();
    }
  });

  // Right stick: X=Roll, Y=Pitch (spring both)
  makeJoystick(rightJoy, rightStick, {
    centerX: true,
    centerY: true,
    initialNx: 0,
    initialNy: 0,
    onMove: (nx, ny) => {
      roll = clamp(nx, -1, 1);
      pitch = clamp(-ny, -1, 1);
    },
    onRelease: () => {
      roll = 0;
      pitch = 0;
    }
  });

  setInterval(() => {
    if (!ws || ws.readyState !== 1) return;
    ws.send(`C,${thr.toFixed(3)},${roll.toFixed(3)},${pitch.toFixed(3)},${yaw.toFixed(3)},${armReq}`);
  }, 20);

  window.addEventListener('resize', () => {
    resizeAttCanvas();
    drawAttitude(lastAtt.roll, lastAtt.pitch, lastAtt.yaw);
  });

  thr = 0;
  updateThrottleUi();
  setMode('control');
  connectWs();
})();
</script>
</body>
</html>
)HTML";
