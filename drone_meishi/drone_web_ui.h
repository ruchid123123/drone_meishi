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

    #app {
      height: 100%;
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

    /* PIDは今回は「1画面」を優先して隠しています。
       使いたくなったら display:none を消してください。 */
    #pidDetails { display: none; }

    @media (orientation:landscape) {
      :root{
        /* iPhone横持ちを想定して、中央パネルの幅を確保できるサイズ */
        --pad-size: min(60vh, 32vw, 260px);
        --center-width: min(360px, 42vw, 380px);
        --card-pad: 6px;
      }

      #app {
        height: 100vh;
        height: 100svh;
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
<body>
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

      <details class="card" id="pidDetails">
        <summary><b>PID tuning</b> (Applyで送信)</summary>
        <div style="margin-top:10px;">
          <div class="row">
            <button id="syncBtn">GET</button>
            <button id="saveBtn">SAVE</button>
            <small>GET=機体の現在値 / SAVE=保存</small>
          </div>

          <div style="margin-top:12px;">
            <b>Angle (Roll/Pitch)</b>
            <div class="row">
              Kp <input id="aKp" type="number" step="0.001">
              Ki <input id="aKi" type="number" step="0.001">
              Kd <input id="aKd" type="number" step="0.001">
              <button id="applyAngle">Apply</button>
            </div>
          </div>

          <div style="margin-top:12px;">
            <b>Rate (Roll/Pitch)</b>
            <div class="row">
              Kp <input id="rKp" type="number" step="0.0001">
              Ki <input id="rKi" type="number" step="0.0001">
              Kd <input id="rKd" type="number" step="0.0001">
              <button id="applyRate">Apply</button>
            </div>
          </div>

          <div style="margin-top:12px;">
            <b>Yaw Rate</b>
            <div class="row">
              Kp <input id="yKp" type="number" step="0.0001">
              Ki <input id="yKi" type="number" step="0.0001">
              Kd <input id="yKd" type="number" step="0.0001">
              <button id="applyYaw">Apply</button>
            </div>
          </div>
        </div>
      </details>
    </div>

    <div class="padCol">
      <div class="joy" id="rightJoy">
        <div class="joyLabel">Roll / Pitch</div>
        <div class="stick" id="rightStick"></div>
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

  const fsMap = {
    0: 'NONE',
    1: 'CMD_TIMEOUT',
    2: 'WS_DISCONNECT',
    3: 'IMU_FAIL',
    4: 'TILT_LIMIT',
    5: 'DT_LIMIT',
    6: 'KILL',
    7: 'MANUAL'
  };

  function clamp(v, vmin, vmax) {
    return Math.max(vmin, Math.min(vmax, v));
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
    thv.textContent = thr.toFixed(2);
  }

  function connectWs() {
    ws = new WebSocket(`ws://${location.host}/ws`);
    ws.onopen = () => { wsLabel.textContent = 'OPEN'; };
    ws.onclose = () => { wsLabel.textContent = 'CLOSED'; setTimeout(connectWs, 500); };
    ws.onerror = () => { wsLabel.textContent = 'ERR'; };
    ws.onmessage = (ev) => {
      const s = String(ev.data || '');
      try {
        const obj = JSON.parse(s);
        if (obj.type === 'tel') {
          stateLabel.textContent = obj.armed ? 'ARMED' : 'DISARMED';
          attLabel.textContent = `${obj.roll.toFixed(1)}, ${obj.pitch.toFixed(1)}, ${obj.yaw.toFixed(1)}`;
          dtHzLabel.textContent = `${obj.dt_ms.toFixed(2)} ms / ${obj.loop_hz.toFixed(0)} Hz`;
          cmdAgeLabel.textContent = `${obj.cmd_age} ms`;
          motLabel.textContent = `${obj.m0.toFixed(2)} ${obj.m1.toFixed(2)} ${obj.m2.toFixed(2)} ${obj.m3.toFixed(2)}`;
          vbattLabel.textContent = (Number.isFinite(obj.vbatt) ? `${obj.vbatt.toFixed(2)} V` : '-');
          fsLabel.textContent = fsMap[obj.fs] || String(obj.fs);
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
        }
      } catch (e) {
        // ignore
      }
    };
  }

  armSwitch.addEventListener('change', () => {
    armReq = armSwitch.checked ? 1 : 0;
  });

  disarmBtn.addEventListener('click', () => {
    armReq = 0;
    armSwitch.checked = false;

    thr = 0;
    yaw = 0;
    roll = 0;
    pitch = 0;

    updateThrottleUi();
    setStickNorm(leftJoy, leftStick, 0, throttleToNy(thr));
    setStickNorm(rightJoy, rightStick, 0, 0);

    if (ws && ws.readyState === 1) {
      ws.send('K');
    }
  });

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
      if (ws && ws.readyState === 1) ws.send('GET');
    });
  }
  if (saveBtn) {
    saveBtn.addEventListener('click', () => {
      if (ws && ws.readyState === 1) ws.send('SAVE');
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

  thr = 0;
  updateThrottleUi();
  connectWs();
})();
</script>
</body>
</html>
)HTML";


