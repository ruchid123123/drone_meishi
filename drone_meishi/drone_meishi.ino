#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>
#include "drone_types.h"

// ============================================================
// Pins / config (基板に合わせて変更)
// ============================================================

// WiFi AP
static const char *kApSsid = "ESP32-DRONE";
static const char *kApPass = "12345678";

// I2C
static const int kI2cSda = 21;
static const int kI2cScl = 22;
static const uint8_t kMpuAddr = 0x68;

// Motors (X configuration example)
static const int kMotor0Pin = 12;  // Front Left
static const int kMotor1Pin = 13;  // Front Right
static const int kMotor2Pin = 14;  // Rear Right
static const int kMotor3Pin = 15;  // Rear Left

// PWM
static const uint32_t kPwmFreqHz = 20000;
static const uint8_t kPwmResolutionBits = 11;
static const uint32_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;

static const int kMotor0Ch = 0;
static const int kMotor1Ch = 1;
static const int kMotor2Ch = 2;
static const int kMotor3Ch = 3;

// Control loop
static const uint32_t kLoopPeriodUs = 2000;  // 500Hz
static const float kDtMaxSec = 0.02f;

// RC and arming
static const uint32_t kCmdTimeoutMs = 300;
static const float kArmThrottleMax = 0.05f;
static const float kStickCenterMax = 0.15f;
static const uint32_t kArmHoldMs = 800;

// Limits
static const float kMaxAngleDeg = 30.0f;
static const float kMaxYawRateDps = 180.0f;
static const float kTiltDisarmDeg = 80.0f;

// Telemetry
static const uint32_t kTelemPeriodMs = 50;  // 20Hz

// IMU filter
static const float kMadgwickBeta = 0.08f;

// Battery telemetry (任意)
// 使わないなら -1 のままでOK
static const int kVbattAdcPin = -1;         // 例: 34
static const float kVbattDividerRatio = 2.0f;  // 分圧比 (VBAT = Vadc * ratio)

// ============================================================
// Utilities
// ============================================================

static inline float clampf(float v, float vmin, float vmax) {
  if (v < vmin) {
    return vmin;
  }
  if (v > vmax) {
    return vmax;
  }
  return v;
}

static inline bool isFinitef(float v) {
  return isfinite(v) != 0;
}

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
        --pad-size: min(72vh, 36vw, 300px);
        --center-width: min(320px, 38vw, 340px);
        --card-pad: 8px;
      }

      #app {
        flex-direction: row;
        align-items: stretch;
        justify-content: center;
      }

      .padCol { width: auto; }

      #centerCol {
        height: var(--pad-size);
      }

      #telemetryCard .kv {
        grid-template-columns: 90px 1fr;
        font-size: 14px;
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


// ============================================================
// RC command
// ============================================================

static RcCommand g_rc = {0.0f, 0.0f, 0.0f, 0.0f, false, 0};
static portMUX_TYPE g_rcMux = portMUX_INITIALIZER_UNLOCKED;

static RcCommand getRcCommand() {
  RcCommand out;
  portENTER_CRITICAL(&g_rcMux);
  out = g_rc;
  portEXIT_CRITICAL(&g_rcMux);
  return out;
}

static void setRcCommand(float throttle, float roll, float pitch, float yaw, bool arm_request) {
  portENTER_CRITICAL(&g_rcMux);
  g_rc.throttle = clampf(throttle, 0.0f, 1.0f);
  g_rc.roll = clampf(roll, -1.0f, 1.0f);
  g_rc.pitch = clampf(pitch, -1.0f, 1.0f);
  g_rc.yaw = clampf(yaw, -1.0f, 1.0f);
  g_rc.arm_request = arm_request;
  g_rc.last_ms = millis();
  portEXIT_CRITICAL(&g_rcMux);
}

// ============================================================
// PID
// ============================================================

class Pid {
 public:
  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  void setOutputLimits(float out_min, float out_max) {
    out_min_ = out_min;
    out_max_ = out_max;
  }

  void reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_ = false;
  }

  float update(float error, float dt) {
    if (dt <= 0.0f) {
      return 0.0f;
    }

    const float p = kp_ * error;

    integral_ += error * dt;
    const float i = ki_ * integral_;

    float d = 0.0f;
    if (has_prev_) {
      d = kd_ * (error - prev_error_) / dt;
    }
    prev_error_ = error;
    has_prev_ = true;

    float out = p + i + d;
    out = clampf(out, out_min_, out_max_);

    if (out == out_min_ || out == out_max_) {
      integral_ *= 0.99f;
    }
    return out;
  }

 private:
  float kp_ = 0.0f;
  float ki_ = 0.0f;
  float kd_ = 0.0f;
  float out_min_ = -1.0f;
  float out_max_ = 1.0f;
  float integral_ = 0.0f;
  float prev_error_ = 0.0f;
  bool has_prev_ = false;
};

// ============================================================
// Madgwick IMU-only
// ============================================================

class MadgwickImu {
 public:
  void begin(float beta) {
    beta_ = beta;
    q0_ = 1.0f;
    q1_ = 0.0f;
    q2_ = 0.0f;
    q3_ = 0.0f;
  }

  void update(float gx_rads, float gy_rads, float gz_rads,
              float ax, float ay, float az,
              float dt) {
    float q0 = q0_;
    float q1 = q1_;
    float q2 = q2_;
    float q3 = q3_;

    const float norm_a = sqrtf(ax * ax + ay * ay + az * az);
    if (norm_a < 1e-6f) {
      integrateGyroOnly(gx_rads, gy_rads, gz_rads, dt);
      return;
    }
    ax /= norm_a;
    ay /= norm_a;
    az /= norm_a;

    const float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    const float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    const float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    const float j_11or24 = -2.0f * q2;
    const float j_12or23 = 2.0f * q3;
    const float j_13or22 = -2.0f * q0;
    const float j_14or21 = 2.0f * q1;
    const float j_32 = 2.0f * j_14or21;
    const float j_33 = 2.0f * j_11or24;

    float s0 = j_14or21 * f2 - j_11or24 * f1;
    float s1 = j_12or23 * f1 + j_13or22 * f2 - j_32 * f3;
    float s2 = j_12or23 * f2 - j_33 * f3 - j_13or22 * f1;
    float s3 = j_14or21 * f1 + j_11or24 * f2;

    const float norm_s = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    if (norm_s > 1e-6f) {
      s0 /= norm_s;
      s1 /= norm_s;
      s2 /= norm_s;
      s3 /= norm_s;
    }

    const float qDot0 = 0.5f * (-q1 * gx_rads - q2 * gy_rads - q3 * gz_rads) - beta_ * s0;
    const float qDot1 = 0.5f * ( q0 * gx_rads + q2 * gz_rads - q3 * gy_rads) - beta_ * s1;
    const float qDot2 = 0.5f * ( q0 * gy_rads - q1 * gz_rads + q3 * gx_rads) - beta_ * s2;
    const float qDot3 = 0.5f * ( q0 * gz_rads + q1 * gy_rads - q2 * gx_rads) - beta_ * s3;

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    const float norm_q = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm_q > 1e-6f) {
      q0 /= norm_q;
      q1 /= norm_q;
      q2 /= norm_q;
      q3 /= norm_q;
    }

    q0_ = q0;
    q1_ = q1;
    q2_ = q2;
    q3_ = q3;
  }

  void getEulerDeg(float *roll_deg, float *pitch_deg, float *yaw_deg) const {
    const float q0 = q0_;
    const float q1 = q1_;
    const float q2 = q2_;
    const float q3 = q3_;

    const float roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                              1.0f - 2.0f * (q1 * q1 + q2 * q2));
    const float sinp = 2.0f * (q0 * q2 - q3 * q1);
    float pitch = 0.0f;
    if (fabsf(sinp) >= 1.0f) {
      pitch = copysignf((float)M_PI / 2.0f, sinp);
    } else {
      pitch = asinf(sinp);
    }
    const float yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                             1.0f - 2.0f * (q2 * q2 + q3 * q3));

    *roll_deg = roll * 180.0f / (float)M_PI;
    *pitch_deg = pitch * 180.0f / (float)M_PI;
    *yaw_deg = yaw * 180.0f / (float)M_PI;
  }

 private:
  void integrateGyroOnly(float gx_rads, float gy_rads, float gz_rads, float dt) {
    float q0 = q0_;
    float q1 = q1_;
    float q2 = q2_;
    float q3 = q3_;

    const float qDot0 = 0.5f * (-q1 * gx_rads - q2 * gy_rads - q3 * gz_rads);
    const float qDot1 = 0.5f * ( q0 * gx_rads + q2 * gz_rads - q3 * gy_rads);
    const float qDot2 = 0.5f * ( q0 * gy_rads - q1 * gz_rads + q3 * gx_rads);
    const float qDot3 = 0.5f * ( q0 * gz_rads + q1 * gy_rads - q2 * gx_rads);

    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    const float norm_q = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    if (norm_q > 1e-6f) {
      q0 /= norm_q;
      q1 /= norm_q;
      q2 /= norm_q;
      q3 /= norm_q;
    }

    q0_ = q0;
    q1_ = q1;
    q2_ = q2;
    q3_ = q3;
  }

  float beta_ = 0.08f;
  float q0_ = 1.0f;
  float q1_ = 0.0f;
  float q2_ = 0.0f;
  float q3_ = 0.0f;
};

// ============================================================
// MPU6050 minimal
// ============================================================

struct ImuSample {
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
};

class Mpu6050 {
 public:
  bool begin() {
    Wire.begin(kI2cSda, kI2cScl, 400000);

    if (!writeReg(0x6B, 0x00)) {
      return false;
    }
    if (!writeReg(0x1A, 0x03)) {
      return false;
    }
    if (!writeReg(0x1B, 0x18)) {
      return false;
    }
    if (!writeReg(0x1C, 0x10)) {
      return false;
    }
    delay(50);
    return true;
  }

  void calibrateGyro(uint16_t samples, uint16_t delay_ms = 2) {
    float sumx = 0.0f;
    float sumy = 0.0f;
    float sumz = 0.0f;

    for (uint16_t i = 0; i < samples; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      if (readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
        const float scale = 2000.0f / 32768.0f;
        sumx += gx * scale;
        sumy += gy * scale;
        sumz += gz * scale;
      }
      delay(delay_ms);
    }

    gyro_bias_x_ = sumx / (float)samples;
    gyro_bias_y_ = sumy / (float)samples;
    gyro_bias_z_ = sumz / (float)samples;
  }

  bool read(ImuSample *out) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
      return false;
    }

    const float a_scale = 8.0f / 32768.0f;
    const float g_scale = 2000.0f / 32768.0f;

    out->ax_g = ax * a_scale;
    out->ay_g = ay * a_scale;
    out->az_g = az * a_scale;

    out->gx_dps = gx * g_scale - gyro_bias_x_;
    out->gy_dps = gy * g_scale - gyro_bias_y_;
    out->gz_dps = gz * g_scale - gyro_bias_z_;
    return true;
  }

 private:
  bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(kMpuAddr);
    Wire.write(reg);
    Wire.write(val);
    return Wire.endTransmission(true) == 0;
  }

  bool readRaw(int16_t *ax, int16_t *ay, int16_t *az,
               int16_t *gx, int16_t *gy, int16_t *gz) {
    Wire.beginTransmission(kMpuAddr);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0) {
      return false;
    }

    const uint8_t n = Wire.requestFrom((int)kMpuAddr, 14, (int)true);
    if (n != 14) {
      return false;
    }

    *ax = (int16_t)((Wire.read() << 8) | Wire.read());
    *ay = (int16_t)((Wire.read() << 8) | Wire.read());
    *az = (int16_t)((Wire.read() << 8) | Wire.read());
    (void)Wire.read(); (void)Wire.read();
    *gx = (int16_t)((Wire.read() << 8) | Wire.read());
    *gy = (int16_t)((Wire.read() << 8) | Wire.read());
    *gz = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
  }

  float gyro_bias_x_ = 0.0f;
  float gyro_bias_y_ = 0.0f;
  float gyro_bias_z_ = 0.0f;
};

// ============================================================
// PID tunings + NVS
// ============================================================

static Tunings g_tunings = {
  {4.0f, 0.0f, 0.0f},       // angle
  {0.010f, 0.0f, 0.0002f},  // rate
  {0.010f, 0.0f, 0.0f},     // yaw
};

static volatile bool g_tunings_dirty = false;
static portMUX_TYPE g_cfgMux = portMUX_INITIALIZER_UNLOCKED;

static Preferences g_prefs;

static void loadTunings() {
  g_prefs.begin("drone", true);

  g_tunings.angle.kp = g_prefs.getFloat("a_kp", g_tunings.angle.kp);
  g_tunings.angle.ki = g_prefs.getFloat("a_ki", g_tunings.angle.ki);
  g_tunings.angle.kd = g_prefs.getFloat("a_kd", g_tunings.angle.kd);

  g_tunings.rate.kp = g_prefs.getFloat("r_kp", g_tunings.rate.kp);
  g_tunings.rate.ki = g_prefs.getFloat("r_ki", g_tunings.rate.ki);
  g_tunings.rate.kd = g_prefs.getFloat("r_kd", g_tunings.rate.kd);

  g_tunings.yaw.kp = g_prefs.getFloat("y_kp", g_tunings.yaw.kp);
  g_tunings.yaw.ki = g_prefs.getFloat("y_ki", g_tunings.yaw.ki);
  g_tunings.yaw.kd = g_prefs.getFloat("y_kd", g_tunings.yaw.kd);

  g_prefs.end();
}

static void saveTunings() {
  g_prefs.begin("drone", false);

  g_prefs.putFloat("a_kp", g_tunings.angle.kp);
  g_prefs.putFloat("a_ki", g_tunings.angle.ki);
  g_prefs.putFloat("a_kd", g_tunings.angle.kd);

  g_prefs.putFloat("r_kp", g_tunings.rate.kp);
  g_prefs.putFloat("r_ki", g_tunings.rate.ki);
  g_prefs.putFloat("r_kd", g_tunings.rate.kd);

  g_prefs.putFloat("y_kp", g_tunings.yaw.kp);
  g_prefs.putFloat("y_ki", g_tunings.yaw.ki);
  g_prefs.putFloat("y_kd", g_tunings.yaw.kd);

  g_prefs.end();
}

// ============================================================
// Globals
// ============================================================

static AsyncWebServer g_server(80);
static AsyncWebSocket g_ws("/ws");

static Mpu6050 g_mpu;
static MadgwickImu g_ahrs;

static Pid g_roll_angle_pid;
static Pid g_pitch_angle_pid;
static Pid g_roll_rate_pid;
static Pid g_pitch_rate_pid;
static Pid g_yaw_rate_pid;

static float g_roll_deg = 0.0f;
static float g_pitch_deg = 0.0f;
static float g_yaw_deg = 0.0f;

static float g_motor_last[4] = {0, 0, 0, 0};

static bool g_armed = false;
static bool g_arm_inhibit = false;
static uint32_t g_arm_hold_start_ms = 0;

enum FailSafeReason : uint8_t {
  FS_NONE = 0,
  FS_CMD_TIMEOUT = 1,
  FS_WS_DISCONNECT = 2,
  FS_IMU_FAIL = 3,
  FS_TILT_LIMIT = 4,
  FS_DT_LIMIT = 5,
  FS_KILL = 6,
  FS_MANUAL = 7,
};

static uint8_t g_last_fs = FS_NONE;

static volatile bool g_kill_pending = false;
static volatile uint8_t g_kill_reason = FS_KILL;
static portMUX_TYPE g_killMux = portMUX_INITIALIZER_UNLOCKED;

static uint32_t g_last_telem_ms = 0;

// ============================================================
// Motor
// ============================================================

static bool motorInit() {
  const bool ok0 = ledcAttach(kMotor0Pin, kPwmFreqHz, kPwmResolutionBits);
  const bool ok1 = ledcAttach(kMotor1Pin, kPwmFreqHz, kPwmResolutionBits);
  const bool ok2 = ledcAttach(kMotor2Pin, kPwmFreqHz, kPwmResolutionBits);
  const bool ok3 = ledcAttach(kMotor3Pin, kPwmFreqHz, kPwmResolutionBits);

  Serial.printf("LEDC attach: %d %d %d %d\n", ok0, ok1, ok2, ok3);

  ledcWrite(kMotor0Pin, 0);
  ledcWrite(kMotor1Pin, 0);
  ledcWrite(kMotor2Pin, 0);
  ledcWrite(kMotor3Pin, 0);

  return ok0 && ok1 && ok2 && ok3;
}


static void motorWriteNorm(int pin, float u) {
  const float uu = clampf(u, 0.0f, 1.0f);
  const uint32_t duty = (uint32_t)lrintf(uu * (float)kPwmMaxDuty);
  ledcWrite((uint8_t)pin, duty);
}

static void motorsWriteAll(float m0, float m1, float m2, float m3) {
  motorWriteNorm(kMotor0Pin, m0);
  motorWriteNorm(kMotor1Pin, m1);
  motorWriteNorm(kMotor2Pin, m2);
  motorWriteNorm(kMotor3Pin, m3);
}

static void resetAllPid() {
  g_roll_angle_pid.reset();
  g_pitch_angle_pid.reset();
  g_roll_rate_pid.reset();
  g_pitch_rate_pid.reset();
  g_yaw_rate_pid.reset();
}

static void requestKill(uint8_t reason) {
  portENTER_CRITICAL(&g_killMux);
  g_kill_pending = true;
  g_kill_reason = reason;
  portEXIT_CRITICAL(&g_killMux);
}

static void disarmNow(uint8_t reason) {
  g_armed = false;
  g_arm_inhibit = true;
  g_arm_hold_start_ms = 0;
  g_last_fs = reason;
  resetAllPid();
  motorsWriteAll(0.0f, 0.0f, 0.0f, 0.0f);
}

static float readVbatt() {
  if (kVbattAdcPin < 0) {
    return NAN;
  }
#if defined(ARDUINO_ARCH_ESP32)
  const uint32_t mv = analogReadMilliVolts(kVbattAdcPin);
  return ((float)mv / 1000.0f) * kVbattDividerRatio;
#else
  return NAN;
#endif
}

// ============================================================
// Apply tunings to PID objects
// ============================================================

static void applyTunings(const Tunings &t) {
  // Angle PIDs: angle error (deg) -> desired rate (dps)
  g_roll_angle_pid.setGains(t.angle.kp, t.angle.ki, t.angle.kd);
  g_pitch_angle_pid.setGains(t.angle.kp, t.angle.ki, t.angle.kd);
  g_roll_angle_pid.setOutputLimits(-250.0f, 250.0f);
  g_pitch_angle_pid.setOutputLimits(-250.0f, 250.0f);

  // Rate PIDs: rate error (dps) -> torque mix (-0.5..0.5)
  g_roll_rate_pid.setGains(t.rate.kp, t.rate.ki, t.rate.kd);
  g_pitch_rate_pid.setGains(t.rate.kp, t.rate.ki, t.rate.kd);
  g_roll_rate_pid.setOutputLimits(-0.5f, 0.5f);
  g_pitch_rate_pid.setOutputLimits(-0.5f, 0.5f);

  // Yaw rate
  g_yaw_rate_pid.setGains(t.yaw.kp, t.yaw.ki, t.yaw.kd);
  g_yaw_rate_pid.setOutputLimits(-0.3f, 0.3f);
}

static void applyTuningsIfDirty() {
  bool dirty = false;
  Tunings t;

  portENTER_CRITICAL(&g_cfgMux);
  if (g_tunings_dirty) {
    t = g_tunings;
    g_tunings_dirty = false;
    dirty = true;
  }
  portEXIT_CRITICAL(&g_cfgMux);

  if (dirty) {
    applyTunings(t);
    // 変更時に積分等が暴れないよう、リセット（飛行中に変えるならここは好み）
    resetAllPid();
  }
}

// ============================================================
// WebSocket protocol
// - C,thr,roll,pitch,yaw,armReq
// - PID,ANGLE,kp,ki,kd
// - PID,RATE,kp,ki,kd
// - PID,YAW,kp,ki,kd
// - GET
// - SAVE
// - K   (kill/disarm)
// ============================================================

static void sendCfgToClient(AsyncWebSocketClient *client) {
  if (client == nullptr) {
    return;
  }

  Tunings t;
  portENTER_CRITICAL(&g_cfgMux);
  t = g_tunings;
  portEXIT_CRITICAL(&g_cfgMux);

  char buf[256];
  snprintf(
    buf,
    sizeof(buf),
    "{\"type\":\"cfg\",\"angle\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f},"
    "\"rate\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f},"
    "\"yaw\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f}}",
    t.angle.kp, t.angle.ki, t.angle.kd,
    t.rate.kp, t.rate.ki, t.rate.kd,
    t.yaw.kp, t.yaw.ki, t.yaw.kd
  );

  client->text(buf);
}

static void handlePidMsg(char *saveptr) {
  // expects: TAG,kp,ki,kd (TAG = ANGLE/RATE/YAW)
  const char *tag = strtok_r(nullptr, ",", &saveptr);
  const char *kp_s = strtok_r(nullptr, ",", &saveptr);
  const char *ki_s = strtok_r(nullptr, ",", &saveptr);
  const char *kd_s = strtok_r(nullptr, ",", &saveptr);

  if (tag == nullptr || kp_s == nullptr || ki_s == nullptr || kd_s == nullptr) {
    return;
  }

  const float kp = (float)atof(kp_s);
  const float ki = (float)atof(ki_s);
  const float kd = (float)atof(kd_s);

  if (!isFinitef(kp) || !isFinitef(ki) || !isFinitef(kd)) {
    return;
  }

  portENTER_CRITICAL(&g_cfgMux);
  if (strcmp(tag, "ANGLE") == 0) {
    g_tunings.angle.kp = kp;
    g_tunings.angle.ki = ki;
    g_tunings.angle.kd = kd;
    g_tunings_dirty = true;
  } else if (strcmp(tag, "RATE") == 0) {
    g_tunings.rate.kp = kp;
    g_tunings.rate.ki = ki;
    g_tunings.rate.kd = kd;
    g_tunings_dirty = true;
  } else if (strcmp(tag, "YAW") == 0) {
    g_tunings.yaw.kp = kp;
    g_tunings.yaw.ki = ki;
    g_tunings.yaw.kd = kd;
    g_tunings_dirty = true;
  }
  portEXIT_CRITICAL(&g_cfgMux);
}

static void handleWsText(AsyncWebSocketClient *client, char *msg) {
  if (msg == nullptr) {
    return;
  }

  // K (kill)
  if (strcmp(msg, "K") == 0) {
    requestKill(FS_KILL);
    return;
  }

  // GET
  if (strcmp(msg, "GET") == 0) {
    sendCfgToClient(client);
    return;
  }

  // SAVE
  if (strcmp(msg, "SAVE") == 0) {
    saveTunings();
    return;
  }

  // CSV parse
  char *saveptr = nullptr;
  char *head = strtok_r(msg, ",", &saveptr);
  if (head == nullptr) {
    return;
  }

  if (strcmp(head, "PID") == 0) {
    handlePidMsg(saveptr);
    return;
  }

  if (strcmp(head, "C") == 0) {
    const char *t_s = strtok_r(nullptr, ",", &saveptr);
    const char *r_s = strtok_r(nullptr, ",", &saveptr);
    const char *p_s = strtok_r(nullptr, ",", &saveptr);
    const char *y_s = strtok_r(nullptr, ",", &saveptr);
    const char *a_s = strtok_r(nullptr, ",", &saveptr);

    if (t_s == nullptr || r_s == nullptr || p_s == nullptr || y_s == nullptr || a_s == nullptr) {
      return;
    }

    const float t = (float)atof(t_s);
    const float r = (float)atof(r_s);
    const float p = (float)atof(p_s);
    const float y = (float)atof(y_s);
    const int a = atoi(a_s);

    const bool arm_req = (a != 0);

    setRcCommand(t, r, p, y, arm_req);
  }
}

static void onWsEvent(AsyncWebSocket *server,
                      AsyncWebSocketClient *client,
                      AwsEventType type,
                      void *arg,
                      uint8_t *data,
                      size_t len) {
  (void)server;
  (void)arg;

  if (type == WS_EVT_CONNECT) {
    // 接続直後に現在ゲインを送る
    sendCfgToClient(client);
    return;
  }

  if (type == WS_EVT_DISCONNECT) {
    requestKill(FS_WS_DISCONNECT);
    return;
  }

  if (type != WS_EVT_DATA) {
    return;
  }

  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info == nullptr) {
    return;
  }
  if (!info->final || info->index != 0 || info->len != len) {
    return;
  }
  if (info->opcode != WS_TEXT) {
    return;
  }

  char buf[192];
  const size_t copy_len = (len < (sizeof(buf) - 1)) ? len : (sizeof(buf) - 1);
  memcpy(buf, data, copy_len);
  buf[copy_len] = '\0';

  handleWsText(client, buf);
}

// ============================================================
// WiFi + server
// ============================================================

static void wifiStartAp() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(kApSsid, kApPass);
  delay(100);

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

static void webServerStart() {
  g_ws.onEvent(onWsEvent);
  g_server.addHandler(&g_ws);

  g_server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send_P(200, "text/html", kIndexHtml);
  });

  g_server.on("/health", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "OK");
  });

  g_server.begin();
}

// ============================================================
// Arming logic
// ============================================================

static bool canArm(const RcCommand &rc) {
  if (rc.throttle > kArmThrottleMax) {
    return false;
  }
  if (fabsf(rc.roll) > kStickCenterMax ||
      fabsf(rc.pitch) > kStickCenterMax ||
      fabsf(rc.yaw) > kStickCenterMax) {
    return false;
  }
  // 機体が変な角度で置かれている状態でのARMを避ける
  if (fabsf(g_roll_deg) > 45.0f || fabsf(g_pitch_deg) > 45.0f) {
    return false;
  }
  return true;
}

static void updateArmingState(const RcCommand &rc, uint32_t now_ms) {
  // kill latch解除: arm_request OFF + throttle low
  if (!rc.arm_request && rc.throttle <= kArmThrottleMax) {
    g_arm_inhibit = false;
  }

  if (!g_armed) {
    motorsWriteAll(0.0f, 0.0f, 0.0f, 0.0f);

    if (g_arm_inhibit) {
      g_arm_hold_start_ms = 0;
      return;
    }

    if (rc.arm_request && canArm(rc)) {
      if (g_arm_hold_start_ms == 0) {
        g_arm_hold_start_ms = now_ms;
      }
      if ((now_ms - g_arm_hold_start_ms) >= kArmHoldMs) {
        g_armed = true;
        g_last_fs = FS_NONE;
        resetAllPid();
      }
    } else {
      g_arm_hold_start_ms = 0;
    }
    return;
  }

  // armed
  if (!rc.arm_request) {
    disarmNow(FS_MANUAL);
    return;
  }
}

// ============================================================
// Control step
// ============================================================

static void sendTelemetry(float dt, const RcCommand &rc, uint32_t cmd_age_ms) {
  const uint32_t now_ms = millis();
  if ((now_ms - g_last_telem_ms) < kTelemPeriodMs) {
    return;
  }
  g_last_telem_ms = now_ms;

  const float loop_hz = (dt > 1e-6f) ? (1.0f / dt) : 0.0f;
  const float vbatt = readVbatt();

  char buf[320];
  snprintf(
    buf,
    sizeof(buf),
    "{\"type\":\"tel\",\"t\":%lu,"
    "\"armed\":%d,"
    "\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f,"
    "\"dt_ms\":%.2f,\"loop_hz\":%.1f,"
    "\"cmd_age\":%lu,"
    "\"m0\":%.3f,\"m1\":%.3f,\"m2\":%.3f,\"m3\":%.3f,"
    "\"vbatt\":%.3f,"
    "\"fs\":%u}",
    (unsigned long)now_ms,
    g_armed ? 1 : 0,
    g_roll_deg, g_pitch_deg, g_yaw_deg,
    dt * 1000.0f, loop_hz,
    (unsigned long)cmd_age_ms,
    g_motor_last[0], g_motor_last[1], g_motor_last[2], g_motor_last[3],
    vbatt,
    (unsigned int)g_last_fs
  );

  g_ws.textAll(buf);
}

static void controlStep(float dt) {
  applyTuningsIfDirty();

  // Check kill request
  bool kill = false;
  uint8_t kill_reason = FS_KILL;

  portENTER_CRITICAL(&g_killMux);
  if (g_kill_pending) {
    kill = true;
    kill_reason = g_kill_reason;
    g_kill_pending = false;
  }
  portEXIT_CRITICAL(&g_killMux);

  if (kill) {
    disarmNow(kill_reason);
    return;
  }

  // Time sanity
  if (dt <= 0.0f || dt > kDtMaxSec) {
    disarmNow(FS_DT_LIMIT);
    return;
  }

  ImuSample s;
  if (!g_mpu.read(&s)) {
    disarmNow(FS_IMU_FAIL);
    return;
  }

  // NOTE: IMU軸が機体軸と違う場合、ここで符号/入れ替えが必要
  const float gx_rads = s.gx_dps * (float)M_PI / 180.0f;
  const float gy_rads = s.gy_dps * (float)M_PI / 180.0f;
  const float gz_rads = s.gz_dps * (float)M_PI / 180.0f;

  g_ahrs.update(gx_rads, gy_rads, gz_rads, s.ax_g, s.ay_g, s.az_g, dt);
  g_ahrs.getEulerDeg(&g_roll_deg, &g_pitch_deg, &g_yaw_deg);

  if (!isFinitef(g_roll_deg) || !isFinitef(g_pitch_deg) || !isFinitef(g_yaw_deg)) {
    disarmNow(FS_IMU_FAIL);
    return;
  }

  if (fabsf(g_roll_deg) > kTiltDisarmDeg || fabsf(g_pitch_deg) > kTiltDisarmDeg) {
    disarmNow(FS_TILT_LIMIT);
    return;
  }

  RcCommand rc = getRcCommand();
  const uint32_t now_ms = millis();
  const uint32_t cmd_age_ms = (rc.last_ms == 0) ? 0xFFFFFFFFUL : (now_ms - rc.last_ms);

  if (cmd_age_ms > kCmdTimeoutMs) {
    disarmNow(FS_CMD_TIMEOUT);
    sendTelemetry(dt, rc, cmd_age_ms);
    return;
  }

  updateArmingState(rc, now_ms);

  if (!g_armed) {
    sendTelemetry(dt, rc, cmd_age_ms);
    return;
  }

  // Setpoints
  const float throttle = clampf(rc.throttle, 0.0f, 1.0f);
  const float roll_sp_deg = rc.roll * kMaxAngleDeg;
  const float pitch_sp_deg = rc.pitch * kMaxAngleDeg;
  const float yaw_rate_sp_dps = rc.yaw * kMaxYawRateDps;

  // Angle -> rate
  const float roll_rate_sp_dps = g_roll_angle_pid.update(roll_sp_deg - g_roll_deg, dt);
  const float pitch_rate_sp_dps = g_pitch_angle_pid.update(pitch_sp_deg - g_pitch_deg, dt);

  // Rate -> torque
  const float roll_torque = g_roll_rate_pid.update(roll_rate_sp_dps - s.gx_dps, dt);
  const float pitch_torque = g_pitch_rate_pid.update(pitch_rate_sp_dps - s.gy_dps, dt);
  const float yaw_torque = g_yaw_rate_pid.update(yaw_rate_sp_dps - s.gz_dps, dt);

  if (!isFinitef(roll_torque) || !isFinitef(pitch_torque) || !isFinitef(yaw_torque)) {
    disarmNow(FS_IMU_FAIL);
    return;
  }

  // Mix (X)
  // yaw sign depends on motor spin directions. If yaw is reversed, flip yaw_torque.
  float m0 = throttle + roll_torque + pitch_torque - yaw_torque;  // FL
  float m1 = throttle - roll_torque + pitch_torque + yaw_torque;  // FR
  float m2 = throttle - roll_torque - pitch_torque - yaw_torque;  // RR
  float m3 = throttle + roll_torque - pitch_torque + yaw_torque;  // RL

  m0 = clampf(m0, 0.0f, 1.0f);
  m1 = clampf(m1, 0.0f, 1.0f);
  m2 = clampf(m2, 0.0f, 1.0f);
  m3 = clampf(m3, 0.0f, 1.0f);

  motorsWriteAll(m0, m1, m2, m3);

  sendTelemetry(dt, rc, cmd_age_ms);
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Boot...");

  wifiStartAp();
  webServerStart();

  Serial.println("Ready. Open http://192.168.4.1/");

  motorInit();
  motorsWriteAll(0, 0, 0, 0);

  if (!g_mpu.begin()) {
    Serial.println("MPU6050 init FAILED");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Calibrating gyro... keep still");
  g_mpu.calibrateGyro(500, 2);
  Serial.println("Gyro calibration done");

  g_ahrs.begin(kMadgwickBeta);

  loadTunings();
  applyTunings(g_tunings);
  resetAllPid();

  if (kVbattAdcPin >= 0) {
#if defined(ARDUINO_ARCH_ESP32)
    analogReadResolution(12);
    analogSetPinAttenuation(kVbattAdcPin, ADC_11db);
#endif
  }

  disarmNow(FS_NONE);
}

void loop() {
  static uint32_t last_us = 0;
  static uint32_t next_us = 0;

  const uint32_t now_us = micros();
  if (last_us == 0) {
    last_us = now_us;
    next_us = now_us + kLoopPeriodUs;
    return;
  }

  if ((int32_t)(now_us - next_us) >= 0) {
    while ((int32_t)(now_us - next_us) >= 0) {
      next_us += kLoopPeriodUs;
    }

    const float dt = (now_us - last_us) * 1e-6f;
    last_us = now_us;

    controlStep(dt);
  } else {
    // WiFi/WSのために少しだけ譲る
    delayMicroseconds(50);
  }
}
