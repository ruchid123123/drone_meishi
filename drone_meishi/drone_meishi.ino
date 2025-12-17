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

// LEDs (LED1: right, LED2: left)
static const int led1 = 18;
static const int led2 = 19;

// PWM
static const uint32_t kPwmFreqHz = 20000;
static const uint8_t kPwmResolutionBits = 11;
static const uint32_t kPwmMaxDuty = (1U << kPwmResolutionBits) - 1U;

static const uint32_t kLedPwmFreqHz = 1000;
static const uint8_t kLedPwmResolutionBits = 8;
static const uint32_t kLedPwmMaxDuty = (1U << kLedPwmResolutionBits) - 1U;

static const int kMotor0Ch = 0;
static const int kMotor1Ch = 1;
static const int kMotor2Ch = 2;
static const int kMotor3Ch = 3;

// Control loop
static const uint32_t kLoopPeriodUs = 2000;  // 500Hz
static const float kDtMaxSec = 0.02f;

// RC and arming
static const uint32_t kDefaultCmdTimeoutMs = 300;
static const uint32_t kMinCmdTimeoutMs = 100;
static const uint32_t kMaxCmdTimeoutMs = 2000;
static const float kArmThrottleMax = 0.05f;
static const float kStickCenterMax = 0.15f;
static const uint32_t kArmHoldMs = 800;

// Limits
static const float kDefaultMaxAngleDeg = 30.0f;
static const float kMinMaxAngleDeg = 5.0f;
static const float kMaxMaxAngleDeg = 80.0f;
static const float kDefaultMaxYawRateDps = 180.0f;
static const float kMinMaxYawRateDps = 30.0f;
static const float kMaxMaxYawRateDps = 360.0f;
static const float kDefaultTiltDisarmDeg = 80.0f;
static const float kMinTiltDisarmDeg = 20.0f;
static const float kMaxTiltDisarmDeg = 85.0f;

// Telemetry
static const uint32_t kDefaultTelemPeriodMs = 50;  // 20Hz
static const uint32_t kMinTelemPeriodMs = 20;
static const uint32_t kMaxTelemPeriodMs = 500;

// IMU filter
static const float kMadgwickBeta = 0.08f;
// Disarmed時に姿勢を重力方向へ寄せてドリフトを抑える
static const float kRelevelAccTolG = 0.15f;
static const float kRelevelGyroDps = 3.0f;

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

static inline uint32_t clampu32(uint32_t v, uint32_t vmin, uint32_t vmax) {
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
// LED status
// ============================================================
#include "drone_led.h"

// ============================================================
// Web UI
// ============================================================
#include "drone_web_ui.h"

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
#include "drone_pid.h"

// ============================================================
// Madgwick IMU-only
// ============================================================
#include "drone_imu.h"

// ============================================================
// PID tunings + NVS
// ============================================================

static RuntimeConfig g_config = {
  kDefaultMaxAngleDeg,
  kDefaultMaxYawRateDps,
  kDefaultTiltDisarmDeg,
  kDefaultCmdTimeoutMs,
  kDefaultTelemPeriodMs,
};

static Tunings g_tunings = {
  {4.0f, 0.0f, 0.0f},       // angle
  {0.010f, 0.0f, 0.0002f},  // rate
  {0.010f, 0.0f, 0.0f},     // yaw
};

static volatile bool g_tunings_dirty = false;
static portMUX_TYPE g_cfgMux = portMUX_INITIALIZER_UNLOCKED;

static Preferences g_prefs;

static void clampConfig(RuntimeConfig *cfg) {
  cfg->max_angle_deg = clampf(cfg->max_angle_deg, kMinMaxAngleDeg, kMaxMaxAngleDeg);
  cfg->max_yaw_rate_dps = clampf(cfg->max_yaw_rate_dps, kMinMaxYawRateDps, kMaxMaxYawRateDps);
  cfg->tilt_disarm_deg = clampf(cfg->tilt_disarm_deg, kMinTiltDisarmDeg, kMaxTiltDisarmDeg);
  cfg->cmd_timeout_ms = clampu32(cfg->cmd_timeout_ms, kMinCmdTimeoutMs, kMaxCmdTimeoutMs);
  cfg->telem_period_ms = clampu32(cfg->telem_period_ms, kMinTelemPeriodMs, kMaxTelemPeriodMs);
}

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

  g_config.max_angle_deg = g_prefs.getFloat("max_ang", g_config.max_angle_deg);
  g_config.max_yaw_rate_dps = g_prefs.getFloat("max_yaw", g_config.max_yaw_rate_dps);
  g_config.tilt_disarm_deg = g_prefs.getFloat("tilt_dis", g_config.tilt_disarm_deg);
  g_config.cmd_timeout_ms = g_prefs.getUInt("cmd_to", g_config.cmd_timeout_ms);
  g_config.telem_period_ms = g_prefs.getUInt("telem_ms", g_config.telem_period_ms);
  clampConfig(&g_config);

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

  g_prefs.putFloat("max_ang", g_config.max_angle_deg);
  g_prefs.putFloat("max_yaw", g_config.max_yaw_rate_dps);
  g_prefs.putFloat("tilt_dis", g_config.tilt_disarm_deg);
  g_prefs.putUInt("cmd_to", g_config.cmd_timeout_ms);
  g_prefs.putUInt("telem_ms", g_config.telem_period_ms);

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
// - CFG,KEY,VALUE (MAX_ANGLE, MAX_YAW_RATE, TILT_DISARM, CMD_TIMEOUT, TELEM_MS)
// - GET
// - SAVE
// - K   (kill/disarm)
// ============================================================

static void sendCfgToClient(AsyncWebSocketClient *client) {
  if (client == nullptr) {
    return;
  }

  Tunings t;
  RuntimeConfig cfg;
  portENTER_CRITICAL(&g_cfgMux);
  t = g_tunings;
  cfg = g_config;
  portEXIT_CRITICAL(&g_cfgMux);

  char buf[384];
  snprintf(
    buf,
    sizeof(buf),
    "{\"type\":\"cfg\",\"angle\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f},"
    "\"rate\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f},"
    "\"yaw\":{\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f},"
    "\"limits\":{\"max_angle\":%.2f,\"max_yaw_rate\":%.2f,\"tilt_disarm\":%.2f,"
    "\"cmd_timeout\":%lu,\"telem_ms\":%lu}}",
    t.angle.kp, t.angle.ki, t.angle.kd,
    t.rate.kp, t.rate.ki, t.rate.kd,
    t.yaw.kp, t.yaw.ki, t.yaw.kd,
    cfg.max_angle_deg, cfg.max_yaw_rate_dps, cfg.tilt_disarm_deg,
    (unsigned long)cfg.cmd_timeout_ms, (unsigned long)cfg.telem_period_ms
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

static void handleCfgMsg(char *saveptr) {
  // expects: KEY,VALUE
  const char *key = strtok_r(nullptr, ",", &saveptr);
  const char *val_s = strtok_r(nullptr, ",", &saveptr);

  if (key == nullptr || val_s == nullptr) {
    return;
  }

  const float vf = (float)atof(val_s);
  const uint32_t vu = (uint32_t)atoi(val_s);

  portENTER_CRITICAL(&g_cfgMux);
  if (strcmp(key, "MAX_ANGLE") == 0) {
    if (isFinitef(vf)) {
      g_config.max_angle_deg = clampf(vf, kMinMaxAngleDeg, kMaxMaxAngleDeg);
    }
  } else if (strcmp(key, "MAX_YAW_RATE") == 0) {
    if (isFinitef(vf)) {
      g_config.max_yaw_rate_dps = clampf(vf, kMinMaxYawRateDps, kMaxMaxYawRateDps);
    }
  } else if (strcmp(key, "TILT_DISARM") == 0) {
    if (isFinitef(vf)) {
      g_config.tilt_disarm_deg = clampf(vf, kMinTiltDisarmDeg, kMaxTiltDisarmDeg);
    }
  } else if (strcmp(key, "CMD_TIMEOUT") == 0) {
    g_config.cmd_timeout_ms = clampu32(vu, kMinCmdTimeoutMs, kMaxCmdTimeoutMs);
  } else if (strcmp(key, "TELEM_MS") == 0) {
    g_config.telem_period_ms = clampu32(vu, kMinTelemPeriodMs, kMaxTelemPeriodMs);
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

  if (strcmp(head, "CFG") == 0) {
    handleCfgMsg(saveptr);
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

static void sendTelemetry(float dt, const RcCommand &rc, uint32_t cmd_age_ms,
                          const RuntimeConfig &cfg) {
  const uint32_t now_ms = millis();
  if ((now_ms - g_last_telem_ms) < cfg.telem_period_ms) {
    return;
  }
  g_last_telem_ms = now_ms;

  const float loop_hz = (dt > 1e-6f) ? (1.0f / dt) : 0.0f;
  const float vbatt = readVbatt();
  char vbatt_buf[16];
  if (isFinitef(vbatt)) {
    snprintf(vbatt_buf, sizeof(vbatt_buf), "%.3f", vbatt);
  } else {
    snprintf(vbatt_buf, sizeof(vbatt_buf), "null");
  }

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
    "\"vbatt\":%s,"
    "\"fs\":%u}",
    (unsigned long)now_ms,
    g_armed ? 1 : 0,
    g_roll_deg, g_pitch_deg, g_yaw_deg,
    dt * 1000.0f, loop_hz,
    (unsigned long)cmd_age_ms,
    g_motor_last[0], g_motor_last[1], g_motor_last[2], g_motor_last[3],
    vbatt_buf,
    (unsigned int)g_last_fs
  );

  g_ws.textAll(buf);
}

static void controlStep(float dt) {
  applyTuningsIfDirty();
  g_last_fs = FS_NONE;

  RuntimeConfig cfg;
  portENTER_CRITICAL(&g_cfgMux);
  cfg = g_config;
  portEXIT_CRITICAL(&g_cfgMux);

  RcCommand rc = getRcCommand();
  const uint32_t now_ms = millis();
  const uint32_t cmd_age_ms = (rc.last_ms == 0) ? 0xFFFFFFFFUL : (now_ms - rc.last_ms);

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
    g_led_mode = LED_FAILSAFE;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
    return;
  }

  // Time sanity
  if (dt <= 0.0f || dt > kDtMaxSec) {
    disarmNow(FS_DT_LIMIT);
    g_led_mode = LED_FAILSAFE;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
    return;
  }

  ImuSample s;
  if (!g_mpu.read(&s)) {
    disarmNow(FS_IMU_FAIL);
    g_led_mode = LED_FAILSAFE;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
    return;
  }

  // NOTE: IMU軸が機体軸と違う場合、ここで符号/入れ替えが必要
  const float gx_rads = s.gx_dps * (float)M_PI / 180.0f;
  const float gy_rads = s.gy_dps * (float)M_PI / 180.0f;
  const float gz_rads = s.gz_dps * (float)M_PI / 180.0f;

  g_ahrs.update(gx_rads, gy_rads, gz_rads, s.ax_g, s.ay_g, s.az_g, dt);
  float roll_deg = 0.0f;
  float pitch_deg = 0.0f;
  float yaw_deg = 0.0f;
  g_ahrs.getEulerDeg(&roll_deg, &pitch_deg, &yaw_deg);

  if (!isFinitef(roll_deg) || !isFinitef(pitch_deg) || !isFinitef(yaw_deg)) {
    disarmNow(FS_IMU_FAIL);
    g_led_mode = LED_FAILSAFE;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
    return;
  }

  float use_roll_deg = roll_deg;
  float use_pitch_deg = pitch_deg;
  if (!g_armed && isImuStationary(s)) {
    float acc_roll_deg = 0.0f;
    float acc_pitch_deg = 0.0f;
    accelToRollPitchDeg(s, &acc_roll_deg, &acc_pitch_deg);
    if (isFinitef(acc_roll_deg) && isFinitef(acc_pitch_deg)) {
      use_roll_deg = acc_roll_deg;
      use_pitch_deg = acc_pitch_deg;
      g_ahrs.setEulerDeg(use_roll_deg, use_pitch_deg, yaw_deg);
    }
  }

  g_roll_deg = use_roll_deg;
  g_pitch_deg = use_pitch_deg;
  g_yaw_deg = yaw_deg;

  if (fabsf(g_roll_deg) > cfg.tilt_disarm_deg ||
      fabsf(g_pitch_deg) > cfg.tilt_disarm_deg) {
    disarmNow(FS_TILT_LIMIT);
    g_led_mode = LED_FAILSAFE;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
    return;
  }

  if (cmd_age_ms > cfg.cmd_timeout_ms) {
    disarmNow(FS_CMD_TIMEOUT);
    g_led_mode = LED_FAILSAFE;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
    return;
  }

  updateArmingState(rc, now_ms);

  if (!g_armed) {
    g_led_mode = (rc.arm_request && !g_arm_inhibit) ? LED_ARMING : LED_DISARMED;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
    return;
  }

  g_led_mode = LED_ARMED;

  // Setpoints
  const float throttle = clampf(rc.throttle, 0.0f, 1.0f);
  const float roll_sp_deg = rc.roll * cfg.max_angle_deg;
  const float pitch_sp_deg = rc.pitch * cfg.max_angle_deg;
  const float yaw_rate_sp_dps = rc.yaw * cfg.max_yaw_rate_dps;

  // Angle -> rate
  const float roll_rate_sp_dps = g_roll_angle_pid.update(roll_sp_deg - g_roll_deg, dt);
  const float pitch_rate_sp_dps = g_pitch_angle_pid.update(pitch_sp_deg - g_pitch_deg, dt);

  // Rate -> torque
  const float roll_torque = g_roll_rate_pid.update(roll_rate_sp_dps - s.gx_dps, dt);
  const float pitch_torque = g_pitch_rate_pid.update(pitch_rate_sp_dps - s.gy_dps, dt);
  const float yaw_torque = g_yaw_rate_pid.update(yaw_rate_sp_dps - s.gz_dps, dt);

  if (!isFinitef(roll_torque) || !isFinitef(pitch_torque) || !isFinitef(yaw_torque)) {
    disarmNow(FS_IMU_FAIL);
    g_led_mode = LED_FAILSAFE;
    sendTelemetry(dt, rc, cmd_age_ms, cfg);
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

  sendTelemetry(dt, rc, cmd_age_ms, cfg);
}

// ============================================================
// Setup / loop
// ============================================================

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("Boot...");

  g_led_mode = LED_BOOT;
  ledInit();

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

  wifiStartAp();
  webServerStart();

  g_led_mode = LED_DISARMED;

  Serial.println("Ready. Open http://192.168.4.1/");
}

void loop() {
  static uint32_t last_us = 0;
  static uint32_t next_us = 0;

  ledUpdate();

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
