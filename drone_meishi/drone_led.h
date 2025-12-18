#pragma once

// ============================================================
// LED status
// ============================================================

enum LedMode : uint8_t {
  LED_BOOT = 0,
  LED_DISARMED = 1,
  LED_ARMING = 2,
  LED_ARMED = 3,
  LED_FAILSAFE = 4,
};

static LedMode g_led_mode = LED_BOOT;

static inline uint32_t ledDutyFromLevel(float level) {
  const float l = clampf(level, 0.0f, 1.0f);
  return (uint32_t)lrintf(l * (float)kLedPwmMaxDuty);
}

static inline void ledWriteLevel(int pin, float level) {
  ledcWrite((uint8_t)pin, ledDutyFromLevel(level));
}

static inline void setLeds(float l1, float l2) {
  ledWriteLevel(led1, l1);
  ledWriteLevel(led2, l2);
}

static float breatheLevel(uint32_t now_ms, uint32_t period_ms) {
  const float phase = (float)(now_ms % period_ms) / (float)period_ms;
  return 0.5f - 0.5f * cosf(2.0f * (float)M_PI * phase);
}

static void ledInit() {
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  ledcAttach(led1, kLedPwmFreqHz, kLedPwmResolutionBits);
  ledcAttach(led2, kLedPwmFreqHz, kLedPwmResolutionBits);
  setLeds(0.0f, 0.0f);
}

static void ledUpdate() {
  const uint32_t now_ms = millis();
  switch (g_led_mode) {
    case LED_BOOT: {
      const uint32_t period_ms = 220;
      const bool right_on = ((now_ms / period_ms) & 1U) == 0U;
      setLeds(right_on ? 1.0f : 0.0f, right_on ? 0.0f : 1.0f);
    } break;
    case LED_FAILSAFE: {
      const uint32_t period_ms = 120;
      const bool on = ((now_ms / period_ms) & 1U) == 0U;
      const float level = on ? 1.0f : 0.0f;
      setLeds(level, level);
    } break;
    case LED_ARMING: {
      const uint32_t period_ms = 300;
      const bool on = ((now_ms / period_ms) & 1U) == 0U;
      const float level = on ? 1.0f : 0.0f;
      setLeds(level, level);
    } break;
    case LED_ARMED: {
      setLeds(1.0f, 1.0f);
    } break;
    case LED_DISARMED:
    default: {
      const float level = breatheLevel(now_ms, 1400);
      setLeds(level, level);
    } break;
  }
}

