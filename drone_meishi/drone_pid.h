#pragma once

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

