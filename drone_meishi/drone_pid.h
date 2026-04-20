#pragma once

// ============================================================
// PID Controller / PID 控制器
// Implements a standard PID controller with integral clamping
// to prevent integral windup.
// 实现标准 PID 控制器，带积分限幅以防止积分饱和。
// ============================================================

class Pid {
 public:
  // Set PID gains / 设置 PID 增益参数
  void setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  // Set output limits / 设置输出限幅
  void setOutputLimits(float out_min, float out_max) {
    out_min_ = out_min;
    out_max_ = out_max;
  }

  // Reset internal state / 重置内部状态
  void reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
    has_prev_ = false;
  }

  // Update PID output / 更新 PID 输出
  // @param error: error signal / 误差信号
  // @param dt: time step in seconds / 时间步长（秒）
  // @return: PID output / PID 输出值
  float update(float error, float dt) {
    if (dt <= 0.0f) {
      return 0.0f;
    }

    const float p = kp_ * error;

    // Integral term with anti-windup / 积分项，带抗饱和处理
    integral_ += error * dt;
    // Clamp integral to prevent windup / 限制积分值以防止积分饱和
    integral_ = clampf(integral_, -kIntegralMax_, kIntegralMax_);
    const float i = ki_ * integral_;

    float d = 0.0f;
    if (has_prev_) {
      d = kd_ * (error - prev_error_) / dt;
    }
    prev_error_ = error;
    has_prev_ = true;

    float out = p + i + d;
    out = clampf(out, out_min_, out_max_);

    // Conditional integration: back off integral when saturated
    // 条件积分：当输出饱和时，减小积分值
    if (out == out_min_ || out == out_max_) {
      integral_ *= 0.99f;
    }
    return out;
  }

  // Set integral limit / 设置积分限幅值
  void setIntegralLimit(float limit) {
    kIntegralMax_ = fabsf(limit);
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
  float kIntegralMax_ = 100.0f;  // Default integral limit / 默认积分限幅
};
