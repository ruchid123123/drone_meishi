#ifndef DRONE_TYPES_H_
#define DRONE_TYPES_H_

#include <Arduino.h>

struct RcCommand {
  float throttle;
  float roll;
  float pitch;
  float yaw;
  bool arm_request;
  uint32_t last_ms;
};

struct MotorTestState;

struct ImuSample {
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
};

struct RuntimeConfig {
  float max_angle_deg;
  float max_yaw_rate_dps;
  float tilt_disarm_deg;
  uint32_t cmd_timeout_ms;
  uint32_t telem_period_ms;
};

struct PidTriplet {
  float kp;
  float ki;
  float kd;
};

struct Tunings {
  PidTriplet angle;
  PidTriplet rate;
  PidTriplet yaw;
};

#endif  // DRONE_TYPES_H_
