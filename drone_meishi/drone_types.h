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
