#pragma once

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

  void setBeta(float beta) {
    if (beta > 0.0f) {
      beta_ = beta;
    }
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

    // NOTE: s = J^T * f (Madgwick original). Sign errors here will cause divergence.
    float s0 = j_11or24 * f1 + j_14or21 * f2;
    float s1 = j_12or23 * f1 - j_13or22 * f2 - j_32 * f3;
    float s2 = j_13or22 * f1 + j_12or23 * f2 + j_33 * f3;
    float s3 = j_14or21 * f1 - j_11or24 * f2;

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

  void setEulerDeg(float roll_deg, float pitch_deg, float yaw_deg) {
    const float roll = roll_deg * (float)M_PI / 180.0f;
    const float pitch = pitch_deg * (float)M_PI / 180.0f;
    const float yaw = yaw_deg * (float)M_PI / 180.0f;
    setEulerRad(roll, pitch, yaw);
  }

  void setEulerRad(float roll, float pitch, float yaw) {
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    q0_ = cr * cp * cy + sr * sp * sy;
    q1_ = sr * cp * cy - cr * sp * sy;
    q2_ = cr * sp * cy + sr * cp * sy;
    q3_ = cr * cp * sy - sr * sp * cy;
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
// Mahony (Betaflight-style DCM) IMU-only
// - Quaternion propagation with gyro
// - Gravity correction from accelerometer (when trusted)
// - Optional integral feedback to estimate gyro bias
// ============================================================

class MahonyImu {
 public:
  void begin(float kp, float ki) {
    kp_ = kp;
    ki_ = ki;
    reset();
  }

  void reset() {
    q0_ = 1.0f;
    q1_ = 0.0f;
    q2_ = 0.0f;
    q3_ = 0.0f;
    integral_fb_x_ = 0.0f;
    integral_fb_y_ = 0.0f;
    integral_fb_z_ = 0.0f;
  }

  void setGains(float kp, float ki) {
    kp_ = kp;
    ki_ = ki;
  }

  void setIntegralLimitRads(float limit_rads) {
    integral_limit_rads_ = fabsf(limit_rads);
  }

  void setSpinRateLimitDps(float limit_dps) {
    spin_rate_limit_rads_ = fabsf(limit_dps) * (float)M_PI / 180.0f;
  }

  void update(float gx_rads, float gy_rads, float gz_rads,
              float ax, float ay, float az,
              float dt, float acc_trust = 1.0f) {
    if (dt <= 0.0f) {
      return;
    }

    float q0 = q0_;
    float q1 = q1_;
    float q2 = q2_;
    float q3 = q3_;

    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;

    const float norm_a = sqrtf(ax * ax + ay * ay + az * az);
    const float trust = clampf(acc_trust, 0.0f, 1.0f);
    const bool acc_ok = (norm_a > 1e-6f) && (trust > 0.0f);

    if (acc_ok) {
      // Normalize accelerometer.
      ax /= norm_a;
      ay /= norm_a;
      az /= norm_a;

      // Estimated direction of gravity (body frame).
      const float vx = 2.0f * (q1 * q3 - q0 * q2);
      const float vy = 2.0f * (q0 * q1 + q2 * q3);
      const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

      // Error is cross product between measured and estimated gravity.
      // e = a x v
      ex = (ay * vz - az * vy);
      ey = (az * vx - ax * vz);
      ez = (ax * vy - ay * vx);

      // Trust weighting.
      ex *= trust;
      ey *= trust;
      ez *= trust;

      // Integral feedback (gyro bias estimation).
      if (ki_ > 0.0f) {
        const float gyro_mag = sqrtf(gx_rads * gx_rads + gy_rads * gy_rads + gz_rads * gz_rads);
        const bool spin_ok = (spin_rate_limit_rads_ <= 0.0f) || (gyro_mag <= spin_rate_limit_rads_);
        if (spin_ok) {
          integral_fb_x_ += ki_ * ex * dt;
          integral_fb_y_ += ki_ * ey * dt;
          integral_fb_z_ += ki_ * ez * dt;

          if (integral_limit_rads_ > 0.0f) {
            integral_fb_x_ = clampf(integral_fb_x_, -integral_limit_rads_, integral_limit_rads_);
            integral_fb_y_ = clampf(integral_fb_y_, -integral_limit_rads_, integral_limit_rads_);
            integral_fb_z_ = clampf(integral_fb_z_, -integral_limit_rads_, integral_limit_rads_);
          }
        }
      }
    }

    // Apply proportional + integral feedback to gyro.
    const float gx = gx_rads + kp_ * ex + integral_fb_x_;
    const float gy = gy_rads + kp_ * ey + integral_fb_y_;
    const float gz = gz_rads + kp_ * ez + integral_fb_z_;

    // Integrate quaternion rate.
    const float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    const float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    const float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    const float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

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

  void setEulerDeg(float roll_deg, float pitch_deg, float yaw_deg) {
    const float roll = roll_deg * (float)M_PI / 180.0f;
    const float pitch = pitch_deg * (float)M_PI / 180.0f;
    const float yaw = yaw_deg * (float)M_PI / 180.0f;
    setEulerRad(roll, pitch, yaw);
  }

  void setEulerRad(float roll, float pitch, float yaw) {
    const float cr = cosf(roll * 0.5f);
    const float sr = sinf(roll * 0.5f);
    const float cp = cosf(pitch * 0.5f);
    const float sp = sinf(pitch * 0.5f);
    const float cy = cosf(yaw * 0.5f);
    const float sy = sinf(yaw * 0.5f);

    q0_ = cr * cp * cy + sr * sp * sy;
    q1_ = sr * cp * cy - cr * sp * sy;
    q2_ = cr * sp * cy + sr * cp * sy;
    q3_ = cr * cp * sy - sr * sp * cy;
  }

 private:
  float kp_ = 2.0f;
  float ki_ = 0.0f;

  float integral_fb_x_ = 0.0f;
  float integral_fb_y_ = 0.0f;
  float integral_fb_z_ = 0.0f;
  float integral_limit_rads_ = 0.35f;  // clamp for bias estimate (rad/s)
  float spin_rate_limit_rads_ = 0.0f;  // 0 = disabled

  float q0_ = 1.0f;
  float q1_ = 0.0f;
  float q2_ = 0.0f;
  float q3_ = 0.0f;
};

// ============================================================
// MPU6050 minimal
// ============================================================

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

    float ax_g = ax * a_scale;
    float ay_g = ay * a_scale;
    float az_g = az * a_scale;

    float gx_dps = gx * g_scale - gyro_bias_x_;
    float gy_dps = gy * g_scale - gyro_bias_y_;
    float gz_dps = gz * g_scale - gyro_bias_z_;

    // Apply board orientation mapping (sensor -> body).
    if (kImuSwapXY) {
      const float t_a = ax_g;
      ax_g = ay_g;
      ay_g = t_a;
      const float t_g = gx_dps;
      gx_dps = gy_dps;
      gy_dps = t_g;
      // When swapping XY, also negate to maintain right-hand rule
      gx_dps = -gx_dps;
      gy_dps = -gy_dps;
    }
    if (kImuFlipX) {
      ax_g = -ax_g;
      gx_dps = -gx_dps;
    }
    if (kImuFlipY) {
      ay_g = -ay_g;
      gy_dps = -gy_dps;
    }
    if (kImuFlipZ) {
      az_g = -az_g;
      gz_dps = -gz_dps;
    }

    out->ax_g = ax_g;
    out->ay_g = ay_g;
    out->az_g = az_g;

    out->gx_dps = gx_dps;
    out->gy_dps = gy_dps;
    out->gz_dps = gz_dps;
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

static void accelToRollPitchDeg(const ImuSample &s, float *roll_deg, float *pitch_deg) {
  const float ax = s.ax_g;
  const float ay = s.ay_g;
  const float az = s.az_g;

  const float roll = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  *roll_deg = roll * 180.0f / (float)M_PI;
  *pitch_deg = pitch * 180.0f / (float)M_PI;
}

static bool isImuStationary(const ImuSample &s) {
  const float acc_norm = sqrtf(s.ax_g * s.ax_g + s.ay_g * s.ay_g + s.az_g * s.az_g);
  if (!isFinitef(acc_norm)) {
    return false;
  }
  if (fabsf(acc_norm - 1.0f) > kRelevelAccTolG) {
    return false;
  }
  if (fabsf(s.gx_dps) > kRelevelGyroDps ||
      fabsf(s.gy_dps) > kRelevelGyroDps ||
      fabsf(s.gz_dps) > kRelevelGyroDps) {
    return false;
  }
  return true;
}
