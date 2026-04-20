#pragma once

// ============================================================
// IMU Sensor Fusion / IMU 传感器融合
// Provides Madgwick, Mahony, and Complementary filters for
// attitude estimation using MPU6050 data.
// 提供 Madgwick、Mahony 和互补滤波器，用于基于 MPU6050 数据的姿态估计。
// ============================================================

// ============================================================
// Madgwick IMU Filter / Madgwick IMU 滤波器
// Quaternion-based attitude estimation with accelerometer correction.
// 基于四元数的姿态估计，带加速度计校正。
// ============================================================

class MadgwickImu {
 public:
  // Initialize filter with beta gain / 使用 beta 增益初始化滤波器
  void begin(float beta) {
    beta_ = beta;
    q0_ = 1.0f;
    q1_ = 0.0f;
    q2_ = 0.0f;
    q3_ = 0.0f;
  }

  // Update beta parameter dynamically / 动态更新 beta 参数
  void setBeta(float beta) {
    if (beta > 0.0f) {
      beta_ = beta;
    }
  }

  // Update attitude estimate / 更新姿态估计
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
    // 注意：s = J^T * f（Madgwick 原始公式）。符号错误会导致发散。
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

  // Get Euler angles in degrees / 获取欧拉角（度）
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

  // Set attitude from Euler angles / 从欧拉角设置姿态
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
  // Gyro-only integration (fallback when accel is invalid)
  // 纯陀螺仪积分（加速度计无效时的回退方案）
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
// Mahony (Betaflight-style DCM) IMU Filter
// Mahony（Betaflight 风格 DCM）IMU 滤波器
// - Quaternion propagation with gyro / 四元数陀螺仪传播
// - Gravity correction from accelerometer / 加速度计重力校正
// - Optional integral feedback for gyro bias estimation
//   可选积分反馈用于陀螺仪偏置估计
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
      // Normalize accelerometer / 归一化加速度计
      ax /= norm_a;
      ay /= norm_a;
      az /= norm_a;

      // Estimated direction of gravity (body frame)
      // 估计重力方向（机体坐标系）
      const float vx = 2.0f * (q1 * q3 - q0 * q2);
      const float vy = 2.0f * (q0 * q1 + q2 * q3);
      const float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

      // Error is cross product between measured and estimated gravity.
      // e = a x v
      // 误差为测量值与估计值的重力叉积
      ex = (ay * vz - az * vy);
      ey = (az * vx - ax * vz);
      ez = (ax * vy - ay * vx);

      // Trust weighting / 信任度加权
      ex *= trust;
      ey *= trust;
      ez *= trust;

      // Integral feedback (gyro bias estimation)
      // 积分反馈（陀螺仪偏置估计）
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

    // Apply proportional + integral feedback to gyro
    // 应用比例+积分反馈到陀螺仪
    const float gx = gx_rads + kp_ * ex + integral_fb_x_;
    const float gy = gy_rads + kp_ * ey + integral_fb_y_;
    const float gz = gz_rads + kp_ * ez + integral_fb_z_;

    // Integrate quaternion rate / 积分四元数变化率
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
  float integral_limit_rads_ = 0.35f;  // clamp for bias estimate (rad/s) / 偏置估计限幅（弧度/秒）
  float spin_rate_limit_rads_ = 0.0f;  // 0 = disabled / 0 = 禁用

  float q0_ = 1.0f;
  float q1_ = 0.0f;
  float q2_ = 0.0f;
  float q3_ = 0.0f;
};

// ============================================================
// MPU6050 Minimal Driver / MPU6050 精简驱动
// Minimal I2C driver for MPU6050 with gyro and accel calibration.
// MPU6050 的精简 I2C 驱动，支持陀螺仪和加速度计校准。
// ============================================================

class Mpu6050 {
 public:
  // Initialize MPU6050 / 初始化 MPU6050
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

  // Calibrate gyroscope bias / 校准陀螺仪偏置
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

  // Calibrate accelerometer / 校准加速度计
  // Computes scale factors to normalize accelerometer to 1g when level.
  // 计算比例因子，使水平时加速度计归一化为 1g。
  void calibrateAccel(uint16_t samples, uint16_t delay_ms = 2) {
    float sumx = 0.0f;
    float sumy = 0.0f;
    float sumz = 0.0f;

    for (uint16_t i = 0; i < samples; i++) {
      int16_t ax, ay, az, gx, gy, gz;
      if (readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
        sumx += ax;
        sumy += ay;
        sumz += az;
      }
      delay(delay_ms);
    }

    const float avg_x = sumx / (float)samples;
    const float avg_y = sumy / (float)samples;
    const float avg_z = sumz / (float)samples;

    // Compute expected Z magnitude when level / 计算水平时的期望 Z 轴幅值
    const float expected_z = sqrtf(32768.0f * 32768.0f - avg_x * avg_x - avg_y * avg_y);

    if (expected_z > 1000.0f) {
      accel_scale_x_ = 32768.0f / avg_x;
      accel_scale_y_ = 32768.0f / avg_y;
      accel_scale_z_ = 32768.0f / avg_z;
    }
  }

  // Read sensor data with calibration applied / 读取传感器数据并应用校准
  bool read(ImuSample *out) {
    int16_t ax, ay, az, gx, gy, gz;
    if (!readRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
      return false;
    }

    const float a_scale = 8.0f / 32768.0f;
    const float g_scale = 2000.0f / 32768.0f;

    // Apply accel calibration if available / 如有则应用加速度计校准
    float ax_raw = ax * accel_scale_x_;
    float ay_raw = ay * accel_scale_y_;
    float az_raw = az * accel_scale_z_;

    float ax_g = ax_raw * a_scale;
    float ay_g = ay_raw * a_scale;
    float az_g = az_raw * a_scale;

    float gx_dps = gx * g_scale - gyro_bias_x_;
    float gy_dps = gy * g_scale - gyro_bias_y_;
    float gz_dps = gz * g_scale - gyro_bias_z_;

    // Apply board orientation mapping (sensor -> body)
    // 应用板载方向映射（传感器 -> 机体）
    if (kImuSwapXY) {
      const float t_a = ax_g;
      ax_g = ay_g;
      ay_g = t_a;
      const float t_g = gx_dps;
      gx_dps = gy_dps;
      gy_dps = t_g;
      // When swapping XY, also negate to maintain right-hand rule
      // 交换 XY 时取反以保持右手定则
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

  // Get accel calibration status / 获取加速度计校准状态
  bool isAccelCalibrated() const {
    return (accel_scale_x_ != 1.0f || accel_scale_y_ != 1.0f || accel_scale_z_ != 1.0f);
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

  // Accel calibration scale factors / 加速度计校准比例因子
  float accel_scale_x_ = 1.0f;
  float accel_scale_y_ = 1.0f;
  float accel_scale_z_ = 1.0f;
};

// Convert accelerometer reading to roll/pitch angles
// 将加速度计读数转换为横滚/俯仰角
static void accelToRollPitchDeg(const ImuSample &s, float *roll_deg, float *pitch_deg) {
  const float ax = s.ax_g;
  const float ay = s.ay_g;
  const float az = s.az_g;

  const float roll = atan2f(ay, az);
  const float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  *roll_deg = roll * 180.0f / (float)M_PI;
  *pitch_deg = pitch * 180.0f / (float)M_PI;
}

// Check if IMU is stationary / 检查 IMU 是否静止
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
