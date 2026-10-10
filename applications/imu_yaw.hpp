#ifndef IMU_YAW_HPP
#define IMU_YAW_HPP

#include <stdint.h>

constexpr int IMU_WAIT = 0;
constexpr int IMU_WARMUP = 1;
constexpr int IMU_CALIBRATING = 2;
constexpr int IMU_READY = 3;
constexpr int IMU_FAULT = 4;
constexpr int IMU_REASON_NONE = 0;
constexpr int IMU_REASON_SPI = 1;
constexpr int IMU_REASON_GAP = 2;
constexpr int IMU_REASON_LEVEL = 3;
constexpr int IMU_REASON_RANGE = 4;
constexpr int IMU_SAMPLE_PERIOD_MS = 5;
constexpr int IMU_MAX_GAP_MS = 20;
constexpr int IMU_LEVEL_FAULT_DELAY_MS = 100;
constexpr int IMU_WARMUP_MS = 3000;
constexpr int IMU_CALIBRATION_MS = 2000;
constexpr int IMU_CALIBRATION_SAMPLES = 400;
constexpr float IMU_GRAVITY_MPS2 = 9.80665f;
constexpr float IMU_GRAVITY_TOLERANCE_MPS2 = 1.0f;
constexpr float IMU_LEVEL_MIN_COSINE = 0.94f;
constexpr float IMU_CALIBRATION_MAX_DEG_S = 3.0f;
constexpr float IMU_CALIBRATION_SPREAD_DEG_S = 0.8f;
constexpr float IMU_GYRO_LIMIT_DEG_S = 1900.0f;
constexpr float IMU_DEG_TO_RAD = 0.01745329252f;

struct ImuYawState
{
  float acceleration_mps2[3];
  float angular_velocity_deg_s[3];
  float bias_deg_s[3];
  float corrected_z_deg_s;
  float yaw_rad;
  float acceleration_norm_mps2;
  uint32_t last_sample_ms;
  uint32_t interval_ms;
  uint32_t max_interval_ms;
  uint32_t samples;
  uint32_t calibration_samples;
  uint32_t calibration_restarts;
  uint32_t io_errors;
  uint32_t gap_events;
  uint8_t state;
  uint8_t reason;
  uint8_t level;
  uint8_t valid;
  uint8_t io_ok;
  uint8_t init_error;
};

void imu_yaw_begin(uint32_t now_ms);
void imu_yaw_io_error(int init_error);
void imu_yaw_update(const float acceleration_mps2[3], const float gyro_deg_s[3], uint32_t now_ms);
void imu_yaw_get_state(ImuYawState * result);

#endif
