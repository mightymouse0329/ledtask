#include "imu_yaw.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

static ImuYawState yaw_state;
static uint32_t warmup_start_ms;
static uint32_t calibration_start_ms;
static float calibration_sum[3];
static float calibration_min[3];
static float calibration_max[3];
static float previous_rate_deg_s;

static void reset_calibration(void)
{
  if (yaw_state.calibration_samples) yaw_state.calibration_restarts++;
  yaw_state.calibration_samples = 0;
  for (int index = 0; index < 3; index++) calibration_sum[index] = 0.0f;
}

static void invalidate_yaw(int reason)
{
  if (yaw_state.state != IMU_FAULT) yaw_state.reason = reason;
  yaw_state.state = IMU_FAULT;
  yaw_state.valid = 0;
}

void imu_yaw_begin(uint32_t now_ms)
{
  yaw_state.init_error = 0;
  if (yaw_state.state == IMU_FAULT) return;
  yaw_state.state = IMU_WARMUP;
  yaw_state.valid = 0;
  warmup_start_ms = now_ms;
  reset_calibration();
}

void imu_yaw_io_error(int init_error)
{
  yaw_state.io_errors++;
  yaw_state.io_ok = 0;
  yaw_state.init_error = init_error;
  if (yaw_state.state == IMU_READY || yaw_state.state == IMU_FAULT) {
    invalidate_yaw(IMU_REASON_SPI);
  } else {
    yaw_state.state = IMU_WAIT;
    reset_calibration();
  }
}

void imu_yaw_update(const float acceleration_mps2[3], const float gyro_deg_s[3], uint32_t now_ms)
{
  float norm_squared = 0.0f;
  int finite = 1;
  int still_candidate = 1;
  uint32_t interval_ms = yaw_state.samples ? now_ms - yaw_state.last_sample_ms : 0;
  yaw_state.interval_ms = interval_ms;
  if (interval_ms > yaw_state.max_interval_ms) yaw_state.max_interval_ms = interval_ms;
  yaw_state.last_sample_ms = now_ms;
  yaw_state.samples++;
  yaw_state.io_ok = 1;
  yaw_state.init_error = 0;
  for (int index = 0; index < 3; index++) {
    yaw_state.acceleration_mps2[index] = acceleration_mps2[index];
    yaw_state.angular_velocity_deg_s[index] = gyro_deg_s[index];
    norm_squared += acceleration_mps2[index] * acceleration_mps2[index];
    if (
      !isfinite(acceleration_mps2[index]) || !isfinite(gyro_deg_s[index]) ||
      fabsf(gyro_deg_s[index]) >= IMU_GYRO_LIMIT_DEG_S)
      finite = 0;
    if (fabsf(gyro_deg_s[index]) > IMU_CALIBRATION_MAX_DEG_S) still_candidate = 0;
  }
  yaw_state.acceleration_norm_mps2 = sqrtf(norm_squared);
  yaw_state.level =
    finite && isfinite(yaw_state.acceleration_norm_mps2) &&
    fabsf(yaw_state.acceleration_norm_mps2 - IMU_GRAVITY_MPS2) < IMU_GRAVITY_TOLERANCE_MPS2 &&
    acceleration_mps2[2] > IMU_LEVEL_MIN_COSINE * yaw_state.acceleration_norm_mps2;
  yaw_state.corrected_z_deg_s = gyro_deg_s[2] - yaw_state.bias_deg_s[2];

  if (yaw_state.state == IMU_FAULT) return;
  if (interval_ms >= IMU_MAX_GAP_MS) {
    yaw_state.gap_events++;
    if (yaw_state.state == IMU_READY) {
      invalidate_yaw(IMU_REASON_GAP);
      return;
    }
    reset_calibration();
  }
  if (yaw_state.state == IMU_WARMUP) {
    if (now_ms - warmup_start_ms < IMU_WARMUP_MS) return;
    yaw_state.state = IMU_CALIBRATING;
  }
  if (yaw_state.state == IMU_CALIBRATING) {
    if (!finite || !yaw_state.level || !still_candidate) {
      reset_calibration();
      return;
    }
    if (!yaw_state.calibration_samples) {
      calibration_start_ms = now_ms;
      for (int index = 0; index < 3; index++) {
        calibration_min[index] = gyro_deg_s[index];
        calibration_max[index] = gyro_deg_s[index];
      }
    }
    for (int index = 0; index < 3; index++) {
      if (gyro_deg_s[index] < calibration_min[index]) calibration_min[index] = gyro_deg_s[index];
      if (gyro_deg_s[index] > calibration_max[index]) calibration_max[index] = gyro_deg_s[index];
      if (calibration_max[index] - calibration_min[index] > IMU_CALIBRATION_SPREAD_DEG_S) {
        reset_calibration();
        return;
      }
    }
    for (int index = 0; index < 3; index++) calibration_sum[index] += gyro_deg_s[index];
    yaw_state.calibration_samples++;
    if (
      yaw_state.calibration_samples >= IMU_CALIBRATION_SAMPLES &&
      now_ms - calibration_start_ms >= IMU_CALIBRATION_MS) {
      for (int index = 0; index < 3; index++) {
        yaw_state.bias_deg_s[index] = calibration_sum[index] / yaw_state.calibration_samples;
      }
      yaw_state.corrected_z_deg_s = gyro_deg_s[2] - yaw_state.bias_deg_s[2];
      previous_rate_deg_s = yaw_state.corrected_z_deg_s;
      yaw_state.yaw_rad = 0.0f;
      yaw_state.reason = IMU_REASON_NONE;
      yaw_state.state = IMU_READY;
      yaw_state.valid = 1;
    }
    return;
  }
  if (yaw_state.state != IMU_READY) return;
  if (!finite) {
    invalidate_yaw(IMU_REASON_RANGE);
    return;
  }
  if (!yaw_state.level) {
    invalidate_yaw(IMU_REASON_LEVEL);
    return;
  }
  float next_yaw_rad = yaw_state.yaw_rad + 0.5f *
                                             (previous_rate_deg_s + yaw_state.corrected_z_deg_s) *
                                             interval_ms * 0.001f * IMU_DEG_TO_RAD;
  if (!isfinite(next_yaw_rad)) {
    invalidate_yaw(IMU_REASON_RANGE);
    return;
  }
  yaw_state.yaw_rad = next_yaw_rad;
  previous_rate_deg_s = yaw_state.corrected_z_deg_s;
}

void imu_yaw_get_state(ImuYawState * result)
{
  uint32_t now_ms;
  taskENTER_CRITICAL();
  *result = yaw_state;
  now_ms = HAL_GetTick();
  taskEXIT_CRITICAL();
  if (!result->samples || now_ms - result->last_sample_ms >= IMU_MAX_GAP_MS) {
    result->valid = 0;
    if (result->state == IMU_READY) {
      result->state = IMU_FAULT;
      result->reason = IMU_REASON_GAP;
    }
  }
}
