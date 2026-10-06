#include "motor_link.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

static LinkState link_state;
static float entry_angle_rad[2];
static float entry_yaw_rad;
static float ratio_yaw_rad;
static float ratio_motor_rad;
static uint32_t last_update_ms;
static uint32_t previous_errors;
static uint8_t initialized;

static float limit_value(float value, float maximum)
{
  if (value > maximum) return maximum;
  if (value < -maximum) return -maximum;
  return value;
}

static float switch_ratio(int value)
{
  if (value == RC_SWITCH_UP) return 3.0f;
  if (value == RC_SWITCH_MID) return -1.0f;
  return 0.5f;
}

void motor_link_stop(int reason)
{
  link_state.active = 0;
  link_state.ready = 0;
  if (!link_state.fault) link_state.fault = reason;
  link_state.output_raw[0] = 0;
  link_state.output_raw[1] = 0;
}

void motor_link_update(
  const MotorState * motor, const RemoteState * remote, const ImuYawState * imu, uint32_t now_ms,
  int16_t commands[2])
{
  uint32_t elapsed_ms = now_ms - last_update_ms;
  int new_error = motor->error_events != previous_errors;
  float ratio = switch_ratio(remote->left_switch);
  float reference_step;
  int16_t next_output[2];
  last_update_ms = now_ms;
  previous_errors = motor->error_events;
  if (!initialized) {
    elapsed_ms = 0;
    initialized = 1;
  }
  commands[0] = 0;
  commands[1] = 0;
  link_state.output_raw[0] = 0;
  link_state.output_raw[1] = 0;
  if (!MOTOR_LINK_ENABLE) {
    link_state.active = 0;
    link_state.ready = 0;
    link_state.fault = 0;
    return;
  }
  if (!motor->started || motor->bus_off || new_error) {
    motor_link_stop(LINK_FAULT_CAN);
    return;
  }
  if (!remote->online || !remote->armed) {
    motor_link_stop(LINK_FAULT_REMOTE);
    return;
  }
  if (
    !imu->valid || imu->state != IMU_READY || !isfinite(imu->yaw_rad) ||
    now_ms - imu->last_sample_ms >= IMU_MAX_GAP_MS) {
    motor_link_stop(LINK_FAULT_IMU);
    return;
  }
  for (int index = 0; index < 2; index++) {
    if (
      !motor->motor[index].online || !motor->motor[index].angle_valid ||
      now_ms - motor->motor[index].last_feedback_ms >= MOTOR_ANGLE_MAX_GAP_MS ||
      !isfinite(motor->motor[index].relative_angle_rad) ||
      motor->motor[index].temperature_deg_c >= MOTOR_POSITION_MAX_TEMP_DEG_C ||
      fabsf(motor->motor[index].speed_rpm * MOTOR_TWO_PI / 60.0f) >
        MOTOR_POSITION_TRIP_SPEED_RAD_S) {
      motor_link_stop(LINK_FAULT_FEEDBACK);
      return;
    }
  }
  if (link_state.active && elapsed_ms > MOTOR_POSITION_MAX_INTERVAL_MS) {
    motor_link_stop(LINK_FAULT_TIMING);
    return;
  }
  if (remote->right_switch == RC_SWITCH_DOWN) {
    link_state.active = 0;
    link_state.ready = 1;
    link_state.fault = 0;
    link_state.ratio = ratio;
    link_state.yaw_change_rad = 0.0f;
    for (int index = 0; index < 2; index++) {
      link_state.target_rad[index] = motor->motor[index].relative_angle_rad;
      link_state.reference_rad[index] = link_state.target_rad[index];
    }
    return;
  }
  if (remote->right_switch != RC_SWITCH_MID || remote->mode != RC_MODE_LINK) {
    link_state.active = 0;
    link_state.ready = 0;
    return;
  }
  if (link_state.fault || !link_state.ready) return;
  if (!link_state.active) {
    entry_yaw_rad = imu->yaw_rad;
    ratio_yaw_rad = imu->yaw_rad;
    link_state.ratio = ratio;
    ratio_motor_rad = motor->motor[1].relative_angle_rad;
    for (int index = 0; index < 2; index++) {
      entry_angle_rad[index] = motor->motor[index].relative_angle_rad;
      link_state.target_rad[index] = entry_angle_rad[index];
      link_state.reference_rad[index] = entry_angle_rad[index];
    }
    link_state.yaw_change_rad = 0.0f;
    link_state.active = 1;
    return;
  }

  link_state.yaw_change_rad = imu->yaw_rad - entry_yaw_rad;
  link_state.target_rad[0] = entry_angle_rad[0] + MOTOR_A_DIRECTION * link_state.yaw_change_rad;
  link_state.target_rad[1] =
    ratio_motor_rad + MOTOR_B_DIRECTION * link_state.ratio * (imu->yaw_rad - ratio_yaw_rad);
  if (ratio != link_state.ratio) {
    ratio_motor_rad = link_state.target_rad[1];
    ratio_yaw_rad = imu->yaw_rad;
    link_state.ratio = ratio;
  }
  reference_step = MOTOR_POSITION_RAMP_RAD_S * elapsed_ms / 1000.0f;
  for (int index = 0; index < 2; index++) {
    float angle_rad = motor->motor[index].relative_angle_rad;
    float speed_rad_s = motor->motor[index].speed_rpm * MOTOR_TWO_PI / 60.0f;
    if (
      !isfinite(link_state.target_rad[index]) ||
      fabsf(angle_rad - entry_angle_rad[index]) > MOTOR_LINK_TRAVEL_LIMIT_RAD ||
      fabsf(link_state.target_rad[index] - entry_angle_rad[index]) > MOTOR_LINK_TRAVEL_LIMIT_RAD ||
      fabsf(link_state.target_rad[index] - angle_rad) > MOTOR_LINK_MAX_ERROR_RAD) {
      motor_link_stop(LINK_FAULT_LIMIT);
      return;
    }
    link_state.reference_rad[index] +=
      limit_value(link_state.target_rad[index] - link_state.reference_rad[index], reference_step);
    float target_speed_rad_s = limit_value(
      MOTOR_POSITION_GAIN * (link_state.reference_rad[index] - angle_rad),
      MOTOR_POSITION_SPEED_LIMIT_RAD_S);
    float output = MOTOR_SPEED_GAIN * (target_speed_rad_s - speed_rad_s);
    if (!isfinite(output)) {
      motor_link_stop(LINK_FAULT_LIMIT);
      return;
    }
    next_output[index] = (int16_t)limit_value(output, MOTOR_POSITION_OUTPUT_LIMIT_RAW);
  }
  for (int index = 0; index < 2; index++) {
    commands[index] = next_output[index];
    link_state.output_raw[index] = next_output[index];
  }
}

void motor_link_get_state(LinkState * result)
{
  taskENTER_CRITICAL();
  *result = link_state;
  taskEXIT_CRITICAL();
}
