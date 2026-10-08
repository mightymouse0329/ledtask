#include "motor_link.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "motor_manual.hpp"
#include "motor_position.hpp"
#include "motor_reset.hpp"
#include "task.h"

static LinkState link_state;
static float entry_angle_rad[2];
static float entry_yaw_rad;
static float reference_yaw_rad;
static float reference_motor_rad;
static float ratio_yaw_rad;
static float ratio_motor_rad;
static uint32_t last_update_ms;
static uint32_t previous_errors;
static uint8_t initialized;
static uint8_t unlock_armed;

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
  motor_position_reset(0);
  motor_position_reset(1);
  motor_manual_reset();
  motor_reset_cancel();
  link_state.active = 0;
  link_state.ready = 0;
  link_state.unlocked = 0;
  link_state.mode = RC_MODE_DISABLED;
  unlock_armed = 0;
  if (!link_state.fault) link_state.fault = reason;
  link_state.output_raw[0] = 0;
  link_state.output_raw[1] = 0;
}

static void unlock_motor(const MotorState * motor, const ImuYawState * imu)
{
  ResetState reset;
  motor_reset_get_state(&reset);
  if (!reset.calibrated) motor_reset_capture(motor, imu->yaw_rad);
  link_state.unlocked = 1;
  link_state.ready = 1;
  link_state.fault = 0;
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

  if (
    !imu->valid || imu->state != IMU_READY || !isfinite(imu->yaw_rad) ||
    !isfinite(imu->corrected_z_deg_s) || now_ms - imu->last_sample_ms >= IMU_MAX_GAP_MS) {
    motor_reset_invalidate();
    motor_link_stop(LINK_FAULT_IMU);
    return;
  }
  for (int index = 0; index < 2; index++) {
    if (
      !motor->motor[index].online || !motor->motor[index].angle_valid ||
      now_ms - motor->motor[index].last_feedback_ms >= MOTOR_ANGLE_MAX_GAP_MS ||
      !isfinite(motor->motor[index].relative_angle_rad)) {
      motor_reset_invalidate();
      motor_link_stop(LINK_FAULT_FEEDBACK);
      return;
    }
    if (
      motor->motor[index].temperature_deg_c >= MOTOR_POSITION_MAX_TEMP_DEG_C ||
      fabsf(motor->motor[index].speed_rpm * MOTOR_TWO_PI / 60.0f) >
        MOTOR_POSITION_TRIP_SPEED_RAD_S) {
      motor_link_stop(LINK_FAULT_FEEDBACK);
      return;
    }
  }
  if (!motor->started || motor->bus_off || new_error) {
    motor_link_stop(LINK_FAULT_CAN);
    return;
  }
  if (!remote->online || !remote->armed) {
    motor_link_stop(LINK_FAULT_REMOTE);
    return;
  }
  if (link_state.active && elapsed_ms > MOTOR_POSITION_MAX_INTERVAL_MS) {
    motor_link_stop(LINK_FAULT_TIMING);
    return;
  }

  if (remote->right_switch == RC_SWITCH_DOWN) {
    motor_position_reset(0);
    motor_position_reset(1);
    motor_manual_reset();
    motor_reset_cancel();
    link_state.active = 0;
    link_state.mode = RC_MODE_DISABLED;
    link_state.ratio = ratio;
    link_state.yaw_reference_rad =
      imu->yaw_rad - MOTOR_A_DIRECTION * motor->motor[0].relative_angle_rad;
    link_state.yaw_change_rad = 0.0f;
    for (int index = 0; index < 2; index++) {
      link_state.target_rad[index] = motor->motor[index].relative_angle_rad;
      link_state.reference_rad[index] = link_state.target_rad[index];
    }
    if (!link_state.unlocked) unlock_armed = 1;
    link_state.ready = link_state.unlocked;
    return;
  }
  if (!link_state.unlocked && unlock_armed) {
    unlock_motor(motor, imu);
    unlock_armed = 0;
  }
  if (!link_state.unlocked || link_state.fault) return;
  int requested_mode;
  if (remote->right_switch == RC_SWITCH_MID && remote->mode == RC_MODE_LINK) {
    requested_mode = RC_MODE_LINK;
  } else if (remote->right_switch == RC_SWITCH_UP && remote->mode == RC_MODE_RESET) {
    requested_mode = RC_MODE_RESET;
  } else {
    motor_link_stop(LINK_FAULT_REMOTE);
    return;
  }
  if (!link_state.active || link_state.mode != requested_mode) {
    motor_position_reset(0);
    motor_position_reset(1);
    motor_manual_reset();
    motor_reset_cancel();
    entry_yaw_rad = imu->yaw_rad;
    reference_yaw_rad = imu->yaw_rad;
    reference_motor_rad = motor->motor[0].relative_angle_rad;
    link_state.yaw_reference_rad = imu->yaw_rad - MOTOR_A_DIRECTION * reference_motor_rad;
    ratio_yaw_rad = imu->yaw_rad;
    link_state.ratio = ratio;
    ratio_motor_rad = motor->motor[1].relative_angle_rad;
    for (int index = 0; index < 2; index++) {
      entry_angle_rad[index] = motor->motor[index].relative_angle_rad;
      link_state.target_rad[index] = entry_angle_rad[index];
      link_state.reference_rad[index] = entry_angle_rad[index];
    }
    link_state.yaw_change_rad = 0.0f;
    link_state.mode = requested_mode;
    link_state.active = 1;
    if (requested_mode == RC_MODE_RESET) motor_reset_begin(motor, imu->yaw_rad, now_ms);
    return;
  }
  if (link_state.mode == RC_MODE_RESET) {
    if (!motor_reset_update(motor, imu->yaw_rad, now_ms, elapsed_ms, link_state.target_rad)) {
      motor_link_stop(LINK_FAULT_RESET);
      return;
    }
  } else {
    link_state.yaw_change_rad = imu->yaw_rad - entry_yaw_rad;
    link_state.target_rad[0] =
      reference_motor_rad + MOTOR_A_DIRECTION * (imu->yaw_rad - reference_yaw_rad);
    link_state.target_rad[1] =
      ratio_motor_rad + MOTOR_B_DIRECTION * link_state.ratio * (imu->yaw_rad - ratio_yaw_rad);
    if (ratio != link_state.ratio) {
      ratio_motor_rad = link_state.target_rad[1];
      ratio_yaw_rad = imu->yaw_rad;
      link_state.ratio = ratio;
    }
    int manual_result = motor_manual_update(
      motor, imu->yaw_rad, imu->corrected_z_deg_s * IMU_DEG_TO_RAD, ratio, elapsed_ms,
      link_state.target_rad);
    if (manual_result < 0) {
      motor_link_stop(LINK_FAULT_MANUAL);
      return;
    }
    if (manual_result > 0) {
      reference_yaw_rad = imu->yaw_rad;
      reference_motor_rad = link_state.target_rad[0];
      ratio_yaw_rad = imu->yaw_rad;
      ratio_motor_rad = link_state.target_rad[1];
    }
    link_state.yaw_reference_rad = reference_yaw_rad - MOTOR_A_DIRECTION * reference_motor_rad;
  }
  ManualState manual;
  motor_manual_get_state(&manual);
  reference_step = MOTOR_POSITION_RAMP_RAD_S * elapsed_ms / 1000.0f;
  for (int index = 0; index < 2; index++) {
    float angle_rad = motor->motor[index].relative_angle_rad;
    float speed_rad_s = motor->motor[index].speed_rpm * MOTOR_TWO_PI / 60.0f;
    if (
      !isfinite(link_state.target_rad[index]) ||
      fabsf(angle_rad - entry_angle_rad[index]) > MOTOR_LINK_TRAVEL_LIMIT_RAD ||
      fabsf(link_state.target_rad[index] - entry_angle_rad[index]) > MOTOR_LINK_TRAVEL_LIMIT_RAD ||
      fabsf(
        (link_state.mode == RC_MODE_RESET ? link_state.reference_rad[index]
                                          : link_state.target_rad[index]) -
        angle_rad) > MOTOR_LINK_MAX_ERROR_RAD) {
      motor_link_stop(LINK_FAULT_LIMIT);
      return;
    }
    if (link_state.mode == RC_MODE_LINK && manual.source == index + 1) {
      motor_position_reset(index);
      link_state.reference_rad[index] = angle_rad;
      next_output[index] = 0;
      continue;
    }
    link_state.reference_rad[index] +=
      limit_value(link_state.target_rad[index] - link_state.reference_rad[index], reference_step);
    float output = motor_position_output(
      index, link_state.reference_rad[index], angle_rad, speed_rad_s, elapsed_ms / 1000.0f);
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
