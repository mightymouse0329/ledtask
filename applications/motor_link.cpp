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
static uint32_t speed_trip_cycles[2];
static uint8_t motor_flags[2];
static uint8_t motor_offline[2];

static float limit_value(float value, float maximum)
{
  if (value > maximum) return maximum;
  if (value < -maximum) return -maximum;
  return value;
}

static float open_loop_current_raw(const RemoteState * remote)
{
  int stick = remote->channel[3];
  if (stick > MOTOR_TEST_STICK_DEADBAND) {
    stick -= MOTOR_TEST_STICK_DEADBAND;
  } else if (stick < -MOTOR_TEST_STICK_DEADBAND) {
    stick += MOTOR_TEST_STICK_DEADBAND;
  } else {
    return 0.0f;
  }
  float fraction =
    (float)stick / (float)(MOTOR_TEST_STICK_FULL_SCALE - MOTOR_TEST_STICK_DEADBAND);
  return limit_value(fraction, 1.0f) * MOTOR_TEST_CURRENT_LIMIT_A * MOTOR_CURRENT_RAW_PER_AMP;
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
  speed_trip_cycles[0] = 0;
  speed_trip_cycles[1] = 0;
  link_state.block = reason;
  if (!link_state.fault) link_state.fault = reason;
  link_state.output_raw[0] = 0;
  link_state.output_raw[1] = 0;
  link_state.target_speed_rad_s[0] = 0.0f;
  link_state.target_speed_rad_s[1] = 0.0f;
}

static void unlock_motor(const MotorState * motor, const ImuYawState * imu)
{
  ResetState reset;
  motor_reset_get_state(&reset);
  if (!reset.calibrated) motor_reset_capture(motor, imu->yaw_rad);
  link_state.unlocked = 1;
  link_state.ready = 1;
  link_state.fault = 0;
  link_state.fault_motor = 0;
  link_state.fault_flags = 0;
  link_state.fault_speed_rpm = 0;
  link_state.fault_temperature = 0;
  link_state.fault_age_ms = 0;
  link_state.fault_previous_output_raw[0] = 0;
  link_state.fault_previous_output_raw[1] = 0;
  link_state.fault_current_raw = 0;
  link_state.fault_encoder_raw = 0;
  link_state.fault_angle_rad = 0.0f;
}

void motor_link_update(
  const MotorState * motor, const RemoteState * remote, const ImuYawState * imu, uint32_t now_ms,
  MotorOutput * output)
{
  uint32_t elapsed_ms = now_ms - last_update_ms;
  int new_error = motor->error_events != previous_errors;
  float ratio = switch_ratio(remote->left_switch);
  int16_t next_output[2];
  float reference_rate_rad_s[2] = {0.0f, 0.0f};
  float speed_cap_rad_s = MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S;
  int16_t previous_output[2] = {
    link_state.output_raw[0],
    link_state.output_raw[1],
  };
  last_update_ms = now_ms;
  previous_errors = motor->error_events;
  if (!initialized) {
    elapsed_ms = 0;
    initialized = 1;
  }
  output->command_raw[0] = 0;
  output->command_raw[1] = 0;
  link_state.block = LINK_FAULT_NONE;
  link_state.motor_flags[0] = 0;
  link_state.motor_flags[1] = 0;
  link_state.output_raw[0] = 0;
  link_state.output_raw[1] = 0;
  link_state.target_speed_rad_s[0] = 0.0f;
  link_state.target_speed_rad_s[1] = 0.0f;

  int open_loop =
    (link_state.mode == RC_MODE_CURRENT_TEST || link_state.mode == RC_MODE_SPIN_TEST);
  if (
    !open_loop &&
    (!imu->valid || imu->state != IMU_READY || !isfinite(imu->yaw_rad) ||
     !isfinite(imu->corrected_z_deg_s) || now_ms - imu->last_sample_ms >= IMU_MAX_GAP_MS)) {
    motor_reset_invalidate();
    motor_link_stop(LINK_FAULT_IMU);
    return;
  }

  float trip_speed_rad_s = (link_state.mode == RC_MODE_SPIN_TEST)
                             ? MOTOR_SPIN_TEST_TRIP_SPEED_RAD_S
                             : MOTOR_POSITION_TRIP_SPEED_RAD_S;
  for (int index = 0; index < 2; index++) {
    uint32_t age_ms = now_ms - motor->motor[index].last_feedback_ms;
    uint8_t flags = 0;
    if (!motor->motor[index].online) flags |= 1;
    if (!motor->motor[index].angle_valid) flags |= 2;
    if (age_ms >= MOTOR_ANGLE_MAX_GAP_MS) flags |= 4;
    if (!isfinite(motor->motor[index].relative_angle_rad)) flags |= 8;
    if (motor->motor[index].temperature_deg_c >= MOTOR_POSITION_MAX_TEMP_DEG_C) flags |= 16;
    if (fabsf(motor->motor[index].speed_rpm * MOTOR_TWO_PI / 60.0f) > trip_speed_rad_s) {
      speed_trip_cycles[index]++;
    } else {
      speed_trip_cycles[index] = 0;
    }
    if (speed_trip_cycles[index] >= MOTOR_POSITION_TRIP_SPEED_DEBOUNCE_CYCLES) flags |= 32;
    link_state.motor_flags[index] = flags;
    motor_flags[index] = flags;
    motor_offline[index] = 0;
  }
  int healthy_motors = (motor_flags[0] == 0) + (motor_flags[1] == 0);
  for (int index = 0; index < 2; index++) {
    if (!motor_flags[index]) continue;

    if (
      MOTOR_ALLOW_SINGLE_ONLINE && healthy_motors >= 1 &&
      (motor_flags[index] & (uint8_t)~7u) == 0) {
      motor_offline[index] = 1;
      continue;
    }
    {
      uint32_t age_ms = now_ms - motor->motor[index].last_feedback_ms;
      if (!link_state.fault) {
        link_state.fault_motor = index + 1;
        link_state.fault_flags = motor_flags[index];
        link_state.fault_speed_rpm = motor->motor[index].speed_rpm;
        link_state.fault_temperature = motor->motor[index].temperature_deg_c;
        link_state.fault_age_ms = age_ms;
        link_state.fault_previous_output_raw[0] = previous_output[0];
        link_state.fault_previous_output_raw[1] = previous_output[1];
        link_state.fault_current_raw = motor->motor[index].current_raw;
        link_state.fault_encoder_raw = motor->motor[index].encoder_raw;
        link_state.fault_angle_rad = motor->motor[index].relative_angle_rad;
      }
    }
    if (motor_flags[index] & 15) motor_reset_invalidate();
    motor_link_stop(LINK_FAULT_FEEDBACK);
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
  } else if (
    MOTOR_BENCH_TEST_MODES && remote->right_switch == RC_SWITCH_UP &&
    remote->mode == RC_MODE_RESET && remote->left_switch == RC_SWITCH_UP) {
    requested_mode = RC_MODE_SPIN_TEST;
  } else if (
    MOTOR_BENCH_TEST_MODES && remote->right_switch == RC_SWITCH_UP &&
    remote->mode == RC_MODE_RESET && remote->left_switch == RC_SWITCH_DOWN) {
    requested_mode = RC_MODE_CURRENT_TEST;
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

    float yaw_rate_rad_s = imu->corrected_z_deg_s * IMU_DEG_TO_RAD;
    reference_rate_rad_s[0] = MOTOR_A_DIRECTION * yaw_rate_rad_s;
    reference_rate_rad_s[1] = MOTOR_B_DIRECTION * yaw_rate_rad_s;
    speed_cap_rad_s = MOTOR_RESET_MAX_TARGET_SPEED_RAD_S;
  } else if (link_state.mode == RC_MODE_SPIN_TEST) {
    for (int index = 0; index < 2; index++) {
      link_state.target_rad[index] = motor->motor[index].relative_angle_rad;
      link_state.reference_rad[index] = link_state.target_rad[index];
    }
  } else if (link_state.mode == RC_MODE_CURRENT_TEST) {
    for (int index = 0; index < 2; index++) {
      link_state.target_rad[index] = motor->motor[index].relative_angle_rad;
      link_state.reference_rad[index] = link_state.target_rad[index];
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
    float yaw_rate_rad_s = imu->corrected_z_deg_s * IMU_DEG_TO_RAD;
    reference_rate_rad_s[0] = MOTOR_A_DIRECTION * yaw_rate_rad_s;
    reference_rate_rad_s[1] = MOTOR_B_DIRECTION * link_state.ratio * yaw_rate_rad_s;
  }
  ManualState manual;
  motor_manual_get_state(&manual);
  if (link_state.mode == RC_MODE_LINK && manual.source) {

    int source = manual.source - 1;
    float source_speed_rad_s = motor->motor[source].speed_rad_s;
    if (source == 0) {
      reference_rate_rad_s[1] = (float)MOTOR_B_DIRECTION * link_state.ratio *
                                (float)MOTOR_A_DIRECTION * source_speed_rad_s;
    } else {
      reference_rate_rad_s[0] = (float)MOTOR_A_DIRECTION * (float)MOTOR_B_DIRECTION *
                                source_speed_rad_s / link_state.ratio;
    }
    reference_rate_rad_s[source] = 0.0f;
  }
  float test_current_raw = open_loop_current_raw(remote);
  for (int index = 0; index < 2; index++) {
    float angle_rad = motor->motor[index].relative_angle_rad;
    float speed_rad_s = motor->motor[index].speed_rad_s;
    if (!isfinite(link_state.target_rad[index])) {
      motor_link_stop(LINK_FAULT_LIMIT);
      return;
    }
    if (motor_offline[index]) {
      motor_position_reset(index);
      link_state.reference_rad[index] = angle_rad;
      link_state.target_speed_rad_s[index] = 0.0f;
      next_output[index] = 0;
      continue;
    }
    if (link_state.mode == RC_MODE_CURRENT_TEST) {

      motor_position_reset(index);
      link_state.reference_rad[index] = angle_rad;
      link_state.target_speed_rad_s[index] = 0.0f;
      next_output[index] = (int16_t)limit_value(test_current_raw, MOTOR_TEST_OUTPUT_LIMIT_RAW);
      continue;
    }
    if (link_state.mode == RC_MODE_SPIN_TEST) {

      link_state.reference_rad[index] = angle_rad;
      float output = motor_position_spin_output(index, MOTOR_SPIN_TEST_TARGET_RAD_S, speed_rad_s);
      link_state.target_speed_rad_s[index] = motor_position_target_speed(index);
      if (!isfinite(output)) {
        motor_link_stop(LINK_FAULT_LIMIT);
        return;
      }
      next_output[index] = (int16_t)limit_value(output, MOTOR_POSITION_OUTPUT_LIMIT_RAW);
      continue;
    }
    if (link_state.mode == RC_MODE_LINK && manual.source == index + 1) {
      motor_position_reset(index);
      link_state.reference_rad[index] = angle_rad;
      link_state.target_speed_rad_s[index] = 0.0f;
      next_output[index] = 0;
      continue;
    }
    link_state.reference_rad[index] = link_state.target_rad[index];
    float output = motor_position_output(
      index, link_state.reference_rad[index], reference_rate_rad_s[index], speed_cap_rad_s,
      angle_rad, speed_rad_s, elapsed_ms / 1000.0f);
    link_state.target_speed_rad_s[index] = motor_position_target_speed(index);
    if (!isfinite(output)) {
      motor_link_stop(LINK_FAULT_LIMIT);
      return;
    }
    next_output[index] = (int16_t)limit_value(output, MOTOR_POSITION_OUTPUT_LIMIT_RAW);
  }
  for (int index = 0; index < 2; index++) {
    output->command_raw[index] = next_output[index];
    link_state.output_raw[index] = next_output[index];
  }
}

void motor_link_get_state(LinkState * result)
{
  taskENTER_CRITICAL();
  *result = link_state;
  taskEXIT_CRITICAL();
}
