#include "motor_manual.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

static ManualState manual;
static uint32_t settling_ms;
static float quiet_yaw_rad;
static float quiet_target_rad[2];
static float quiet_actual_rad[2];
static float source_start_target_rad[2];
static float source_start_yaw_rad;
static float source_ratio;
static float still_angle_rad;

void motor_manual_reset(void)
{
  manual.ready = 0;
  manual.candidate = 0;
  manual.source = 0;
  manual.candidate_ms = 0;
  manual.still_ms = 0;
  settling_ms = 0;
}

static int input_conflict(void)
{
  manual.conflicts++;
  motor_manual_reset();
  return -1;
}

static float motor_speed_rad_s(const MotorState * motor, int index)
{
  return motor->motor[index].speed_rpm * MOTOR_TWO_PI / 60.0f;
}

int motor_manual_update(
  const MotorState * motor, float yaw_rad, float yaw_rate_rad_s, float ratio, uint32_t elapsed_ms,
  float target_rad[2])
{
  int candidate = 0;
  if (!isfinite(yaw_rad) || !isfinite(yaw_rate_rad_s) || !isfinite(ratio) || ratio == 0.0f) {
    return input_conflict();
  }

  if (!manual.source) {
    if (fabsf(yaw_rate_rad_s) > MOTOR_MANUAL_YAW_QUIET_RAD_S) {
      motor_manual_reset();
      return 0;
    }
    if (!manual.ready) {
      if (!settling_ms) {
        quiet_yaw_rad = yaw_rad;
        for (int index = 0; index < 2; index++) {
          quiet_target_rad[index] = target_rad[index];
          quiet_actual_rad[index] = motor->motor[index].relative_angle_rad;
        }
      }
      for (int index = 0; index < 2; index++) {
        if (
          fabsf(motor->motor[index].relative_angle_rad - target_rad[index]) >
            MOTOR_MANUAL_SETTLED_ERROR_RAD ||
          fabsf(motor_speed_rad_s(motor, index)) > MOTOR_MANUAL_SETTLE_SPEED_RAD_S ||
          fabsf(motor->motor[index].relative_angle_rad - quiet_actual_rad[index]) >
            MOTOR_MANUAL_STILL_TRAVEL_RAD ||
          fabsf(target_rad[index] - quiet_target_rad[index]) > MOTOR_MANUAL_TARGET_QUIET_RAD ||
          fabsf(yaw_rad - quiet_yaw_rad) > MOTOR_MANUAL_YAW_QUIET_RAD) {
          motor_manual_reset();
          return 0;
        }
      }
      settling_ms += elapsed_ms;
      if (settling_ms >= MOTOR_MANUAL_SETTLE_MS) manual.ready = 1;
      return 0;
    }

    for (int index = 0; index < 2; index++) {
      if (
        fabsf(target_rad[index] - quiet_target_rad[index]) > MOTOR_MANUAL_TARGET_QUIET_RAD ||
        fabsf(yaw_rad - quiet_yaw_rad) > MOTOR_MANUAL_YAW_QUIET_RAD) {
        motor_manual_reset();
        return 0;
      }
      if (
        fabsf(motor->motor[index].relative_angle_rad - target_rad[index]) >
        MOTOR_MANUAL_TRIGGER_RAD) {
        if (candidate) return input_conflict();
        candidate = index + 1;
      }
    }
    if (!candidate) {
      manual.candidate = 0;
      manual.candidate_ms = 0;
      return 0;
    }
    if (manual.candidate != candidate) {
      manual.candidate = candidate;
      manual.candidate_ms = 0;
    }
    manual.candidate_ms += elapsed_ms;
    if (manual.candidate_ms < MOTOR_MANUAL_TRIGGER_MS) return 0;

    manual.source = candidate;
    manual.ready = 0;
    manual.takeovers++;
    source_start_yaw_rad = yaw_rad;
    source_ratio = ratio;
    for (int index = 0; index < 2; index++) source_start_target_rad[index] = target_rad[index];
    still_angle_rad = motor->motor[candidate - 1].relative_angle_rad;
  }

  if (
    ratio != source_ratio || fabsf(yaw_rad - source_start_yaw_rad) > MOTOR_MANUAL_YAW_QUIET_RAD ||
    fabsf(yaw_rate_rad_s) > MOTOR_MANUAL_YAW_QUIET_RAD_S) {
    return input_conflict();
  }
  int source_index = manual.source - 1;
  float source_angle_rad = motor->motor[source_index].relative_angle_rad;
  float common_change_rad;
  if (source_index == 0) {
    common_change_rad = MOTOR_A_DIRECTION * (source_angle_rad - source_start_target_rad[0]);
    target_rad[0] = source_angle_rad;
    target_rad[1] = source_start_target_rad[1] + MOTOR_B_DIRECTION * ratio * common_change_rad;
  } else {
    common_change_rad = MOTOR_B_DIRECTION * (source_angle_rad - source_start_target_rad[1]) / ratio;
    target_rad[1] = source_angle_rad;
    target_rad[0] = source_start_target_rad[0] + MOTOR_A_DIRECTION * common_change_rad;
  }

  if (fabsf(source_angle_rad - still_angle_rad) > MOTOR_MANUAL_STILL_TRAVEL_RAD) {
    manual.still_ms = 0;
    still_angle_rad = source_angle_rad;
  } else {
    manual.still_ms += elapsed_ms;
  }
  if (manual.still_ms >= MOTOR_MANUAL_RELEASE_MS) {
    manual.releases++;
    motor_manual_reset();
  }
  return 1;
}

void motor_manual_get_state(ManualState * result)
{
  taskENTER_CRITICAL();
  *result = manual;
  taskEXIT_CRITICAL();
}
