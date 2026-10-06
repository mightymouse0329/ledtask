#include "motor_reset.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

static ResetState reset_state;
static float entry_yaw_rad;
static float entry_target_rad[2];
static uint32_t progress_start_ms;

void motor_reset_cancel(void)
{
  reset_state.active = 0;
  reset_state.done = 0;
  reset_state.settled_ms = 0;
}

void motor_reset_invalidate(void)
{
  motor_reset_cancel();
  reset_state.calibrated = 0;
}

void motor_reset_capture(const MotorState * motor, float yaw_rad)
{
  motor_reset_cancel();
  reset_state.zero_yaw_rad = yaw_rad;
  for (int index = 0; index < 2; index++) {
    reset_state.zero_motor_rad[index] = motor->motor[index].relative_angle_rad;
  }
  reset_state.calibrated = 1;
}

void motor_reset_begin(const MotorState * motor, float yaw_rad, uint32_t now_ms)
{
  const int directions[2] = {MOTOR_A_DIRECTION, MOTOR_B_DIRECTION};
  motor_reset_cancel();
  if (!reset_state.calibrated) return;
  entry_yaw_rad = yaw_rad;
  progress_start_ms = now_ms;
  for (int index = 0; index < 2; index++) {
    float current_rad = motor->motor[index].relative_angle_rad;
    float aligned_rad =
      reset_state.zero_motor_rad[index] + directions[index] * (yaw_rad - reset_state.zero_yaw_rad);
    float difference_rad = remainderf(aligned_rad - current_rad, MOTOR_TWO_PI);
    if (difference_rad <= -MOTOR_TWO_PI * 0.5f) difference_rad += MOTOR_TWO_PI;
    entry_target_rad[index] = current_rad + difference_rad;
    reset_state.target_rad[index] = entry_target_rad[index];
  }
  reset_state.active = 1;
}

int motor_reset_update(
  const MotorState * motor, float yaw_rad, uint32_t now_ms, uint32_t elapsed_ms,
  float target_rad[2])
{
  const int directions[2] = {MOTOR_A_DIRECTION, MOTOR_B_DIRECTION};
  int settled = 1;
  if (!reset_state.calibrated || !reset_state.active) return 0;
  for (int index = 0; index < 2; index++) {
    target_rad[index] = entry_target_rad[index] + directions[index] * (yaw_rad - entry_yaw_rad);
    reset_state.target_rad[index] = target_rad[index];
    if (
      !isfinite(target_rad[index]) ||
      fabsf(target_rad[index] - motor->motor[index].relative_angle_rad) > MOTOR_RESET_ERROR_RAD ||
      fabsf(motor->motor[index].speed_rpm * MOTOR_TWO_PI / 60.0f) > MOTOR_RESET_SPEED_RAD_S) {
      settled = 0;
    }
  }
  if (settled) {
    if (reset_state.settled_ms < MOTOR_RESET_SETTLE_MS) reset_state.settled_ms += elapsed_ms;
    reset_state.done = reset_state.settled_ms >= MOTOR_RESET_SETTLE_MS;
    if (reset_state.done) progress_start_ms = now_ms;
  } else {
    reset_state.settled_ms = 0;
    reset_state.done = 0;
  }
  if (!reset_state.done && now_ms - progress_start_ms >= MOTOR_RESET_TIMEOUT_MS) return 0;
  return 1;
}

void motor_reset_get_state(ResetState * result)
{
  taskENTER_CRITICAL();
  *result = reset_state;
  taskEXIT_CRITICAL();
}
