#ifndef MOTOR_RESET_HPP
#define MOTOR_RESET_HPP

#include "motor_control.h"

struct ResetState
{
  uint8_t calibrated;
  uint8_t active;
  uint8_t done;
  uint32_t settled_ms;
  float zero_yaw_rad;
  float zero_motor_rad[2];
  float target_rad[2];
};

void motor_reset_capture(const MotorState * motor, float yaw_rad);
void motor_reset_cancel(void);
void motor_reset_invalidate(void);
void motor_reset_begin(const MotorState * motor, float yaw_rad, uint32_t now_ms);
int motor_reset_update(
  const MotorState * motor, float yaw_rad, uint32_t now_ms, uint32_t elapsed_ms,
  float target_rad[2]);
void motor_reset_get_state(ResetState * result);

#endif
