#ifndef MOTOR_MANUAL_HPP
#define MOTOR_MANUAL_HPP

#include "motor_control.h"

struct ManualState
{
  uint8_t ready;
  uint8_t candidate;
  uint8_t source;
  uint32_t candidate_ms;
  uint32_t still_ms;
  uint32_t takeovers;
  uint32_t releases;
  uint32_t conflicts;
};

void motor_manual_reset(void);
int motor_manual_update(
  const MotorState * motor, float yaw_rad, float yaw_rate_rad_s, float ratio, uint32_t elapsed_ms,
  float target_rad[2]);
void motor_manual_get_state(ManualState * result);

#endif
