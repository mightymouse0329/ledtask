#ifndef MOTOR_POSITION_HPP
#define MOTOR_POSITION_HPP

#include "motor_control.h"
#include "remote_control.h"

struct PositionState
{
  uint8_t active;
  uint8_t ready;
  uint8_t fault;
  float reference_rad;
  float target_rad;
  float speed_target_rad_s;
  int16_t output_raw;
};

int16_t motor_position_update(
  const MotorState * motor, const RemoteState * remote, uint32_t now_ms);
void motor_position_stop(void);
void motor_position_get_state(PositionState * result);

#endif
