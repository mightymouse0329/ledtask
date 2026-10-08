#ifndef MOTOR_POSITION_HPP
#define MOTOR_POSITION_HPP

void motor_position_reset(int index);
float motor_position_output(
  int index, float reference_rad, float angle_rad, float speed_rad_s, float dt_s);

#endif
