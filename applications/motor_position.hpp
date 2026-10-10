#ifndef MOTOR_POSITION_HPP
#define MOTOR_POSITION_HPP

void motor_position_reset(int index);
float motor_position_target_speed(int index);
// Spin test (mode 3): hold a fixed target speed with the same speed PID and current limit as
// the linkage inner loop, using the encoder feedback. Returns raw current for frame 0x1FE.
float motor_position_spin_output(int index, float target_speed_rad_s, float speed_rad_s);
float motor_position_output(
  int index, float reference_rad, float reference_rate_rad_s, float max_target_speed_rad_s,
  float angle_rad, float speed_rad_s, float dt_s);

#endif
