#include "motor_position.hpp"

#include "control_constants.hpp"

static float limit_value(float value, float maximum)
{
  if (value > maximum) return maximum;
  if (value < -maximum) return -maximum;
  return value;
}

float motor_position_output(float reference_rad, float angle_rad, float speed_rad_s)
{
  float target_speed_rad_s = limit_value(
    MOTOR_POSITION_GAIN * (reference_rad - angle_rad), MOTOR_POSITION_SPEED_LIMIT_RAD_S);
  return limit_value(
    MOTOR_SPEED_GAIN * (target_speed_rad_s - speed_rad_s), MOTOR_POSITION_OUTPUT_LIMIT_RAW);
}
