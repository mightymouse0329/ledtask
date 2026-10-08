#include "motor_position.hpp"

#include <math.h>
#include "control_constants.hpp"

struct SpeedPidState
{
  float integral_a;
  float previous_speed;
  float filtered_acceleration;
  int initialized;
};

static SpeedPidState speed_pid[2];

static float limit_value(float value, float maximum)
{
  if (value > maximum) return maximum;
  if (value < -maximum) return -maximum;
  return value;
}

void motor_position_reset(int index)
{
  if (index < 0 || index >= 2) return;
  speed_pid[index] = {};
}

float motor_position_output(
  int index, float reference_rad, float angle_rad, float speed_rad_s, float dt_s)
{
  if (index < 0 || index >= 2) return NAN;
  if (!isfinite(reference_rad) || !isfinite(angle_rad) || !isfinite(speed_rad_s) ||
      !isfinite(dt_s) || dt_s < 0.0f || dt_s > MOTOR_POSITION_MAX_INTERVAL_MS / 1000.0f) {
    motor_position_reset(index);
    return NAN;
  }
  SpeedPidState & state = speed_pid[index];
  if (!state.initialized) {
    state.previous_speed = speed_rad_s;
    state.initialized = 1;
  }
  float target_speed = limit_value(
    MOTOR_POSITION_GAIN * (reference_rad - angle_rad), MOTOR_POSITION_SPEED_LIMIT_RAD_S);
  float error = target_speed - speed_rad_s;
  float p_a = MOTOR_SPEED_KP * error;
  if (dt_s > 0.0f) {
    // Differentiate measurement to avoid a kick when the target changes.
    float acceleration = (speed_rad_s - state.previous_speed) / dt_s;
    float alpha = dt_s / (MOTOR_SPEED_D_FILTER_S + dt_s);
    state.filtered_acceleration += alpha * (acceleration - state.filtered_acceleration);
  }
  state.previous_speed = speed_rad_s;
  float d_a = -MOTOR_SPEED_KD * state.filtered_acceleration;
  float candidate_i = limit_value(
    state.integral_a + MOTOR_SPEED_KI * error * dt_s, MOTOR_SPEED_I_LIMIT_A);
  float candidate_output = p_a + candidate_i + d_a;
  // Freeze integration only when it would drive saturation further.
  if (!((candidate_output > MOTOR_CURRENT_LIMIT_A && error > 0.0f) ||
        (candidate_output < -MOTOR_CURRENT_LIMIT_A && error < 0.0f))) {
    state.integral_a = candidate_i;
  }
  float current_a = limit_value(p_a + state.integral_a + d_a, MOTOR_CURRENT_LIMIT_A);
  return current_a * MOTOR_CURRENT_RAW_PER_AMP;
}
