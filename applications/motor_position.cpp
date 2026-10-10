#include "motor_position.hpp"

#include <math.h>

#include "control_constants.hpp"
#include "motor_control.h"
#include "tools/pid/pid.hpp"

constexpr float MOTOR_SPEED_PID_DT_S = MOTOR_CONTROL_PERIOD_MS / 1000.0f;
constexpr float MOTOR_SPEED_PID_D_ALPHA =
  MOTOR_SPEED_PID_DT_S / (MOTOR_SPEED_D_FILTER_S + MOTOR_SPEED_PID_DT_S);

static sp::PID position_pid[2] = {
  sp::PID(
    MOTOR_SPEED_PID_DT_S, MOTOR_POSITION_KP, MOTOR_POSITION_KI, MOTOR_POSITION_KD,
    MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S, MOTOR_POSITION_I_LIMIT_RAD_S, 1.0f, false, true),
  sp::PID(
    MOTOR_SPEED_PID_DT_S, MOTOR_POSITION_KP, MOTOR_POSITION_KI, MOTOR_POSITION_KD,
    MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S, MOTOR_POSITION_I_LIMIT_RAD_S, 1.0f, false, true),
};
static sp::PID speed_pid[2] = {
  sp::PID(
    MOTOR_SPEED_PID_DT_S, MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD,
    MOTOR_CURRENT_LIMIT_A, MOTOR_SPEED_I_LIMIT_A, MOTOR_SPEED_PID_D_ALPHA),
  sp::PID(
    MOTOR_SPEED_PID_DT_S, MOTOR_SPEED_KP, MOTOR_SPEED_KI, MOTOR_SPEED_KD,
    MOTOR_CURRENT_LIMIT_A, MOTOR_SPEED_I_LIMIT_A, MOTOR_SPEED_PID_D_ALPHA),
};
static float target_speed[2];

static float limit_value(float value, float maximum)
{
  if (value > maximum) return maximum;
  if (value < -maximum) return -maximum;
  return value;
}

void motor_position_reset(int index)
{
  if (index < 0 || index >= 2) return;
  position_pid[index].clear();
  speed_pid[index].clear();
  target_speed[index] = 0.0f;
}

float motor_position_target_speed(int index)
{
  if (index < 0 || index >= 2) return NAN;
  return target_speed[index];
}

static void ramp_target_speed(int index, float requested_speed, float dt_s, float max_speed_rad_s)
{
  requested_speed = limit_value(requested_speed, max_speed_rad_s);
  if (dt_s > 0.0f) {
    float max_speed_change = MOTOR_POSITION_TARGET_ACCEL_RAD_S2 * dt_s;
    target_speed[index] += limit_value(requested_speed - target_speed[index], max_speed_change);
  }
}

static float motor_speed_output(int index, float speed_rad_s)
{
  speed_pid[index].calc(target_speed[index], speed_rad_s);
  float current_a = limit_value(speed_pid[index].out, MOTOR_CURRENT_LIMIT_A);
  return current_a * MOTOR_CURRENT_RAW_PER_AMP;
}

float motor_position_spin_output(int index, float target_speed_rad_s, float speed_rad_s)
{
  if (index < 0 || index >= 2) return NAN;
  if (!isfinite(target_speed_rad_s) || !isfinite(speed_rad_s)) {
    motor_position_reset(index);
    return NAN;
  }
  target_speed[index] = target_speed_rad_s;
  return motor_speed_output(index, speed_rad_s);
}

float motor_position_output(
  int index, float reference_rad, float reference_rate_rad_s, float max_target_speed_rad_s,
  float angle_rad, float speed_rad_s, float dt_s)
{
  if (index < 0 || index >= 2) return NAN;
  if (!isfinite(reference_rad) || !isfinite(reference_rate_rad_s) ||
      !isfinite(max_target_speed_rad_s) || !isfinite(angle_rad) || !isfinite(speed_rad_s) ||
      !isfinite(dt_s) || dt_s < 0.0f || dt_s > MOTOR_POSITION_MAX_INTERVAL_MS / 1000.0f) {
    motor_position_reset(index);
    return NAN;
  }
  position_pid[index].calc(reference_rad, angle_rad);

  float requested_speed =
    position_pid[index].out + MOTOR_POSITION_FEEDFORWARD * reference_rate_rad_s;
  ramp_target_speed(index, requested_speed, dt_s, max_target_speed_rad_s);
  return motor_speed_output(index, speed_rad_s);
}
