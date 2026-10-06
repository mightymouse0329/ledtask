#include "motor_position.hpp"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

static PositionState position;
static float entry_angle_rad;
static uint32_t last_update_ms;
static uint32_t previous_errors;
static uint8_t initialized;

static float limit_value(float value, float maximum)
{
  if (value > maximum) return maximum;
  if (value < -maximum) return -maximum;
  return value;
}

void motor_position_stop(void)
{
  position.active = 0;
  position.ready = 0;
  position.fault = 1;
  position.output_raw = 0;
  position.speed_target_rad_s = 0.0f;
}

int16_t motor_position_update(const MotorState * motor, const RemoteState * remote, uint32_t now_ms)
{
  uint32_t elapsed_ms = now_ms - last_update_ms;
  int new_error = motor->error_events != previous_errors;
  float angle_rad = motor->motor[0].relative_angle_rad;
  float speed_rad_s = motor->motor[0].speed_rpm * (MOTOR_TWO_PI / 60.0f);
  float reference_step;
  float output;
  last_update_ms = now_ms;
  previous_errors = motor->error_events;
  if (!initialized) {
    elapsed_ms = 0;
    initialized = 1;
  }

  position.output_raw = 0;
  position.speed_target_rad_s = 0.0f;
  if (!MOTOR_POSITION_ENABLE) {
    position.active = 0;
    position.ready = 0;
    position.fault = 0;
    return 0;
  }

  if (
    !motor->started || motor->bus_off || new_error || !remote->online || !remote->armed ||
    !motor->motor[0].online || !motor->motor[1].online || !motor->motor[0].angle_valid ||
    !motor->motor[1].angle_valid ||
    motor->motor[0].temperature_deg_c >= MOTOR_POSITION_MAX_TEMP_DEG_C ||
    motor->motor[1].temperature_deg_c >= MOTOR_POSITION_MAX_TEMP_DEG_C || !isfinite(angle_rad) ||
    !isfinite(speed_rad_s) || fabsf(speed_rad_s) > MOTOR_POSITION_TRIP_SPEED_RAD_S ||
    (position.active && elapsed_ms > MOTOR_POSITION_MAX_INTERVAL_MS)) {
    motor_position_stop();
    return 0;
  }

  if (remote->right_switch == RC_SWITCH_DOWN) {
    position.active = 0;
    position.ready = 1;
    position.fault = 0;
    position.reference_rad = angle_rad;
    position.target_rad = angle_rad;
    return 0;
  }
  if (remote->right_switch != RC_SWITCH_MID || remote->mode != RC_MODE_LINK) {
    position.active = 0;
    position.ready = 0;
    return 0;
  }
  if (position.fault || !position.ready) return 0;
  if (!position.active) {
    entry_angle_rad = angle_rad;
    position.reference_rad = angle_rad;
    position.target_rad = angle_rad + MOTOR_POSITION_TEST_OFFSET_RAD;
    position.active = 1;
    return 0;
  }

  if (fabsf(angle_rad - entry_angle_rad) > MOTOR_POSITION_TRAVEL_LIMIT_RAD) {
    motor_position_stop();
    return 0;
  }
  reference_step = MOTOR_POSITION_RAMP_RAD_S * elapsed_ms / 1000.0f;
  position.reference_rad +=
    limit_value(position.target_rad - position.reference_rad, reference_step);
  position.speed_target_rad_s = limit_value(
    MOTOR_POSITION_GAIN * (position.reference_rad - angle_rad), MOTOR_POSITION_SPEED_LIMIT_RAD_S);
  output = MOTOR_SPEED_GAIN * (position.speed_target_rad_s - speed_rad_s);
  if (!isfinite(output)) {
    motor_position_stop();
    return 0;
  }
  position.output_raw = (int16_t)limit_value(output, MOTOR_POSITION_OUTPUT_LIMIT_RAW);
  return position.output_raw;
}

void motor_position_get_state(PositionState * result)
{
  taskENTER_CRITICAL();
  *result = position;
  taskEXIT_CRITICAL();
}
