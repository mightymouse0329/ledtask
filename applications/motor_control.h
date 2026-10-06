#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
#include "control_constants.hpp"
extern "C" {
#endif

typedef struct
{
  uint16_t encoder_raw;
  int16_t speed_rpm;
  int16_t current_raw;
  uint8_t temperature_deg_c;
  uint8_t online;
  uint8_t received;
  uint32_t last_feedback_ms;
  uint32_t frames;
} MotorFeedback;

typedef struct
{
  MotorFeedback motor[2];
  uint8_t started;
  uint8_t bus_off;
  uint32_t queued_frames;
  uint32_t skipped_frames;
  uint32_t invalid_frames;
  uint32_t error_events;
  uint32_t error_flags;
} MotorState;

void motor_service(void);
void motor_get_state(MotorState * state);

#ifdef __cplusplus
}
#endif

#endif
