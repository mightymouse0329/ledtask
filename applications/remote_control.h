#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

#ifdef __cplusplus
#include "control_constants.hpp"
extern "C" {
#endif

typedef struct
{
  int16_t channel[4];
  uint8_t right_switch;
  uint8_t left_switch;
  uint8_t online;
  uint8_t armed;
  uint8_t mode;
  uint32_t frames;
  uint32_t errors;
} RemoteState;

void remote_start(void);
void remote_service(void);
void remote_get_state(RemoteState * state);

#ifdef __cplusplus
}
#endif

#endif
