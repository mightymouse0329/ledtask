#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>

#define RC_SWITCH_UP 1
#define RC_SWITCH_DOWN 2
#define RC_SWITCH_MID 3
#define RC_MODE_DISABLED 0
#define RC_MODE_LINK 1
#define RC_MODE_RESET 2

typedef struct {
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
void remote_get_state(RemoteState *state);

#endif
