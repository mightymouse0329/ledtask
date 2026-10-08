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

typedef struct
{
  uint32_t events;
  uint32_t bytes;
  uint32_t last_size;
  uint32_t length_errors;
  uint32_t decode_errors;
  uint32_t uart_errors;
  uint32_t start_errors;
  uint32_t last_uart_error;
  uint32_t start_status;
  uint32_t irq_count;
  uint32_t buffered_bytes;
} RemoteDiagnostic;

void remote_note_uart_irq(void);
void remote_get_diagnostic(RemoteDiagnostic * result);
void remote_start(void);
void remote_service(void);
void remote_get_state(RemoteState * state);

#ifdef __cplusplus
}
#endif

#endif
