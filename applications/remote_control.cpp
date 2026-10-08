#include "remote_control.h"

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

static uint8_t receive_buffer[36];
static volatile RemoteState remote;
static volatile RemoteDiagnostic diagnostic;
static volatile uint32_t last_frame_ms;
static volatile uint8_t consecutive_frames;
static volatile uint8_t restart_needed;

static void invalidate_remote(void)
{
  int index;
  remote.online = 0;
  remote.armed = 0;
  remote.mode = RC_MODE_DISABLED;
  remote.right_switch = RC_SWITCH_DOWN;
  remote.left_switch = RC_SWITCH_DOWN;
  for (index = 0; index < 4; index++) {
    remote.channel[index] = 0;
  }
  consecutive_frames = 0;
}

static void receive_again(void)
{
  restart_needed = 1;
  HAL_StatusTypeDef status =
    HAL_UARTEx_ReceiveToIdle_IT(&huart3, receive_buffer, sizeof(receive_buffer));
  diagnostic.start_status = status;
  if (status == HAL_OK) {
    restart_needed = 0;
  } else {
    diagnostic.start_errors++;
  }
}

void remote_start(void)
{
  invalidate_remote();
  receive_again();
}

static int decode_frame(const uint8_t data[])
{
  int channel[4];
  int right;
  int left;
  int index;
  uint32_t now_ms = HAL_GetTick();

  channel[0] = (data[0] | (data[1] << 8)) & 0x07FF;
  channel[1] = ((data[1] >> 3) | (data[2] << 5)) & 0x07FF;
  channel[2] = ((data[2] >> 6) | (data[3] << 2) | (data[4] << 10)) & 0x07FF;
  channel[3] = ((data[4] >> 1) | (data[5] << 7)) & 0x07FF;
  right = (data[5] >> 4) & 0x03;
  left = (data[5] >> 6) & 0x03;

  for (index = 0; index < 4; index++) {
    if (channel[index] < 364 || channel[index] > 1684) return 0;
  }
  if (right < 1 || right > 3 || left < 1 || left > 3) return 0;

  if (now_ms - last_frame_ms >= 100) {
    invalidate_remote();
  }
  for (index = 0; index < 4; index++) {
    remote.channel[index] = channel[index] - 1024;
  }
  remote.right_switch = right;
  remote.left_switch = left;
  remote.frames++;
  last_frame_ms = now_ms;
  if (consecutive_frames < 3) consecutive_frames++;
  remote.online = (consecutive_frames >= 3);

  if (remote.online && right == RC_SWITCH_DOWN) {
    remote.armed = 1;
  }
  remote.mode = RC_MODE_DISABLED;
  if (remote.online && remote.armed) {
    if (right == RC_SWITCH_MID) remote.mode = RC_MODE_LINK;
    if (right == RC_SWITCH_UP) remote.mode = RC_MODE_RESET;
  }
  return 1;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t size)
{
  if (huart->Instance != USART3) return;
  diagnostic.events++;
  diagnostic.bytes += size;
  diagnostic.last_size = size;
  int valid = 0;
  if (size != 18)
    diagnostic.length_errors++;
  else {
    valid = decode_frame(receive_buffer);
    if (!valid) diagnostic.decode_errors++;
  }
  if (!valid) {
    remote.errors++;
    invalidate_remote();
  }
  receive_again();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart->Instance != USART3) return;
  diagnostic.uart_errors++;
  diagnostic.last_uart_error = HAL_UART_GetError(huart);
  remote.errors++;
  invalidate_remote();
  HAL_UART_AbortReceive(&huart3);
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  restart_needed = 1;
}

void remote_service(void)
{
  taskENTER_CRITICAL();
  if (HAL_GetTick() - last_frame_ms >= 100) {
    invalidate_remote();
  }
  if (restart_needed) {
    HAL_UART_AbortReceive(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    receive_again();
  }
  taskEXIT_CRITICAL();
}

void remote_get_state(RemoteState * state)
{
  taskENTER_CRITICAL();
  if (HAL_GetTick() - last_frame_ms >= 100) {
    invalidate_remote();
  }
  for (int index = 0; index < 4; index++) {
    state->channel[index] = remote.channel[index];
  }
  state->right_switch = remote.right_switch;
  state->left_switch = remote.left_switch;
  state->online = remote.online;
  state->armed = remote.armed;
  state->mode = remote.mode;
  state->frames = remote.frames;
  state->errors = remote.errors;
  taskEXIT_CRITICAL();
}

void remote_note_uart_irq(void)
{
  diagnostic.irq_count++;
}

void remote_get_diagnostic(RemoteDiagnostic * result)
{
  taskENTER_CRITICAL();
  result->events = diagnostic.events;
  result->bytes = diagnostic.bytes;
  result->last_size = diagnostic.last_size;
  result->length_errors = diagnostic.length_errors;
  result->decode_errors = diagnostic.decode_errors;
  result->uart_errors = diagnostic.uart_errors;
  result->start_errors = diagnostic.start_errors;
  result->last_uart_error = diagnostic.last_uart_error;
  result->start_status = diagnostic.start_status;
  result->irq_count = diagnostic.irq_count;
  /* Snapshot of the current transfer, not a cumulative byte counter. */
  result->buffered_bytes = 0;
  if (huart3.RxState == HAL_UART_STATE_BUSY_RX &&
      huart3.RxXferCount <= huart3.RxXferSize) {
    result->buffered_bytes = huart3.RxXferSize - huart3.RxXferCount;
  }
  taskEXIT_CRITICAL();
}
