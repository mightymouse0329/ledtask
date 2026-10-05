#include "remote_control.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"

static uint8_t rx_buffer[36];
static volatile RemoteState remote;
static volatile uint32_t last_frame_tick;
static volatile uint8_t consecutive_frames;
static volatile uint8_t restart_needed;

static void invalidate_remote(void)
{
  int i;
  remote.online = 0;
  remote.armed = 0;
  remote.mode = RC_MODE_DISABLED;
  remote.right_switch = RC_SWITCH_DOWN;
  remote.left_switch = RC_SWITCH_DOWN;
  for (i = 0; i < 4; i++) {
    remote.channel[i] = 0;
  }
  consecutive_frames = 0;
}

static void receive_again(void)
{
  restart_needed = 1;
  if (HAL_UARTEx_ReceiveToIdle_IT(&huart3, rx_buffer, sizeof(rx_buffer)) == HAL_OK) {
    restart_needed = 0;
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
  int i;
  uint32_t now = HAL_GetTick();

  channel[0] = (data[0] | (data[1] << 8)) & 0x07FF;
  channel[1] = ((data[1] >> 3) | (data[2] << 5)) & 0x07FF;
  channel[2] = ((data[2] >> 6) | (data[3] << 2) | (data[4] << 10)) & 0x07FF;
  channel[3] = ((data[4] >> 1) | (data[5] << 7)) & 0x07FF;
  right = (data[5] >> 4) & 0x03;
  left = (data[5] >> 6) & 0x03;

  for (i = 0; i < 4; i++) {
    if (channel[i] < 364 || channel[i] > 1684) return 0;
  }
  if (right < 1 || right > 3 || left < 1 || left > 3) return 0;

  if (now - last_frame_tick >= 100) {
    invalidate_remote();
  }
  for (i = 0; i < 4; i++) {
    remote.channel[i] = channel[i] - 1024;
  }
  remote.right_switch = right;
  remote.left_switch = left;
  remote.frames++;
  last_frame_tick = now;
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance != USART3) return;
  if (size != 18 || !decode_frame(rx_buffer)) {
    remote.errors++;
    invalidate_remote();
  }
  receive_again();
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART3) return;
  remote.errors++;
  invalidate_remote();
  HAL_UART_AbortReceive(&huart3);
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  restart_needed = 1;
}

void remote_service(void)
{
  taskENTER_CRITICAL();
  if (HAL_GetTick() - last_frame_tick >= 100) {
    invalidate_remote();
  }
  if (restart_needed) {
    HAL_UART_AbortReceive(&huart3);
    __HAL_UART_CLEAR_OREFLAG(&huart3);
    receive_again();
  }
  taskEXIT_CRITICAL();
}

void remote_get_state(RemoteState *state)
{
  taskENTER_CRITICAL();
  if (HAL_GetTick() - last_frame_tick >= 100) {
    invalidate_remote();
  }
  *state = remote;
  taskEXIT_CRITICAL();
}
