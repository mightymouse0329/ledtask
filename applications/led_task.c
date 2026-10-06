#include "application_tasks.h"
#include "cmsis_os.h"
#include "tim.h"

static volatile uint32_t heartbeat = 0;

void app_heartbeat(void) { heartbeat++; }

static void led_set(int color, int brightness_percent)
{
  int pulse;

  pulse = 65536 * brightness_percent / 100;

  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);

  if (color == 0) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, pulse);
  } else if (color == 1) {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse);
  } else {
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
  }
}

static void check_heartbeat(void)
{
  static uint32_t previous = 0;
  static uint32_t last_check_ms = 0;

  if (HAL_GetTick() - last_check_ms < 500) {
    return;
  }

  while (heartbeat == previous) {
    led_set(0, 15);
    osDelay(20);
  }

  previous = heartbeat;
  last_check_ms = HAL_GetTick();
}

void led_task(void const * argument)
{
  int color;
  int brightness_percent;
  (void)argument;

  __HAL_TIM_SET_AUTORELOAD(&htim5, 65535);
  led_set(0, 0);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  while (1) {
    for (color = 0; color < 3; color++) {
      for (brightness_percent = 0; brightness_percent <= 50; brightness_percent++) {
        check_heartbeat();
        led_set(color, brightness_percent);
        osDelay(30);
      }

      for (brightness_percent = 49; brightness_percent >= 0; brightness_percent--) {
        check_heartbeat();
        led_set(color, brightness_percent);
        osDelay(30);
      }
    }
  }
}
