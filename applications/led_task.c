#include "cmsis_os.h"
#include "tim.h"

static volatile int heartbeat = 0;

void app_heartbeat(void)
{
  heartbeat++;
}

static void led_set(int color, int brightness)
{
  int pulse;

  pulse = 65536 * brightness / 100;

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
  static int previous = 0;
  static int last_check = 0;

  if (HAL_GetTick() - last_check < 500) {
    return;
  }

  while (heartbeat == previous) {
    led_set(0, 15);
    osDelay(20);
  }

  previous = heartbeat;
  last_check = HAL_GetTick();
}

void led_task(void const *argument)
{
  int color;
  int brightness;
  (void)argument;

  __HAL_TIM_SET_AUTORELOAD(&htim5, 65535);
  led_set(0, 0);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);

  while (1) {
    for (color = 0; color < 3; color++) {
      for (brightness = 0; brightness <= 50; brightness++) {
        check_heartbeat();
        led_set(color, brightness);
        osDelay(30);
      }

      for (brightness = 49; brightness >= 0; brightness--) {
        check_heartbeat();
        led_set(color, brightness);
        osDelay(30);
      }
    }
  }
}
