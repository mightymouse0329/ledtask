#include "application_tasks.h"
#include "cmsis_os.h"
#include "tim.h"

void buzzer_task(void const * argument)
{
  int notes_hz[3] = {1000, 1500, 2000};
  int index;
  int period_ticks;
  (void)argument;

  __HAL_TIM_SET_PRESCALER(&htim4, 83);

  for (index = 0; index < 3; index++) {
    period_ticks = 1000000 / notes_hz[index];
    __HAL_TIM_SET_AUTORELOAD(&htim4, period_ticks - 1);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period_ticks / 10);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    osDelay(120);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    osDelay(60);
  }

  while (1) {
    osDelay(1000);
  }
}
