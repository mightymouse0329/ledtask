#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

extern "C" void buzzer_task(void const * argument)
{
  (void)argument;

  sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84000000.0f);
  constexpr float notes[] = {1000.0f, 1500.0f, 2000.0f};

  buzzer.stop();
  for (const float frequency : notes) {
    buzzer.set(frequency, 0.10f);

    if (HAL_TIM_GenerateEvent(&htim4, TIM_EVENTSOURCE_UPDATE) != HAL_OK) {
      Error_Handler();
    }
    __HAL_TIM_CLEAR_FLAG(&htim4, TIM_FLAG_UPDATE);

    buzzer.start();
    osDelay(120);
    buzzer.stop();
    osDelay(60);
  }

  for (;;) {
    osDelay(1000);
  }
}
