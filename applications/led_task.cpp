#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "io/led/led.hpp"
#include "task.h"

namespace
{
uint32_t work_heartbeat = 0;

uint32_t read_heartbeat()
{
  taskENTER_CRITICAL();
  const uint32_t value = work_heartbeat;
  taskEXIT_CRITICAL();
  return value;
}
}  // namespace

extern "C" void app_heartbeat(void)
{
  taskENTER_CRITICAL();
  ++work_heartbeat;
  taskEXIT_CRITICAL();
}

extern "C" void led_task(void const * argument)
{
  (void)argument;
  sp::LED led(&htim5);
  led.set(0.0f, 0.0f, 0.0f);
  led.start();

  uint32_t previous = read_heartbeat();
  uint8_t color = 0;
  constexpr float brightness = 0.15f;

  for (;;) {
    osDelay(500);
    const uint32_t current = read_heartbeat();

    if (current == previous) {
      led.set(brightness, 0.0f, 0.0f);
      continue;
    }
    previous = current;

    switch (color) {
      case 0:
        led.set(brightness, 0.0f, 0.0f);
        break;
      case 1:
        led.set(0.0f, brightness, 0.0f);
        break;
      default:
        led.set(0.0f, 0.0f, brightness);
        break;
    }
    color = static_cast<uint8_t>((color + 1U) % 3U);
  }
}
