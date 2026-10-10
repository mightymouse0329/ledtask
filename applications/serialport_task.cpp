#include <math.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "application_tasks.h"
#include "cmsis_os.h"
#include "imu_yaw.hpp"
#include "motor_report.hpp"
#include "remote_control.h"
#include "task.h"
#include "usart.h"

constexpr int SERIALPORT_PERIOD_MS = 50;
static_assert(SERIALPORT_PERIOD_MS >= 20, "Use a plotting period of at least 20 ms");

extern "C" {
volatile uint32_t serialport_debug[8] = {0x53504C54, 0, 0, 0, 0, 0, 0xFFFFFFFF, 0};
}

void telemetry_task(void const * argument)
{
  (void)argument;
  while (1) {
    ImuYawState imu;
    RemoteState remote;
    taskENTER_CRITICAL();
    remote_get_state(&remote);
    imu_yaw_get_state(&imu);
    taskEXIT_CRITICAL();
    serialport_debug[1]++;
    serialport_debug[2]++;

    struct Channel {
      float value;
      int decimals;
    };
    const Channel channels[] = {
      {(float)remote.online, 0},
      {(float)remote.armed, 0},
      {(float)remote.right_switch, 0},
      {(float)remote.left_switch, 0},
      {(float)remote.mode, 0},
      {(float)remote.channel[0], 0},
      {(float)remote.channel[1], 0},
      {(float)remote.channel[2], 0},
      {(float)remote.channel[3], 0},
      {(float)(imu.state * 10 + imu.reason), 0},
      {imu.acceleration_mps2[0], 3},
      {imu.acceleration_mps2[1], 3},
      {imu.acceleration_mps2[2], 3},
      {imu.angular_velocity_deg_s[0], 3},
      {imu.angular_velocity_deg_s[1], 3},
      {imu.angular_velocity_deg_s[2], 3},
    };
    constexpr int channel_count = 16;
    static_assert(
      sizeof(channels) / sizeof(channels[0]) == channel_count, "Update the CSV channel list");
    char text[256];
    int used = 0;
    int valid_line = 1;
    for (int index = 0; index < channel_count; index++) {
      if (!isfinite(channels[index].value)) {
        valid_line = 0;
        break;
      }
      int length = snprintf(
        text + used, sizeof(text) - used, channels[index].decimals ? "%.3f" : "%.0f",
        (double)channels[index].value);
      if (length < 0 || length >= (int)sizeof(text) - used) {
        valid_line = 0;
        break;
      }
      used += length;
      length = snprintf(text + used, sizeof(text) - used, index == channel_count - 1 ? "\r\n" : ",");
      if (length < 0 || length >= (int)sizeof(text) - used) {
        valid_line = 0;
        break;
      }
      used += length;
    }
    if (valid_line) {
      serialport_debug[3]++;
      HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, (uint8_t *)text, used, 100);
      serialport_debug[6] = status;
      if (status == HAL_OK)
        serialport_debug[4]++;
      else
        serialport_debug[5]++;
    } else {
      serialport_debug[7]++;
    }
    motor_report_print();
    osDelay(SERIALPORT_PERIOD_MS);
  }
}
