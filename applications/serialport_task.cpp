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

    // Acceptance output: remote control state, then the BMI088 six-axis data.
    // ACC is m/s^2 (includes gravity when at rest), GYRO is deg/s; both are the raw sensor
    // axes, so tilt/turn the board to check the mounting direction of each axis.
    struct Channel {
      float value;
      int decimals;
    };
    const Channel channels[] = {
      {(float)remote.online, 0},           // 1  receiver link valid
      {(float)remote.armed, 0},            // 2  right switch was seen DOWN, other modes allowed
      {(float)remote.right_switch, 0},     // 3  1 = up, 2 = down, 3 = middle
      {(float)remote.left_switch, 0},      // 4
      {(float)remote.mode, 0},             // 5  0 disabled, 1 link request, 2 reset request
      {(float)remote.channel[0], 0},       // 6  right horizontal, about -660..660
      {(float)remote.channel[1], 0},       // 7  right vertical
      {(float)remote.channel[2], 0},       // 8  left horizontal
      {(float)remote.channel[3], 0},       // 9  left vertical
      {(float)(imu.state * 10 + imu.reason), 0},  // 10 30 = READY; 41/42/43/44 = faults
      {imu.acceleration_mps2[0], 3},       // 11 ACC X
      {imu.acceleration_mps2[1], 3},       // 12 ACC Y
      {imu.acceleration_mps2[2], 3},       // 13 ACC Z
      {imu.angular_velocity_deg_s[0], 3},  // 14 GYRO X
      {imu.angular_velocity_deg_s[1], 3},  // 15 GYRO Y
      {imu.angular_velocity_deg_s[2], 3},  // 16 GYRO Z
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
