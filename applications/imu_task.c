#include "cmsis_os.h"
#include "usart.h"
#include "bmi088_simple.h"
#include <stdio.h>
#include <string.h>


static int send_text(char text[])
{
  return HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100) == HAL_OK;
}

void imu_task(void const *argument)
{
  float acc[3];
  float gyro[3];
  char text[192];
  int error;
  int ready = 0;
  int length;
  (void)argument;

  while (1) {
    if (ready == 0) {
      error = bmi088_init();
      if (error != 0) {
        snprintf(text, sizeof(text), "BMI088 init error=%d (1:SPI 2:ACC_ID 3:GYRO_ID 4:ACC_CFG 5:GYRO_CFG)\r\n", error);
        send_text(text);
        osDelay(1000);
        continue;
      }
      ready = 1;
      send_text("BMI088 OK. ACC: m/s^2; GYRO: deg/s; axes: X,Y,Z\r\n");
    }

    if (bmi088_read(acc, gyro)) {

      length = snprintf(text, sizeof(text),
        "ACC X=%.3f Y=%.3f Z=%.3f | GYRO X=%.3f Y=%.3f Z=%.3f\r\n",
        acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2]);
      if (length > 0 && length < (int)sizeof(text)) {
        send_text(text);
      }
    } else {
      send_text("BMI088 SPI read error; retrying init\r\n");
      ready = 0;
    }
    osDelay(100);
  }
}
