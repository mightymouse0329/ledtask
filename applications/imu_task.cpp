#include "application_tasks.h"
#include "bmi088_simple.h"
#include "cmsis_os.h"
#include "imu_yaw.hpp"
#include "usart.h"

void imu_task(void const * argument)
{
  float acceleration_mps2[3];
  float angular_velocity_deg_s[3];
  int ready = 0;
  int error;
  (void)argument;
  while (1) {
    if (!ready) {
      error = bmi088_init();
      if (error) {
        imu_yaw_io_error(error);
        osDelay(1000);
        continue;
      }
      imu_yaw_begin(HAL_GetTick());
      ready = 1;
    }
    if (bmi088_read(acceleration_mps2, angular_velocity_deg_s)) {
      imu_yaw_update(acceleration_mps2, angular_velocity_deg_s, HAL_GetTick());
    } else {
      imu_yaw_io_error(0);
      ready = 0;
    }
    osDelay(IMU_SAMPLE_PERIOD_MS);
  }
}
