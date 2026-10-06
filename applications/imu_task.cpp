#include <stdio.h>
#include <string.h>

#include "application_tasks.h"
#include "bmi088_simple.h"
#include "cmsis_os.h"
#include "motor_control.h"
#include "remote_control.h"
#include "usart.h"

static int send_text(const char text[])
{
  return HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100) == HAL_OK;
}

static const char * switch_name(int value)
{
  if (value == RC_SWITCH_UP) return "UP";
  if (value == RC_SWITCH_MID) return "MID";
  return "DOWN";
}

static void print_remote(void)
{
  static uint32_t last_print_ms;
  RemoteState state;
  char text[192];
  const char * mode = "DISABLED";
  const char * ratio = "0.5";
  int length;

  if (HAL_GetTick() - last_print_ms < 200) return;
  last_print_ms = HAL_GetTick();
  remote_get_state(&state);
  if (state.mode == RC_MODE_LINK) mode = "LINK_REQUEST";
  if (state.mode == RC_MODE_RESET) mode = "RESET_REQUEST";
  if (state.left_switch == RC_SWITCH_MID) ratio = "-1";
  if (state.left_switch == RC_SWITCH_UP) ratio = "3";

  length = snprintf(
    text, sizeof(text),
    "RC %s ready=%u R=%s L=%s mode=%s B_ratio=%s CH=%d,%d,%d,%d frames=%lu errors=%lu\r\n",
    state.online ? "ONLINE" : "OFFLINE", (unsigned int)state.armed, switch_name(state.right_switch),
    switch_name(state.left_switch), mode, ratio, state.channel[0], state.channel[1],
    state.channel[2], state.channel[3], (unsigned long)state.frames, (unsigned long)state.errors);
  if (length > 0 && length < (int)sizeof(text)) send_text(text);
}
static void print_motors(void)
{
  static uint32_t last_print_ms;
  MotorState state;
  char text[192];
  int index;
  int length;
  if (HAL_GetTick() - last_print_ms < 500) return;
  last_print_ms = HAL_GetTick();
  motor_get_state(&state);
  length = snprintf(
    text, sizeof(text),
    "CAN started=%u bus_off=%u ZERO_ONLY queued=%lu skipped=%lu errors=%lu flags=0x%lX bad=%lu\r\n",
    (unsigned int)state.started, (unsigned int)state.bus_off, (unsigned long)state.queued_frames,
    (unsigned long)state.skipped_frames, (unsigned long)state.error_events,
    (unsigned long)state.error_flags, (unsigned long)state.invalid_frames);
  if (length > 0 && length < (int)sizeof(text)) send_text(text);
  for (index = 0; index < 2; index++) {
    length = snprintf(
      text, sizeof(text),
      "MOTOR %c %s encoder=%u speed_rpm=%d current_raw=%d temp_C=%u frames=%lu\r\n", 'A' + index,
      state.motor[index].online ? "ONLINE" : "OFFLINE",
      (unsigned int)state.motor[index].encoder_raw, (int)state.motor[index].speed_rpm,
      (int)state.motor[index].current_raw, (unsigned int)state.motor[index].temperature_deg_c,
      (unsigned long)state.motor[index].frames);
    if (length > 0 && length < (int)sizeof(text)) send_text(text);
    length = snprintf(
      text, sizeof(text), "ANGLE %c valid=%u single_rad=%.4f relative_rad=%.4f counts=%ld\r\n",
      'A' + index, (unsigned int)state.motor[index].angle_valid,
      state.motor[index].single_angle_rad, state.motor[index].relative_angle_rad,
      (long)state.motor[index].relative_counts);
    if (length > 0 && length < (int)sizeof(text)) send_text(text);
  }
}

void imu_task(void const * argument)
{
  float acceleration_mps2[3];
  float angular_velocity_deg_s[3];
  char text[192];
  int error;
  int ready = 0;
  int length;
  (void)argument;

  while (1) {
    print_remote();
    print_motors();
    if (ready == 0) {
      error = bmi088_init();
      if (error != 0) {
        snprintf(
          text, sizeof(text),
          "BMI088 init error=%d (1:SPI 2:ACC_ID 3:GYRO_ID 4:ACC_CFG 5:GYRO_CFG)\r\n", error);
        send_text(text);
        osDelay(1000);
        continue;
      }
      ready = 1;
      send_text("BMI088 OK. ACC: m/s^2; GYRO: deg/s; axes: X,Y,Z\r\n");
    }

    if (bmi088_read(acceleration_mps2, angular_velocity_deg_s)) {
      length = snprintf(
        text, sizeof(text), "ACC X=%.3f Y=%.3f Z=%.3f | GYRO X=%.3f Y=%.3f Z=%.3f\r\n",
        acceleration_mps2[0], acceleration_mps2[1], acceleration_mps2[2], angular_velocity_deg_s[0],
        angular_velocity_deg_s[1], angular_velocity_deg_s[2]);
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
