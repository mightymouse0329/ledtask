#include <stdio.h>
#include <string.h>

#include "application_tasks.h"
#include "bmi088_simple.h"
#include "cmsis_os.h"
#include "imu_yaw.hpp"
#include "motor_control.h"
#include "motor_link.hpp"
#include "motor_position.hpp"
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
  PositionState position;
  LinkState link;
  char text[192];
  int index;
  int length;
  if (HAL_GetTick() - last_print_ms < 500) return;
  last_print_ms = HAL_GetTick();
  motor_get_state(&state);
  motor_position_get_state(&position);
  motor_link_get_state(&link);
  length = snprintf(
    text, sizeof(text),
    "CAN started=%u bus_off=%u queued=%lu skipped=%lu errors=%lu flags=0x%lX bad=%lu\r\n",
    (unsigned int)state.started, (unsigned int)state.bus_off, (unsigned long)state.queued_frames,
    (unsigned long)state.skipped_frames, (unsigned long)state.error_events,
    (unsigned long)state.error_flags, (unsigned long)state.invalid_frames);
  if (length > 0 && length < (int)sizeof(text)) send_text(text);
  length = snprintf(
    text, sizeof(text),
    "POSITION enabled=%u active=%u ready=%u fault=%u target_rad=%.3f ref_rad=%.3f out_raw=%d\r\n",
    (unsigned int)MOTOR_POSITION_ENABLE, (unsigned int)position.active,
    (unsigned int)position.ready, (unsigned int)position.fault, position.target_rad,
    position.reference_rad, (int)position.output_raw);
  if (length > 0 && length < (int)sizeof(text)) send_text(text);
  length = snprintf(
    text, sizeof(text), "LINK enabled=%u active=%u ready=%u fault=%u ratio=%.1f yaw_rad=%.3f\r\n",
    (unsigned int)MOTOR_LINK_ENABLE, (unsigned int)link.active, (unsigned int)link.ready,
    (unsigned int)link.fault, link.ratio, link.yaw_change_rad);
  if (length > 0 && length < (int)sizeof(text)) send_text(text);
  length = snprintf(
    text, sizeof(text), "LINK A target=%.3f ref=%.3f out=%d | B target=%.3f ref=%.3f out=%d\r\n",
    link.target_rad[0], link.reference_rad[0], (int)link.output_raw[0], link.target_rad[1],
    link.reference_rad[1], (int)link.output_raw[1]);
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

static const char * imu_state_name(int state)
{
  if (state == IMU_WARMUP) return "WARMUP";
  if (state == IMU_CALIBRATING) return "CALIBRATING";
  if (state == IMU_READY) return "READY";
  if (state == IMU_FAULT) return "FAULT";
  return "WAIT";
}

static const char * imu_reason_name(int reason)
{
  if (reason == IMU_REASON_SPI) return "SPI";
  if (reason == IMU_REASON_GAP) return "GAP";
  if (reason == IMU_REASON_LEVEL) return "LEVEL_OR_ACCEL";
  if (reason == IMU_REASON_RANGE) return "RANGE";
  return "NONE";
}

static void print_imu(void)
{
  static uint32_t last_print_ms;
  ImuYawState state;
  char text[256];
  if (HAL_GetTick() - last_print_ms < 500) return;
  last_print_ms = HAL_GetTick();
  imu_yaw_get_state(&state);
  const uint32_t age_ms = HAL_GetTick() - state.last_sample_ms;
  snprintf(
    text, sizeof(text),
    "IMU %s valid=%u reason=%s level=%u io=%u cal=%lu/%d restarts=%lu init_error=%u\r\n",
    imu_state_name(state.state), (unsigned int)state.valid, imu_reason_name(state.reason),
    (unsigned int)state.level, (unsigned int)state.io_ok, (unsigned long)state.calibration_samples,
    IMU_CALIBRATION_SAMPLES, (unsigned long)state.calibration_restarts,
    (unsigned int)state.init_error);
  send_text(text);
  snprintf(
    text, sizeof(text),
    "YAW t_ms=%lu deg=%.4f rate_deg_s=%.4f bias_z_deg_s=%.4f dt_ms=%lu max_dt_ms=%lu age_ms=%lu "
    "samples=%lu io_errors=%lu gaps=%lu\r\n",
    (unsigned long)state.last_sample_ms, state.yaw_rad / IMU_DEG_TO_RAD, state.corrected_z_deg_s,
    state.bias_deg_s[2], (unsigned long)state.interval_ms, (unsigned long)state.max_interval_ms,
    (unsigned long)age_ms, (unsigned long)state.samples, (unsigned long)state.io_errors,
    (unsigned long)state.gap_events);
  send_text(text);
  snprintf(
    text, sizeof(text), "ACC X=%.3f Y=%.3f Z=%.3f norm=%.3f | GYRO X=%.3f Y=%.3f Z=%.3f\r\n",
    state.acceleration_mps2[0], state.acceleration_mps2[1], state.acceleration_mps2[2],
    state.acceleration_norm_mps2, state.angular_velocity_deg_s[0], state.angular_velocity_deg_s[1],
    state.angular_velocity_deg_s[2]);
  send_text(text);
}

void telemetry_task(void const * argument)
{
  (void)argument;
  send_text("IMU planar yaw: keep board face up, level and still during calibration.\r\n");
  while (1) {
    print_remote();
    print_motors();
    print_imu();
    osDelay(20);
  }
}

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
