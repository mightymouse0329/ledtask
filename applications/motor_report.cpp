#include "motor_report.hpp"

#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

struct ReportSample
{
  MotorState motor;
  LinkState link;
  uint32_t time_ms;
  uint32_t interval_ms;
  float encoder_rpm[2];
  float acceleration_mps2[3];
  float acceleration_norm_mps2;
  uint8_t estimate_valid[2];
  uint8_t imu_valid, imu_reason, imu_state, imu_level, rc_online, rc_right;
};
static ReportSample history[32];
static ReportSample latest;
static unsigned write_index, sample_count;
static int frozen, printed, previous_active;
static uint32_t previous_ms, last_history_ms;
static int32_t previous_counts[2];
static uint8_t previous_valid[2];
static const uint32_t history_period_ms = MOTOR_CONTROL_PERIOD_MS;

void motor_report_record(
  const MotorState * motor, const RemoteState * remote, const ImuYawState * imu, uint32_t now_ms)
{
  taskENTER_CRITICAL();
  latest.motor = *motor;
  motor_link_get_state(&latest.link);
  latest.time_ms = now_ms;
  latest.interval_ms = now_ms - previous_ms;
  latest.imu_valid = imu->valid;
  latest.imu_state = imu->state;
  latest.imu_reason = imu->reason;
  latest.imu_level = imu->level;
  for (int i = 0; i < 3; i++) latest.acceleration_mps2[i] = imu->acceleration_mps2[i];
  latest.acceleration_norm_mps2 = imu->acceleration_norm_mps2;
  latest.rc_online = remote->online;
  latest.rc_right = remote->right_switch;
  for (int i = 0; i < 2; i++) {
    latest.estimate_valid[i] = previous_valid[i] && motor->motor[i].angle_valid &&
                               latest.interval_ms > 0 && latest.interval_ms <= 50;
    latest.encoder_rpm[i] =
      latest.estimate_valid[i]
        ? ((float)motor->motor[i].relative_counts - (float)previous_counts[i]) * 60000.0f /
            (8192.0f * latest.interval_ms)
        : 0.0f;
    previous_counts[i] = motor->motor[i].relative_counts;
    previous_valid[i] = motor->motor[i].angle_valid;
  }
  previous_ms = now_ms;
  // Preserve the first running-to-fault transition until MCU reset.
  int fault_transition = previous_active && latest.link.fault;
  if (!frozen && (fault_transition || now_ms - last_history_ms >= history_period_ms)) {
    history[write_index] = latest;
    write_index = (write_index + 1) % 32;
    if (sample_count < 32) sample_count++;
    last_history_ms = now_ms;
    if (fault_transition) frozen = 1;
  }
  previous_active = latest.link.active;
  taskEXIT_CRITICAL();
}

static void send_line(const char * text)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), 100);
}

static void print_sample(const ReportSample * s, const char * tag)
{
  char line[384];
  snprintf(
    line, sizeof(line),
    "%s,t=%lu,dt=%lu,active=%u,mode=%u,fault=%u,motor=%u,flags=%u,"
    "A_angle=%.3f,A_rpm=%d,A_current=%d,A_out=%d,A_target=%.3f,"
    "B_angle=%.3f,B_rpm=%d,B_current=%d,B_out=%d,B_target=%.3f,"
    "A_target_rpm=%.3f,B_target_rpm=%.3f,"
    "B_enc=%u,B_temp=%u,B_age=%lu,IMU=%u/%u/%u,accel_xyz=%.3f/%.3f/%.3f,"
    "accel_norm=%.3f,level=%u,RC=%u/%u\r\n",
    tag, (unsigned long)s->time_ms, (unsigned long)s->interval_ms, s->link.active, s->link.mode,
    s->link.fault, s->link.fault_motor, s->link.fault_flags,
    s->motor.motor[0].relative_angle_rad / IMU_DEG_TO_RAD, s->motor.motor[0].speed_rpm,
    s->motor.motor[0].current_raw, s->link.output_raw[0], s->link.target_rad[0] / IMU_DEG_TO_RAD,
    s->motor.motor[1].relative_angle_rad / IMU_DEG_TO_RAD, s->motor.motor[1].speed_rpm,
    s->motor.motor[1].current_raw, s->link.output_raw[1], s->link.target_rad[1] / IMU_DEG_TO_RAD,
    s->link.target_speed_rad_s[0] * 60.0f / MOTOR_TWO_PI,
    s->link.target_speed_rad_s[1] * 60.0f / MOTOR_TWO_PI, s->motor.motor[1].encoder_raw,
    s->motor.motor[1].temperature_deg_c,
    (unsigned long)(s->time_ms - s->motor.motor[1].last_feedback_ms), s->imu_state, s->imu_valid,
    s->imu_reason, s->acceleration_mps2[0], s->acceleration_mps2[1], s->acceleration_mps2[2],
    s->acceleration_norm_mps2, s->imu_level, s->rc_online, s->rc_right);
  send_line(line);
}

void motor_report_print(void)
{
  unsigned count;
  unsigned start;
  taskENTER_CRITICAL();
  if (!frozen || printed) {
    taskEXIT_CRITICAL();
    return;
  }
  printed = 1;
  count = sample_count;
  start = (write_index + 32 - count) % 32;
  taskEXIT_CRITICAL();
  send_line("FAULT_HISTORY_BEGIN\r\n");
  for (unsigned i = 0; i < count; i++) print_sample(&history[(start + i) % 32], "HISTORY");
  send_line("FAULT_HISTORY_END\r\n");
}
