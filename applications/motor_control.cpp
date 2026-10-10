#include "motor_control.h"

#include "FreeRTOS.h"
#include "can.h"
#include "motor_link.hpp"
#include "motor_report.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "task.h"

static volatile MotorState motor_state;
static sp::RM_Motor middleware_motors[2] = {
  sp::RM_Motor(MOTOR_A_ID, sp::RM_Motors::GM6020),
  sp::RM_Motor(MOTOR_B_ID, sp::RM_Motors::GM6020),
};

void motor_control_task(void const * argument)
{
  (void)argument;
  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(MOTOR_CONTROL_PERIOD_MS);
  configASSERT(period > 0);
  for (;;) {
    motor_service();
    vTaskDelayUntil(&last_wake_time, period);
  }
}

static int start_can(void)
{
  CAN_FilterTypeDef filter = {};
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (0x204 + MOTOR_A_ID) << 5;
  filter.FilterIdLow = 0;
  filter.FilterMaskIdHigh = (0x204 + MOTOR_B_ID) << 5;
  filter.FilterMaskIdLow = 0;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) return 0;
  if (HAL_CAN_Start(&hcan1) != HAL_OK) return 0;
  if (
    HAL_CAN_ActivateNotification(
      &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN | CAN_IT_ERROR | CAN_IT_BUSOFF |
                CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
    HAL_CAN_Stop(&hcan1);
    return 0;
  }
  return 1;
}

void motor_service(void)
{
  static uint32_t last_attempt_ms;
  static uint8_t attempted;
  MotorState snapshot;
  RemoteState remote;
  ImuYawState imu;
  MotorOutput output = {};
  CAN_TxHeaderTypeDef header = {};
  uint8_t data[8] = {0};
  uint32_t mailbox;
  uint32_t now_ms = HAL_GetTick();

  if (!motor_state.started) {
    if (attempted && now_ms - last_attempt_ms < 1000) return;
    attempted = 1;
    last_attempt_ms = now_ms;
    motor_state.started = start_can();
    if (!motor_state.started) return;
  }

  motor_get_state(&snapshot);
  remote_get_state(&remote);
  imu_yaw_get_state(&imu);
  motor_link_update(&snapshot, &remote, &imu, HAL_GetTick(), &output);
  if ((hcan1.Instance->ESR & CAN_ESR_BOFF) || HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3) {
    motor_link_stop(LINK_FAULT_CAN);
    HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
    motor_state.skipped_frames++;
    motor_report_record(&snapshot, &remote, &imu, HAL_GetTick());
    return;
  }
  for (int index = 0; index < 2; index++) {
    int command = output.command_raw[index];
    // Last-resort net: each mode already clamped to its own limit (the open-loop test uses a
    // higher one), so this must not clip a valid command.
    if (command > MOTOR_HARD_OUTPUT_LIMIT_RAW) command = MOTOR_HARD_OUTPUT_LIMIT_RAW;
    if (command < -MOTOR_HARD_OUTPUT_LIMIT_RAW) command = -MOTOR_HARD_OUTPUT_LIMIT_RAW;
    float current_a = command / MOTOR_CURRENT_RAW_PER_AMP;
    middleware_motors[index].cmd(current_a * MOTOR_GM6020_TORQUE_PER_AMP);
    middleware_motors[index].write(data);
  }
  header.StdId = middleware_motors[0].tx_id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;
  if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) == HAL_OK) {
    motor_state.queued_frames++;
  } else {
    motor_link_stop(LINK_FAULT_CAN);
    motor_state.skipped_frames++;
  }
  motor_report_record(&snapshot, &remote, &imu, HAL_GetTick());
}

static int16_t read_signed_value(const uint8_t * data)
{
  int32_t value = ((uint16_t)data[0] << 8) | data[1];
  if (value >= 32768) value -= 65536;
  return (int16_t)value;
}

static void update_motor_angle(int index, uint16_t encoder_raw, uint32_t now_ms)
{
  int delta;
  int32_t total;
  if (!motor_state.motor[index].received) {
    motor_state.motor[index].relative_counts = 0;
    motor_state.motor[index].angle_valid = 1;
    return;
  }
  if (!motor_state.motor[index].angle_valid) return;
  if (now_ms - motor_state.motor[index].last_feedback_ms >= MOTOR_ANGLE_MAX_GAP_MS) {
    motor_state.motor[index].angle_valid = 0;
    return;
  }
  delta = (int)encoder_raw - (int)motor_state.motor[index].encoder_raw;
  if (delta == MOTOR_ENCODER_HALF_COUNTS || delta == -MOTOR_ENCODER_HALF_COUNTS) {
    motor_state.motor[index].angle_valid = 0;
    return;
  }
  if (delta > MOTOR_ENCODER_HALF_COUNTS) delta -= MOTOR_ENCODER_COUNTS;
  if (delta < -MOTOR_ENCODER_HALF_COUNTS) delta += MOTOR_ENCODER_COUNTS;
  total = motor_state.motor[index].relative_counts;
  if ((delta > 0 && total > INT32_MAX - delta) || (delta < 0 && total < INT32_MIN - delta)) {
    motor_state.motor[index].angle_valid = 0;
    return;
  }
  motor_state.motor[index].relative_counts = total + delta;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint16_t encoder_raw;
  int index;
  int count;

  if (hcan->Instance != CAN1) return;
  /* FIFO depth is three; bound the work done in one interrupt. */
  for (count = 0; count < 3 && HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); count++) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      motor_state.invalid_frames++;
      return;
    }
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC != 8) {
      motor_state.invalid_frames++;
      continue;
    }
    if (header.StdId == 0x204 + MOTOR_A_ID) {
      index = 0;
    } else if (header.StdId == 0x204 + MOTOR_B_ID) {
      index = 1;
    } else {
      motor_state.invalid_frames++;
      continue;
    }
    encoder_raw = ((uint16_t)data[0] << 8) | data[1];
    if (encoder_raw >= MOTOR_ENCODER_COUNTS) {
      motor_state.invalid_frames++;
      continue;
    }
    for (int byte = 0; byte < 8; byte++) motor_state.motor[index].raw_frame[byte] = data[byte];
    uint32_t now_ms = HAL_GetTick();
    middleware_motors[index].read(data, now_ms);
    update_motor_angle(index, encoder_raw, now_ms);
    motor_state.motor[index].encoder_raw = encoder_raw;
    motor_state.motor[index].speed_rpm = read_signed_value(&data[2]);
    motor_state.motor[index].speed_rad_s = middleware_motors[index].speed;
    motor_state.motor[index].current_raw = read_signed_value(&data[4]);
    motor_state.motor[index].temperature_deg_c = middleware_motors[index].temperature;
    motor_state.motor[index].last_feedback_ms = now_ms;
    motor_state.motor[index].received = 1;
    motor_state.motor[index].frames++;
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef * hcan)
{
  if (hcan->Instance != CAN1) return;
  if (HAL_CAN_GetError(hcan) & (HAL_CAN_ERROR_RX_FOV0 | HAL_CAN_ERROR_BOF)) {
    for (int index = 0; index < 2; index++) {
      if (motor_state.motor[index].received) motor_state.motor[index].angle_valid = 0;
    }
  }
  motor_state.error_events++;
  motor_state.error_flags |= HAL_CAN_GetError(hcan);
}

void motor_get_state(MotorState * state)
{
  int index;
  uint32_t now_ms;
  taskENTER_CRITICAL();
  state->started = motor_state.started;
  state->bus_off = motor_state.bus_off;
  state->queued_frames = motor_state.queued_frames;
  state->skipped_frames = motor_state.skipped_frames;
  state->invalid_frames = motor_state.invalid_frames;
  state->error_events = motor_state.error_events;
  state->error_flags = motor_state.error_flags;
  now_ms = HAL_GetTick();
  for (index = 0; index < 2; index++) {
    for (int byte = 0; byte < 8; byte++)
      state->motor[index].raw_frame[byte] = motor_state.motor[index].raw_frame[byte];
    state->motor[index].relative_counts = motor_state.motor[index].relative_counts;
    state->motor[index].angle_valid = motor_state.motor[index].angle_valid;
    state->motor[index].encoder_raw = motor_state.motor[index].encoder_raw;
    state->motor[index].speed_rpm = motor_state.motor[index].speed_rpm;
    state->motor[index].speed_rad_s = motor_state.motor[index].speed_rad_s;
    state->motor[index].current_raw = motor_state.motor[index].current_raw;
    state->motor[index].temperature_deg_c = motor_state.motor[index].temperature_deg_c;
    state->motor[index].online = middleware_motors[index].is_alive(now_ms);
    state->motor[index].received = motor_state.motor[index].received;
    state->motor[index].last_feedback_ms = motor_state.motor[index].last_feedback_ms;
    state->motor[index].frames = motor_state.motor[index].frames;
  }
  state->bus_off = (hcan1.Instance->ESR & CAN_ESR_BOFF) != 0;
  taskEXIT_CRITICAL();
  for (index = 0; index < 2; index++) {
    state->motor[index].single_angle_rad =
      state->motor[index].encoder_raw * (MOTOR_TWO_PI / MOTOR_ENCODER_COUNTS);
    state->motor[index].relative_angle_rad =
      state->motor[index].relative_counts * (MOTOR_TWO_PI / MOTOR_ENCODER_COUNTS);
    state->motor[index].angle_valid =
      state->motor[index].angle_valid && state->motor[index].received &&
      now_ms - state->motor[index].last_feedback_ms < MOTOR_ANGLE_MAX_GAP_MS;
  }
}
