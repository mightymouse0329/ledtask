#ifndef MOTOR_LINK_HPP
#define MOTOR_LINK_HPP

#include "imu_yaw.hpp"
#include "motor_control.h"
#include "remote_control.h"

constexpr int LINK_FAULT_NONE = 0;
constexpr int LINK_FAULT_FEEDBACK = 1;
constexpr int LINK_FAULT_IMU = 2;
constexpr int LINK_FAULT_REMOTE = 3;
constexpr int LINK_FAULT_CAN = 4;
constexpr int LINK_FAULT_TIMING = 5;
constexpr int LINK_FAULT_LIMIT = 6;
constexpr int LINK_FAULT_MANUAL = 7;
constexpr int LINK_FAULT_RESET = 8;

struct LinkState
{
  uint8_t unlocked;
  uint8_t mode;
  uint32_t unlock_ms;
  uint8_t active;
  uint8_t ready;
  uint8_t fault;
  float ratio;
  float yaw_change_rad;
  float yaw_reference_rad;
  float target_rad[2];
  float reference_rad[2];
  int16_t output_raw[2];
};

void motor_link_update(
  const MotorState * motor, const RemoteState * remote, const ImuYawState * imu, uint32_t now_ms,
  int16_t commands[2]);
void motor_link_stop(int reason);
void motor_link_get_state(LinkState * result);

#endif
