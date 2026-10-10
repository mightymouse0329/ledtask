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
  uint8_t active;
  uint8_t ready;
  uint8_t fault;
  // Live reason the control path is currently stopped (same codes as fault), refreshed
  // every control cycle. Unlike fault, it is not the latched first fault, so it tells the
  // operator which condition is blocking the unlock right now.
  uint8_t block;
  uint8_t fault_motor;
  uint8_t fault_flags;
  // Live per-motor condition bits (1 offline, 2 angle invalid, 4 feedback aged, 8 non-finite,
  // 16 over temperature, 32 over speed), refreshed every control cycle.
  uint8_t motor_flags[2];
  int16_t fault_speed_rpm;
  uint8_t fault_temperature;
  uint32_t fault_age_ms;
  int16_t fault_previous_output_raw[2];
  int16_t fault_current_raw;
  uint16_t fault_encoder_raw;
  float fault_angle_rad;
  float ratio;
  float yaw_change_rad;
  float yaw_reference_rad;
  float target_rad[2];
  float reference_rad[2];
  float target_speed_rad_s[2];
  int16_t output_raw[2];
};

struct MotorOutput
{
  // Payload of the GM6020 current command frame 0x1FE.
  int16_t command_raw[2];
};

void motor_link_update(
  const MotorState * motor, const RemoteState * remote, const ImuYawState * imu, uint32_t now_ms,
  MotorOutput * output);
void motor_link_stop(int reason);
void motor_link_get_state(LinkState * result);

#endif
