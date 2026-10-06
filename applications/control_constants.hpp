#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

constexpr int RC_SWITCH_UP = 1;
constexpr int RC_SWITCH_DOWN = 2;
constexpr int RC_SWITCH_MID = 3;
constexpr int RC_MODE_DISABLED = 0;
constexpr int RC_MODE_LINK = 1;
constexpr int RC_MODE_RESET = 2;
constexpr int MOTOR_A_ID = 1;
constexpr int MOTOR_B_ID = 2;
constexpr int MOTOR_FEEDBACK_TIMEOUT_MS = 100;
constexpr int MOTOR_COMMAND_ID = 0x1FF;

constexpr int MOTOR_ENCODER_COUNTS = 8192;
constexpr int MOTOR_ENCODER_HALF_COUNTS = MOTOR_ENCODER_COUNTS / 2;
constexpr int MOTOR_ANGLE_MAX_GAP_MS = 10;
constexpr float MOTOR_TWO_PI = 6.28318530718f;

#endif
