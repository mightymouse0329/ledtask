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

// Bench experiment only. Confirm hardware and tune before enabling.
constexpr bool MOTOR_POSITION_ENABLE = false;
constexpr float MOTOR_POSITION_TEST_OFFSET_RAD = 0.174532925f;
constexpr float MOTOR_POSITION_RAMP_RAD_S = 0.2f;
constexpr float MOTOR_POSITION_SPEED_LIMIT_RAD_S = 0.5f;
constexpr float MOTOR_POSITION_TRIP_SPEED_RAD_S = 1.0f;
constexpr float MOTOR_POSITION_TRAVEL_LIMIT_RAD = 0.34906585f;
constexpr float MOTOR_POSITION_GAIN = 2.0f;
constexpr float MOTOR_SPEED_GAIN = 1000.0f;
constexpr int MOTOR_POSITION_OUTPUT_LIMIT_RAW = 1500;
constexpr int MOTOR_POSITION_MAX_TEMP_DEG_C = 60;
constexpr int MOTOR_POSITION_MAX_INTERVAL_MS = 50;

#endif
