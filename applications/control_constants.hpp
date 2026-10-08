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
// GM6020 current commands for motor IDs 1 to 4.
constexpr int MOTOR_COMMAND_ID = 0x1FE;
constexpr float MOTOR_CURRENT_RAW_PER_AMP = 16384.0f / 3.0f;

constexpr int MOTOR_ENCODER_COUNTS = 8192;
constexpr int MOTOR_ENCODER_HALF_COUNTS = MOTOR_ENCODER_COUNTS / 2;
constexpr int MOTOR_ANGLE_MAX_GAP_MS = 10;
constexpr float MOTOR_TWO_PI = 6.28318530718f;

// Initial control parameters require hardware tuning.
constexpr float MOTOR_POSITION_RAMP_RAD_S = 0.2f;
constexpr float MOTOR_POSITION_SPEED_LIMIT_RAD_S = 0.5f;
constexpr float MOTOR_POSITION_TRIP_SPEED_RAD_S = 1.0f;
constexpr float MOTOR_POSITION_GAIN = 2.0f;
// Speed PID: Kp [A/(rad/s)], Ki [A/rad], Kd [A/(rad/s^2)].
constexpr float MOTOR_SPEED_KP = 0.15f;
constexpr float MOTOR_SPEED_KI = 0.05f;
constexpr float MOTOR_SPEED_KD = 0.0f;
constexpr float MOTOR_SPEED_I_LIMIT_A = 0.10f;
constexpr float MOTOR_SPEED_D_FILTER_S = 0.05f;
constexpr float MOTOR_CURRENT_LIMIT_A = 0.20f;
constexpr int MOTOR_POSITION_OUTPUT_LIMIT_RAW =
  (int)(MOTOR_CURRENT_LIMIT_A * MOTOR_CURRENT_RAW_PER_AMP);
static_assert(MOTOR_CURRENT_LIMIT_A > 0.0f && MOTOR_CURRENT_LIMIT_A <= 3.0f,
              "Current limit must be within the GM6020 command range");
constexpr int MOTOR_POSITION_MAX_TEMP_DEG_C = 60;
constexpr int MOTOR_POSITION_MAX_INTERVAL_MS = 50;

constexpr int MOTOR_A_DIRECTION = 1;
constexpr int MOTOR_B_DIRECTION = 1;
constexpr float MOTOR_LINK_TRAVEL_LIMIT_RAD = MOTOR_TWO_PI;
constexpr float MOTOR_LINK_MAX_ERROR_RAD = 0.523598776f;
static_assert(
  MOTOR_A_ID >= 1 && MOTOR_A_ID <= 4 && MOTOR_B_ID >= 1 && MOTOR_B_ID <= 4 &&
    MOTOR_A_ID != MOTOR_B_ID,
  "Motor IDs must be different and in the 0x1FE current-command group");
static_assert(
  (MOTOR_A_DIRECTION == 1 || MOTOR_A_DIRECTION == -1) &&
    (MOTOR_B_DIRECTION == 1 || MOTOR_B_DIRECTION == -1),
  "Direction must be 1 or -1");

constexpr int MOTOR_MANUAL_SETTLE_MS = 500;
constexpr int MOTOR_MANUAL_TRIGGER_MS = 120;
constexpr int MOTOR_MANUAL_RELEASE_MS = 500;
constexpr float MOTOR_MANUAL_TRIGGER_RAD = 0.034906585f;
constexpr float MOTOR_MANUAL_SETTLED_ERROR_RAD = 0.017453293f;
constexpr float MOTOR_MANUAL_TARGET_QUIET_RAD = 0.005f;
constexpr float MOTOR_MANUAL_YAW_QUIET_RAD = 0.005f;
constexpr float MOTOR_MANUAL_YAW_QUIET_RAD_S = 0.02f;
constexpr float MOTOR_MANUAL_STILL_SPEED_RAD_S = 0.04f;
constexpr float MOTOR_MANUAL_STILL_TRAVEL_RAD = 0.003f;

constexpr int MOTOR_RESET_SETTLE_MS = 500;
constexpr int MOTOR_RESET_TIMEOUT_MS = 30000;
constexpr float MOTOR_RESET_ERROR_RAD = 0.034906585f;
constexpr float MOTOR_RESET_SPEED_RAD_S = 0.04f;

#endif
