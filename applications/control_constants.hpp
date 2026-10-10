#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

constexpr int RC_SWITCH_UP = 1;
constexpr int RC_SWITCH_DOWN = 2;
constexpr int RC_SWITCH_MID = 3;
constexpr int RC_MODE_DISABLED = 0;
constexpr int RC_MODE_LINK = 1;
constexpr int RC_MODE_RESET = 2;

constexpr bool MOTOR_BENCH_TEST_MODES = false;
constexpr int RC_MODE_SPIN_TEST = 3;
constexpr int RC_MODE_CURRENT_TEST = 4;
constexpr int MOTOR_A_ID = 1;
constexpr int MOTOR_B_ID = 2;
constexpr int MOTOR_FEEDBACK_TIMEOUT_MS = 100;
constexpr int MOTOR_COMMAND_ID = 0x1FE;
constexpr float MOTOR_CURRENT_RAW_PER_AMP = 16384.0f / 3.0f;
constexpr float MOTOR_GM6020_TORQUE_PER_AMP = 0.741f;

constexpr int MOTOR_ENCODER_COUNTS = 8192;
constexpr int MOTOR_ENCODER_HALF_COUNTS = MOTOR_ENCODER_COUNTS / 2;
constexpr int MOTOR_ANGLE_MAX_GAP_MS = 10;
constexpr float MOTOR_TWO_PI = 6.28318530718f;

constexpr float MOTOR_POSITION_TRIP_SPEED_RAD_S = 200.0f * MOTOR_TWO_PI / 60.0f;
constexpr int MOTOR_POSITION_TRIP_SPEED_DEBOUNCE_CYCLES = 10;
static_assert(
  MOTOR_POSITION_TRIP_SPEED_DEBOUNCE_CYCLES >= 1,
  "Debounce must require at least one over-speed sample");

constexpr float MOTOR_SPIN_TEST_TARGET_RAD_S = 60.0f * MOTOR_TWO_PI / 60.0f;

constexpr float MOTOR_SPIN_TEST_TRIP_SPEED_RAD_S = 150.0f * MOTOR_TWO_PI / 60.0f;
static_assert(
  MOTOR_SPIN_TEST_TARGET_RAD_S * 2.0f < MOTOR_SPIN_TEST_TRIP_SPEED_RAD_S,
  "Spin test target must stay well below its overspeed limit");

constexpr float MOTOR_POSITION_KP = 9.0f;
constexpr float MOTOR_POSITION_KI = 10.0f;
constexpr float MOTOR_POSITION_KD = 0.1f;

constexpr float MOTOR_POSITION_I_LIMIT_RAD_S = 0.5f;

constexpr float MOTOR_POSITION_FEEDFORWARD = 1.0f;

constexpr float MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S = 90.0f * MOTOR_TWO_PI / 60.0f;

constexpr float MOTOR_POSITION_TARGET_ACCEL_RAD_S2 = 300.0f * MOTOR_TWO_PI / 60.0f;
static_assert(
  MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S * 2.0f < MOTOR_POSITION_TRIP_SPEED_RAD_S,
  "Position speed cap must stay well below the overspeed limit");

constexpr float MOTOR_SPEED_KP = 0.02f;
constexpr float MOTOR_SPEED_KI = 0.6f;
constexpr float MOTOR_SPEED_KD = 0.0f;
constexpr float MOTOR_SPEED_I_LIMIT_A = 0.30f;
constexpr float MOTOR_SPEED_D_FILTER_S = 0.05f;

constexpr float MOTOR_CURRENT_LIMIT_A = 0.45f;
constexpr int MOTOR_POSITION_OUTPUT_LIMIT_RAW =
  (int)(MOTOR_CURRENT_LIMIT_A * MOTOR_CURRENT_RAW_PER_AMP);
static_assert(
  MOTOR_CURRENT_LIMIT_A > 0.0f && MOTOR_CURRENT_LIMIT_A <= 3.0f,
  "Current limit must be within the GM6020 command range");
constexpr int MOTOR_POSITION_MAX_TEMP_DEG_C = 60;
constexpr int MOTOR_POSITION_MAX_INTERVAL_MS = 50;

constexpr float MOTOR_TEST_CURRENT_LIMIT_A = 0.60f;
constexpr int MOTOR_TEST_OUTPUT_LIMIT_RAW =
  (int)(MOTOR_TEST_CURRENT_LIMIT_A * MOTOR_CURRENT_RAW_PER_AMP);
constexpr int MOTOR_TEST_STICK_FULL_SCALE = 660;
constexpr int MOTOR_TEST_STICK_DEADBAND = 30;

constexpr float MOTOR_HARD_CURRENT_LIMIT_A = 0.60f;
constexpr int MOTOR_HARD_OUTPUT_LIMIT_RAW =
  (int)(MOTOR_HARD_CURRENT_LIMIT_A * MOTOR_CURRENT_RAW_PER_AMP);
static_assert(
  MOTOR_HARD_CURRENT_LIMIT_A >= MOTOR_CURRENT_LIMIT_A &&
    MOTOR_HARD_CURRENT_LIMIT_A >= MOTOR_TEST_CURRENT_LIMIT_A,
  "Hard limit must not clip any control mode");
static_assert(
  MOTOR_TEST_STICK_FULL_SCALE > MOTOR_TEST_STICK_DEADBAND,
  "Stick full scale must exceed the deadband");

constexpr int MOTOR_A_DIRECTION = 1;
constexpr int MOTOR_B_DIRECTION = 1;

constexpr bool MOTOR_ALLOW_SINGLE_ONLINE = false;
static_assert(
  MOTOR_A_ID >= 1 && MOTOR_A_ID <= 4 && MOTOR_B_ID >= 1 && MOTOR_B_ID <= 4 &&
    MOTOR_A_ID != MOTOR_B_ID,
  "Motor IDs must be different and in the 0x1FE current-command group");
static_assert(
  (MOTOR_A_DIRECTION == 1 || MOTOR_A_DIRECTION == -1) &&
    (MOTOR_B_DIRECTION == 1 || MOTOR_B_DIRECTION == -1),
  "Direction must be 1 or -1");

constexpr int MOTOR_MANUAL_SETTLE_MS = 500;

constexpr int MOTOR_MANUAL_TRIGGER_MS = 60;
constexpr int MOTOR_MANUAL_RELEASE_MS = 500;
constexpr float MOTOR_MANUAL_TRIGGER_RAD = 0.020944f;
constexpr float MOTOR_MANUAL_SETTLED_ERROR_RAD = 0.017453293f;
constexpr float MOTOR_MANUAL_TARGET_QUIET_RAD = 0.005f;
constexpr float MOTOR_MANUAL_YAW_QUIET_RAD = 0.005f;
constexpr float MOTOR_MANUAL_YAW_QUIET_RAD_S = 0.02f;

constexpr float MOTOR_MANUAL_SETTLE_SPEED_RAD_S = 0.5f;
constexpr float MOTOR_MANUAL_STILL_TRAVEL_RAD = 0.003f;

constexpr int MOTOR_RESET_SETTLE_MS = 1000;
constexpr int MOTOR_RESET_TIMEOUT_MS = 30000;

constexpr float MOTOR_RESET_ERROR_RAD = 0.020944f;
constexpr float MOTOR_RESET_SPEED_RAD_S = 0.08f;

constexpr float MOTOR_RESET_MAX_TARGET_SPEED_RAD_S = 20.0f * MOTOR_TWO_PI / 60.0f;

#endif
