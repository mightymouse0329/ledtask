#ifndef CONTROL_CONSTANTS_HPP
#define CONTROL_CONSTANTS_HPP

constexpr int RC_SWITCH_UP = 1;
constexpr int RC_SWITCH_DOWN = 2;
constexpr int RC_SWITCH_MID = 3;
constexpr int RC_MODE_DISABLED = 0;
constexpr int RC_MODE_LINK = 1;
constexpr int RC_MODE_RESET = 2;
// Bench test modes (mode 3 speed-loop spin test, mode 4 open-loop current test). They stay in
// the firmware for bench bring-up but are disabled by default, so the delivered mapping is the
// task specification: right switch UP alone means reset and the left switch is ignored.
constexpr bool MOTOR_BENCH_TEST_MODES = false;
constexpr int RC_MODE_SPIN_TEST = 3;
constexpr int RC_MODE_CURRENT_TEST = 4;
constexpr int MOTOR_A_ID = 1;
constexpr int MOTOR_B_ID = 2;
constexpr int MOTOR_FEEDBACK_TIMEOUT_MS = 100;
// GM6020 current commands for motor IDs 1 to 4.
constexpr int MOTOR_COMMAND_ID = 0x1FE;
constexpr float MOTOR_CURRENT_RAW_PER_AMP = 16384.0f / 3.0f;
constexpr float MOTOR_GM6020_TORQUE_PER_AMP = 0.741f;

constexpr int MOTOR_ENCODER_COUNTS = 8192;
constexpr int MOTOR_ENCODER_HALF_COUNTS = MOTOR_ENCODER_COUNTS / 2;
constexpr int MOTOR_ANGLE_MAX_GAP_MS = 10;
constexpr float MOTOR_TWO_PI = 6.28318530718f;

// Initial control parameters require hardware tuning.
// Overspeed shutdown for the closed-loop modes; not a commanded speed. Kept at roughly twice
// MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S so legitimate tracking never trips it while a real
// runaway still does. The GM6020 speed field is a short-window encoder difference: 8192
// counts per turn means 30 rpm is only about 4 counts per millisecond, so mechanical chatter
// and stick-slip reach small values while the rotor looks stationary; requiring several
// consecutive samples keeps one noisy frame from latching the fault.
constexpr float MOTOR_POSITION_TRIP_SPEED_RAD_S = 200.0f * MOTOR_TWO_PI / 60.0f;
constexpr int MOTOR_POSITION_TRIP_SPEED_DEBOUNCE_CYCLES = 10;
static_assert(
  MOTOR_POSITION_TRIP_SPEED_DEBOUNCE_CYCLES >= 1,
  "Debounce must require at least one over-speed sample");

// Spin test (mode 3), used before an experiment to check that a motor turns: the encoder
// speed loop holds a fixed target speed through the normal current frame 0x1FE, so the
// rotation stays bounded and the test rehearses the linkage inner loop. The GM6020 voltage
// frame 0x1FF is deliberately not used: at 1.5 V these motors drew no current and did not
// move, while the current frame drives them normally.
// 15 rpm sat deep in the low-speed stick-slip regime and in the coarsest part of the encoder
// speed quantization, so the test target is raised.
constexpr float MOTOR_SPIN_TEST_TARGET_RAD_S = 60.0f * MOTOR_TWO_PI / 60.0f;
// The spin test holds a bounded target speed, so its overspeed net only has to catch a real
// runaway. The other modes keep the tighter 30 rpm limit.
constexpr float MOTOR_SPIN_TEST_TRIP_SPEED_RAD_S = 150.0f * MOTOR_TWO_PI / 60.0f;
static_assert(
  MOTOR_SPIN_TEST_TARGET_RAD_S * 2.0f < MOTOR_SPIN_TEST_TRIP_SPEED_RAD_S,
  "Spin test target must stay well below its overspeed limit");
// Position loop: verified at 2.0 (2026-10-08). Raising it reduces the steady-state tracking
// lag (= hand rate / Kp), but this cascade goes unstable long before the lag disappears: at
// Kp = 20 a stopped board left both motors in a ~1 Hz limit cycle swinging +/-17 deg (A) and
// +/-30 deg (B), with the speed command pinned at its cap. Step up 1.5x at a time and use
// "does it settle after the board stops" as the test.
// MOTOR_POSITION_KI is live: the position PID now gets a non-zero integral limit (see
// MOTOR_POSITION_I_LIMIT_RAD_S). Scale first - the integral accumulates err * dt and position
// errors are small (1 deg = 0.0175 rad), so a usable KI is in the tens, not around 1: with
// KI = 1 a 1 deg error moves the integral by only 0.017 rad/s per second and the term looks
// dead, while KI = 10 moves it 0.17 rad/s per second. KD is a raw, unfiltered derivative.
constexpr float MOTOR_POSITION_KP = 9.0f;
constexpr float MOTOR_POSITION_KI = 10.0f;
constexpr float MOTOR_POSITION_KD = 0.1f;
// Integral authority of the position loop, expressed as its contribution to the speed command.
// 0.5 rad/s (about 5 rpm) is enough to break static friction and erase a standing position
// error, and small enough that a large reset step cannot wind the term up into an overshoot.
constexpr float MOTOR_POSITION_I_LIMIT_RAD_S = 0.5f;
// Velocity feedforward for the yaw-following modes (linkage and reset): the reference angular
// rate is added to the position P term, which removes the inherent "faster hand motion means
// larger lag" behaviour of a P-only loop (its steady-state lag is rate / Kp). 1.0 is exact
// feedforward. The sum is still clamped by MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S, so a hand
// rotating faster than that cap still cannot be followed; raise the cap and the overspeed
// threshold together if that is needed.
constexpr float MOTOR_POSITION_FEEDFORWARD = 1.0f;
// Sizing follows the 1:3 left-switch position, where motor B turns three times as fast as the
// board: a hand rate of 30 rpm (180 deg/s) already needs 90 rpm at B, so the old 30 rpm cap
// saturated B as soon as the hand passed 60 deg/s and B visibly lagged A.
constexpr float MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S = 90.0f * MOTOR_TWO_PI / 60.0f;
// The 90 rpm command has to be reachable in about a third of a second, otherwise the ramp
// itself becomes the visible lag at the start of a hand motion.
constexpr float MOTOR_POSITION_TARGET_ACCEL_RAD_S2 = 300.0f * MOTOR_TWO_PI / 60.0f;
// The overspeed net stays a factor of two above the cap.
static_assert(
  MOTOR_POSITION_MAX_TARGET_SPEED_RAD_S * 2.0f < MOTOR_POSITION_TRIP_SPEED_RAD_S,
  "Position speed cap must stay well below the overspeed limit");
// Speed PID output is current in A; input is motor encoder speed in rad/s.
// Measured 2026-10-08: with Kp = 0.3 A/(rad/s) and a 0.15 A limit the loop did not hold a
// speed at all, it sat in a full-scale limit cycle (request flipping between +819 and -819
// raw, speed swinging +/-40 rpm, motor only buzzing). The plant is a small direct-drive
// inertia: its closed-loop time constant J/(Kp*Kt) must stay well above the 2 ms control
// period, so Kp has to be far smaller than the old value. The integral term then supplies
// the friction current that a small P gain cannot reach. Raise Kp again only if the loop
// proves too sluggish.
constexpr float MOTOR_SPEED_KP = 0.02f;
constexpr float MOTOR_SPEED_KI = 0.6f;
constexpr float MOTOR_SPEED_KD = 0.0f;
constexpr float MOTOR_SPEED_I_LIMIT_A = 0.30f;
constexpr float MOTOR_SPEED_D_FILTER_S = 0.05f;
// Raised from 0.15 A (about 0.11 N*m, which sat at the friction threshold so the actuator
// could only stick-slip). 0.45 A is about 0.33 N*m, well inside the GM6020 rating.
constexpr float MOTOR_CURRENT_LIMIT_A = 0.45f;
constexpr int MOTOR_POSITION_OUTPUT_LIMIT_RAW =
  (int)(MOTOR_CURRENT_LIMIT_A * MOTOR_CURRENT_RAW_PER_AMP);
static_assert(
  MOTOR_CURRENT_LIMIT_A > 0.0f && MOTOR_CURRENT_LIMIT_A <= 3.0f,
  "Current limit must be within the GM6020 command range");
constexpr int MOTOR_POSITION_MAX_TEMP_DEG_C = 60;
constexpr int MOTOR_POSITION_MAX_INTERVAL_MS = 50;

// Open-loop bench test (mode 4): both motors receive the current commanded directly by
// the left stick vertical channel, bypassing the position and speed loops. It is the
// only path allowed above MOTOR_CURRENT_LIMIT_A, and it exists to separate "not enough
// torque", "current frame has no effect" and "motor or mechanism is stuck".
constexpr float MOTOR_TEST_CURRENT_LIMIT_A = 0.60f;
constexpr int MOTOR_TEST_OUTPUT_LIMIT_RAW =
  (int)(MOTOR_TEST_CURRENT_LIMIT_A * MOTOR_CURRENT_RAW_PER_AMP);
// The left stick vertical channel spans about +/-660 around centre; ignore drift.
constexpr int MOTOR_TEST_STICK_FULL_SCALE = 660;
constexpr int MOTOR_TEST_STICK_DEADBAND = 30;
// Last-resort clamp applied in motor_service before the CAN frame is built. Each mode
// clamps to its own limit first, so this only covers a coding mistake.
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

// Bench option: allow the control path to run when only one motor reports feedback. The
// silent motor always receives zero output, and every other protection still applies. Only
// the two open-loop test modes make sense with one motor; keep this false for the linkage,
// reset and competition behaviour, where both motors must be healthy before anything moves.
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
// Trigger: a hand can only hold a deviation against the position loop while it beats the
// actuator, so 2 deg / 120 ms asked the operator to win a tug of war before the takeover.
// 1.2 deg / 60 ms takes over as soon as the hand clearly displaces the motor, and the quiet
// gates below still prevent a takeover while the board is being rotated normally.
constexpr int MOTOR_MANUAL_TRIGGER_MS = 60;
constexpr int MOTOR_MANUAL_RELEASE_MS = 500;
constexpr float MOTOR_MANUAL_TRIGGER_RAD = 0.020944f;
constexpr float MOTOR_MANUAL_SETTLED_ERROR_RAD = 0.017453293f;
constexpr float MOTOR_MANUAL_TARGET_QUIET_RAD = 0.005f;
constexpr float MOTOR_MANUAL_YAW_QUIET_RAD = 0.005f;
constexpr float MOTOR_MANUAL_YAW_QUIET_RAD_S = 0.02f;
// Arming must not depend on the reported speed: at a few rpm the GM6020 speed field is
// quantised in steps of about 7 rpm, so the old 0.38 rpm "still" threshold kept restarting the
// 500 ms window and the takeover often never armed - the motor simply kept holding position.
// Arm on displacement instead, with a loose speed sanity check, and decide "the hand let go"
// from displacement only.
constexpr float MOTOR_MANUAL_SETTLE_SPEED_RAD_S = 0.5f;
constexpr float MOTOR_MANUAL_STILL_TRAVEL_RAD = 0.003f;

constexpr int MOTOR_RESET_SETTLE_MS = 1000;
constexpr int MOTOR_RESET_TIMEOUT_MS = 30000;
// Tighter than the old 2 deg so "done" means the arrows really line up; the speed criterion is
// loosened because 0.04 rad/s (0.38 rpm) sits at the speed-field quantisation floor.
constexpr float MOTOR_RESET_ERROR_RAD = 0.020944f;
constexpr float MOTOR_RESET_SPEED_RAD_S = 0.08f;
// Reset approaches the aligned position at its own, slower speed cap: the entry target can be
// up to 180 deg away from the current position, and the linkage cap (90 rpm) would slew that
// at 540 deg/s. 20 rpm is 120 deg/s, so a worst-case half turn takes about 1.5 s.
constexpr float MOTOR_RESET_MAX_TARGET_SPEED_RAD_S = 20.0f * MOTOR_TWO_PI / 60.0f;

#endif
