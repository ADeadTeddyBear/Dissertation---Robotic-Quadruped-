// ============================================================
// QuadSensors.ino
// Hip servo control + MPU6050 + dual VL53L0X (TOF400C)
//
// Libraries required (install via Library Manager):
//   VL53L0X  by Pololu
//   (MPU6050 driven via raw Wire — no extra library needed)
//
// VL53L0X notes (different chip/library from VL53L1X):
//   - Max realistic range ~2m, vs VL53L1X's 4m.
//   - No distance-mode selector; range/noise-immunity is traded off
//     via setSignalRateLimit() + setVcselPulsePeriod() instead.
//   - No per-reading range_status enum, so "no target" is inferred
//     from the raw distance exceeding TOF_MAX_MM.
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

// Declared this early so it's a known type wherever Arduino's
// auto-generated function prototypes land (right after the
// #includes, before anything else in the file) -- see the VERIFIED
// CLIMB TIERS comment further down for what this is actually for.
struct ClimbPose {
  int hipFL, kneeFL, hipFR, kneeFR, hipRL, kneeRL, hipRR, kneeRR;
};

// Same reason as ClimbPose above: handleCommand() (which starts/
// cancels a square-up) and startDrive()/startDriveToTof() (which
// check it to avoid driving while turning) both come before the
// SQUARE-UP section further down that actually uses this -- only
// functions are auto-prototyped by Arduino, not enums or plain
// globals, so this has to live here instead.
enum SquareState { SQUARE_IDLE, SQUARE_TURN_PULSE, SQUARE_SETTLE, SQUARE_VERIFY };
SquareState   squareState = SQUARE_IDLE;
int           squareAttempts = 0;
unsigned long squareStateStartMs = 0;
unsigned long squareCurrentPulseMs = 0; // set by applySquareTurn(), proportional to the error size -- see its comment

// Same reason: handleCommand()'s drive_stop branch cancels an
// in-progress turn test, but comes before the TURN TEST section
// further down that actually defines this.
bool          turnTestActive = false;
unsigned long turnTestStopAtMs = 0;

// Same reason again: LIFT_REVERSE (in updateLiftSequence(), well
// above the DRIVE section that normally defines these) needs to read
// driveActive to know when the pre-lift reverse has finished.
bool          driveActive = false;
unsigned long driveStopAtMs = 0;
float         driveTofTargetMM = -1; // -1 = plain timed drive, no ToF stop condition
bool          driveTofApproaching = false; // true: stop once tof1_mm <= target (closing in); false: stop once tof1_mm >= target (backing away)
float         driveTofStartMM = -1; // ToF1 reading at the moment a ToF-guided drive started -- see DRIVE_TOF_AWAY_JUMP_MM below

// Same reason again: handleCommand()'s drive_stop branch (well above
// the RAISE REAR section that normally defines this) needs to read
// raiseRearState to know whether there's a rear-raise to cancel.
enum RaiseRearState { RAISE_REAR_IDLE, RAISE_REAR_STEPPING, RAISE_REAR_DRIVING, RAISE_REAR_SETTLING };
RaiseRearState raiseRearState = RAISE_REAR_IDLE;

// Same reason again: startLiftSequence() (well above the RAISE REAR
// section that normally defines this) needs to reset this flag at the
// start of every new climb -- without it, re-invoking raise_rear after
// it stops short of level silently re-captures a fresh (already-
// advanced) front hip/knee start each time, throwing off the progress
// fraction the front legs interpolate against.
bool raiseRearFrontKneeCaptured = false;

// ============================================================
// HIP SERVO PINS
// ============================================================
#define HIP_FL_PIN   6
#define HIP_FR_PIN   5
// RL/RR pin numbers were swapped here (8/7, not 7/8) to correct a
// naming mixup: what the code called "RL" was actually wired to the
// physical right-rear leg and vice versa. Direction/mirroring was
// already correct for each physical leg, so the fix is just naming --
// the calibration below (mirror, trim) moved together with its pin.
#define HIP_RL_PIN   8
#define HIP_RR_PIN   7

// ============================================================
// KNEE SERVO PINS
// RL/RR are reserved but not wired yet; they follow the same pattern
// once those legs are physically added.
// ============================================================
#define KNEE_FL_PIN  30
#define KNEE_FR_PIN  31
#define KNEE_RL_PIN  32
#define KNEE_RR_PIN  33

// ============================================================
// WHEEL DRIVE MOTOR PINS (L298N, one channel per wheel)
// Each wheel gets its own IN1/IN2/EN, fully independent in hardware,
// even though updateDrive()/setWheelSpeeds() below drive all four in
// sync for now -- see startDrive()'s comment for why (no wheel
// encoders exist in this codebase, so driving is open-loop/timed).
// ============================================================
#define WHEEL_FL_IN1 26
#define WHEEL_FL_IN2 27
#define WHEEL_FL_EN  11
#define WHEEL_FR_IN1 28
#define WHEEL_FR_IN2 29
#define WHEEL_FR_EN  12
#define WHEEL_RL_IN1 24
#define WHEEL_RL_IN2 25
#define WHEEL_RL_EN  10
#define WHEEL_RR_IN1 22
#define WHEEL_RR_IN2 23
#define WHEEL_RR_EN   9

// ============================================================
// VL53L0X XSHUT PINS
// ============================================================
#define XSHUT_1   A0
#define XSHUT_2   A1
#define TOF2_ADDR 0x52

// ============================================================
// MPU6050 REGISTERS
// ============================================================
#define MPU_ADDR      0x68
#define PWR_MGMT_1    0x6B
#define ACCEL_XOUT_H  0x3B

// ============================================================
// SERVO CONFIG
// ============================================================
enum HipIndex { FL = 0, FR, RL, RR, NUM_HIPS };

const int  HIP_PINS[NUM_HIPS]   = { HIP_FL_PIN, HIP_FR_PIN, HIP_RL_PIN, HIP_RR_PIN };
// FL/FR are mechanically limited to 6/2 as their furthest inward point --
// going lower risks the leg colliding with/damaging the robot. RR
// confirmed clash-free all the way down to 0. RL raised 0->4 after the
// replacement hip servo: confirmed on hardware that 0 goes far enough
// to risk pinching a wire, so 4 is the new floor for that leg specifically.
const int  HIP_MIN[NUM_HIPS]    = {   6,   2,   4,   0 };
// FL/FR confirmed by manual cautious jogging (from the low/splayed
// lift stance) to be safe up to 220. RL/RR confirmed clash-free up
// to 200 -- the rear doesn't need the same reach as the front since
// it won't be climbing stairs backwards.
const int  HIP_MAX[NUM_HIPS]    = { 220, 220, 200, 200 };
// 30 = leg straight down (the home pose for IK); 0-30 lets the leg swing
// inward a bit from there, 30-170 covers the rest of its outward/upward travel.
const int  HIP_START[NUM_HIPS]  = { 30, 30, 30, 30 };
// Right side servos are mounted opposite — mirror their angle so
// sending 30 to FL and FR both means "straight down". RL/RR swapped
// here to match the pin renaming above -- each mirror value stayed
// attached to its actual physical leg.
const bool HIP_MIRROR[NUM_HIPS] = { false, true, true, false }; // FL, FR, RL, RR
// Per-servo calibration: added before mirroring so commanding the same
// logical angle (e.g. 30) points every leg straight down, regardless of
// how each servo horn happens to be seated. Fill in from the by-eye
// calibration: trim = (angle that looked straight down) - 30.
// FL re-calibrated again: 44 looked straight -> trim +14.
// RR re-calibrated after fixing an under-voltage issue that was
// causing unreliable servo behavior on the rear knees: 40 straight
// -> trim +10.
// FR calibrated: 67 looked straight down -> trim +37.
// RL recalibrated after replacing its hip servo: crouch-low (logical
// 140) now needs 130 to look level, so trim = 30 + (130-140) = 20.
const int  HIP_TRIM[NUM_HIPS]   = { 14, 37, 20, 10 };

const char* HIP_NAMES[NUM_HIPS] = { "hip_fl", "hip_fr", "hip_rl", "hip_rr" };

// These are 270-degree servos (500-2500us pulse width, per datasheet).
// Arduino's Servo::write() only accepts 0-180 as a "degrees" argument and
// silently clamps anything above that, which flattens the whole upper half
// of a 270-degree range onto a single position. Driving the pulse width
// directly via writeMicroseconds() avoids that clamp entirely.
#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_MAX_US 2500

Servo hipServos[NUM_HIPS];
int   hipPos[NUM_HIPS];

// ============================================================
// KNEE CONFIG
// Same servo model as the hips (500-2500us, 270 degrees), so the
// same pulse mapping applies. All four legs now wired; kneeInstalled
// still gates setup()/updateServoMotion() as a safety net in case a
// knee ever needs to be temporarily pulled back out.
// ============================================================
const int  KNEE_PINS[NUM_HIPS]     = { KNEE_FL_PIN, KNEE_FR_PIN, KNEE_RL_PIN, KNEE_RR_PIN };
bool       kneeInstalled[NUM_HIPS] = { true, true, true, true };
// RL/RR confirmed by testing: below 20, the knee/calf touches the
// ground before the wheel does, so 20 is the floor for lowering the
// rear stance -- AT THE HIP ANGLE THAT TEST USED (the normal
// standing/walking range, hip ~30 degrees and up). Ground clearance
// depends on hip AND knee together, not knee alone: separately
// confirmed by testing that with the hip down near 0 (the tucked
// pre-climb rear stance), the knee/calf stays clear of the ground at
// values below 20 that would have hit the ground at the higher hip
// angle the original 20 was calibrated against. See setKnee() below --
// the relaxed floor only applies when the hip is actually down in that
// tested low range, not universally, so the original hip~30+ case
// keeps its original protection.
const int  KNEE_MIN[NUM_HIPS]      = {   0,   0,  20,  20 };
const int  KNEE_MAX[NUM_HIPS]      = { 270, 270, 270, 270 };
// Below this hip angle, RL/RR are confirmed clear of the ground even
// with the knee relaxed to REAR_KNEE_MIN_LOW_HIP -- only tested around
// hip 0-10 degrees so far (the tucked pre-climb stance), not smoothly
// characterized across the whole range in between, so this is a
// conservative gate rather than a derived boundary.
#define REAR_KNEE_LOW_HIP_THRESHOLD_DEG 15
#define REAR_KNEE_MIN_LOW_HIP 0
// FL/FR re-calibrated again after fixing the under-voltage issue:
// FL straight at 125, FR straight at 130. RL/RR default to the
// servo's datasheet neutral (1500us) pending their own by-eye
// calibration, now that voltage is fixed.
const int  KNEE_START[NUM_HIPS]    = { 125, 130, 135, 135 };
// Right side knees are mounted opposite, same as the hips -- mirror
// their angle so the same logical angle bends both sides the same way.
const bool KNEE_MIRROR[NUM_HIPS]   = { false, true, false, true }; // FL, FR, RL, RR
const char* KNEE_NAMES[NUM_HIPS]   = { "knee_fl", "knee_fr", "knee_rl", "knee_rr" };

Servo kneeServos[NUM_HIPS];
int   kneePos[NUM_HIPS];

// ============================================================
// SENSOR OBJECTS
// ============================================================
VL53L0X tof1, tof2;
bool tof1Active = false;
bool tof2Active = false;

#define TOF_MAX_MM 2000  // VL53L0X's realistic range ceiling; beyond this treat as "no target"

uint16_t tof1_mm = 0, tof2_mm = 0;
bool     tof1_ok = false, tof2_ok = false;

// Small rolling-average filter for both ToF sensors -- confirmed on
// hardware that a single instantaneous reading carries several mm of
// noise (each sensor's own std dev ~2-3mm, see square-up's comment),
// enough to matter for anything comparing two readings against a
// tight tolerance. Pushed every pollTofSensors() call so it's already
// warm by the time anything needs it; only pushes when BOTH readings
// are currently valid, keeping the two buffers in sync sample-for-
// sample. Currently used by square-up only -- other systems (the step
// scan, drive_to) still read tof1_mm/tof2_mm directly and are
// unaffected.
#define TOF_FILTER_N 5
uint16_t tof1FilterBuf[TOF_FILTER_N] = {0};
uint16_t tof2FilterBuf[TOF_FILTER_N] = {0};
int      tofFilterIdx = 0;
int      tofFilterCount = 0;

void pushTofFilterSample() {
  if (!tof1_ok || !tof2_ok) return;
  tof1FilterBuf[tofFilterIdx] = tof1_mm;
  tof2FilterBuf[tofFilterIdx] = tof2_mm;
  tofFilterIdx = (tofFilterIdx + 1) % TOF_FILTER_N;
  if (tofFilterCount < TOF_FILTER_N) tofFilterCount++;
}

float tof1Filtered() {
  if (tofFilterCount == 0) return (float)tof1_mm;
  long sum = 0;
  for (int i = 0; i < tofFilterCount; i++) sum += tof1FilterBuf[i];
  return (float)sum / tofFilterCount;
}

float tof2Filtered() {
  if (tofFilterCount == 0) return (float)tof2_mm;
  long sum = 0;
  for (int i = 0; i < tofFilterCount; i++) sum += tof2FilterBuf[i];
  return (float)sum / tofFilterCount;
}

#define FIRMWARE_BUILD "QuadSensors build 2026-07-25-l (VL53L0X)"

// ============================================================
// SERVO HELPERS
// ============================================================
// setHip()/setKnee() no longer jump straight to the target angle --
// they hand it to the smooth-motion system below, which eases the
// servo there over updateServoMotion() calls instead of snapping at
// full speed.
void setHip(int i, int angle) {
  angle = constrain(angle, HIP_MIN[i], HIP_MAX[i]);
  startHipMove(i, angle);
}

void setKnee(int i, int angle) {
  int kneeMin = KNEE_MIN[i];
  if ((i == RL || i == RR) && hipPos[i] <= REAR_KNEE_LOW_HIP_THRESHOLD_DEG) {
    kneeMin = REAR_KNEE_MIN_LOW_HIP; // confirmed clear of the ground at this low a hip angle -- see KNEE_MIN comment above
  }
  angle = constrain(angle, kneeMin, KNEE_MAX[i]);
  startKneeMove(i, angle);
}

// ============================================================
// SMOOTH SERVO MOTION
// A cosine ease (zero slope at both ends, so motion ramps up then
// back down instead of snapping to full speed and stopping dead)
// applied over a duration scaled to the size of the move, so small
// and large moves both take a sensible amount of time. Tune
// *_MOVE_DEG_PER_SEC by feel -- both are well under the servos'
// actual max slew rate, leaving room to move slower than the
// hardware's limit.
// ============================================================
#define HIP_MOVE_DEG_PER_SEC  180.0
#define KNEE_MOVE_DEG_PER_SEC 180.0
#define MOVE_MIN_MS           50UL

// Global slowdown knob for startHipMove()/startKneeMove() -- 1.0 is
// full speed (the rates above), smaller values stretch move duration
// proportionally. Used to make the lift/step-placement sequence move
// more gradually than routine commands, the same way manually jogging
// it in small careful steps avoids sudden weight transfer -- see
// LIFT_MOVE_SPEED_SCALE below.
float moveSpeedScale = 1.0;

float hipMoveFrom[NUM_HIPS], hipMoveTo[NUM_HIPS];
unsigned long hipMoveStartMs[NUM_HIPS], hipMoveDurationMs[NUM_HIPS];

float kneeMoveFrom[NUM_HIPS], kneeMoveTo[NUM_HIPS];
unsigned long kneeMoveStartMs[NUM_HIPS], kneeMoveDurationMs[NUM_HIPS];

float easeInOut(float progress) {
  return (1.0 - cos(progress * PI)) / 2.0;
}

// Physically writes a hip servo to an exact logical angle right now --
// bypasses easing. Used internally by updateServoMotion() as it steps
// through a move.
void applyHipAngle(int i, float angle) {
  hipPos[i] = (int)round(angle);
  int trimmed = (int)constrain(round(angle) + HIP_TRIM[i], HIP_MIN[i], HIP_MAX[i]);
  int physical = HIP_MIRROR[i] ? (270 - trimmed) : trimmed;
  int pulse = map(physical, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  hipServos[i].writeMicroseconds(pulse);
}

void applyKneeAngle(int i, float angle) {
  kneePos[i] = (int)round(angle);
  int physical = KNEE_MIRROR[i] ? (270 - kneePos[i]) : kneePos[i];
  int pulse = map(physical, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  kneeServos[i].writeMicroseconds(pulse);
}

void startHipMove(int i, int angle) {
  hipMoveFrom[i] = hipPos[i];
  hipMoveTo[i]   = angle;
  hipMoveStartMs[i] = millis();
  hipMoveDurationMs[i] = max((unsigned long)(fabs(angle - hipPos[i]) / (HIP_MOVE_DEG_PER_SEC * moveSpeedScale) * 1000.0), MOVE_MIN_MS);
}

void startKneeMove(int i, int angle) {
  kneeMoveFrom[i] = kneePos[i];
  kneeMoveTo[i]   = angle;
  kneeMoveStartMs[i] = millis();
  kneeMoveDurationMs[i] = max((unsigned long)(fabs(angle - kneePos[i]) / (KNEE_MOVE_DEG_PER_SEC * moveSpeedScale) * 1000.0), MOVE_MIN_MS);
}

// Steps every in-progress move forward -- call every loop() pass.
void updateServoMotion() {
  unsigned long now = millis();
  for (int i = 0; i < NUM_HIPS; i++) {
    unsigned long elapsed = now - hipMoveStartMs[i];
    float progress = (float)elapsed / (float)hipMoveDurationMs[i];
    if (progress >= 1.0) {
      applyHipAngle(i, hipMoveTo[i]);
    } else {
      applyHipAngle(i, hipMoveFrom[i] + (hipMoveTo[i] - hipMoveFrom[i]) * easeInOut(progress));
    }

    if (!kneeInstalled[i]) continue;
    unsigned long kElapsed = now - kneeMoveStartMs[i];
    float kProgress = (float)kElapsed / (float)kneeMoveDurationMs[i];
    if (kProgress >= 1.0) {
      applyKneeAngle(i, kneeMoveTo[i]);
    } else {
      applyKneeAngle(i, kneeMoveFrom[i] + (kneeMoveTo[i] - kneeMoveFrom[i]) * easeInOut(kProgress));
    }
  }
}

// ============================================================
// LEG INVERSE KINEMATICS
// Coordinate frame: origin at the leg's own hip pivot, x = forward
// (+), y = down (+). HIP_START[i] is thigh straight down (theta1 =
// 0); KNEE_START[i] is calf in line with the thigh, fully extended
// (theta2 = 0) -- both referenced directly from the calibrated
// constants rather than hardcoded, so a recalibration (e.g. after
// reassembling a leg) can't silently desync the IK math from the
// actual servo zero points. Both joints share the same sign
// convention on every leg -- mirroring is handled transparently by
// setHip()/setKnee() -- below the straight reference bends the
// segment back toward the chassis, above bends it forward,
// confirmed by testing on FL.
//
// Every reachable (x, y) has two elbow solutions (knee folded
// backward or forward) that land on the same foot point but leave
// the calf/wheel pointing a different way. solveLegIK() picks
// whichever keeps the calf closer to vertical (wheel facing down
// rather than leaning further out), which for a forward target
// reduces to the backward-fold branch above and only flips to the
// other branch for backward/extreme targets where it tucks the foot
// inward instead. Confirmed by testing on all four legs.
//
// SAFETY: the "full combined range (thigh and knee both bent back to
// their limits at once) doesn't hit the chassis" check is now
// confirmed on all four legs. No combined-angle limit is enforced
// beyond each leg's own HIP_MIN/MAX and KNEE_MIN/MAX -- mirrored legs
// aren't guaranteed identical clearance (the same reason FR needed its
// own hip_fr minimum (2) distinct from FL's (6)), so a future
// recalibration of any leg's HIP_MIN/MAX/KNEE_MIN/MAX should be
// re-checked against the chassis before trusting foot_* near the edge
// of that leg's workspace again.
// ============================================================
const float LEG_THIGH_MM = 165.0;
// hip-to-knee measured 165mm, knee-to-ground measured 150mm -- but the
// 150mm turned out to still NOT include the wheel despite the note it
// once carried, confirmed by cross-checking against a real measured
// hip-pivot height (275mm) in the tall verified pose: FR's predicted
// height matched to within 7mm once the wheel's 45mm radius (90mm
// diameter) was added to the calf length (150+45=195), while leaving
// it at 150 was off by 36mm. RL/RR still don't match even with this
// fix (they're off by ~115mm regardless) -- that's a separate,
// still-open rear-leg calibration problem, not a wheel-radius issue
// (confirmed: at RL/RR's tested angle the calf sits nearly horizontal,
// so its length barely affects height either way).
const float LEG_CALF_MM  = 195.0;

// Solves 2-link planar IK for leg i. (x, y) is the desired foot
// position relative to that leg's hip pivot, in mm (x forward+, y
// down+). Returns false if the target is out of reach; otherwise
// fills hipAngleOut/kneeAngleOut with servo angles. These are not yet
// clamped to leg i's HIP_MIN/MAX or KNEE_MIN/MAX -- setFoot() still
// routes them through setHip()/setKnee(), which enforce those.
// forceBranch: -1 (default) picks whichever of the two elbow solutions
// is closest to the leg's current commanded angles, as described below.
// 0 forces the backward fold, 1 forces the forward fold, regardless of
// continuity -- used to deliberately hold a stance leg in the OTHER
// elbow configuration for the same foot point (see the rear-knee-fold
// experiment in LIFT_SHIFTING).
bool solveLegIK(int i, float x, float y, float &hipAngleOut, float &kneeAngleOut, int forceBranch = -1) {
  float d2 = x * x + y * y;
  float d  = sqrt(d2);
  if (d > (LEG_THIGH_MM + LEG_CALF_MM) || d < fabs(LEG_THIGH_MM - LEG_CALF_MM)) {
    return false; // unreachable
  }

  float cosKnee = (d2 - LEG_THIGH_MM * LEG_THIGH_MM - LEG_CALF_MM * LEG_CALF_MM)
                  / (2.0 * LEG_THIGH_MM * LEG_CALF_MM);
  cosKnee = constrain(cosKnee, -1.0, 1.0);
  float kneeMag = acos(cosKnee);

  // Try both elbow solutions -- knee folded backward (toward the
  // chassis, matching a normal walking gait where the foot lifts by
  // folding the knee back and up) or forward -- and keep whichever
  // requires the SMALLER total change from the leg's current commanded
  // angles (hipPos[i]/kneePos[i]), not whichever looks most vertical.
  //
  // Confirmed necessary on hardware: solving fresh for a nearby target
  // (e.g. a small weight-shift nudge) with a pure "closest to
  // vertical" tie-break can pick a branch that's mathematically valid
  // but 100+ degrees of combined hip+knee rotation away from where the
  // leg actually is -- for a rear leg mid-crouch, this snapped it into
  // a completely different, uncommanded configuration instead of the
  // small incremental move that was intended. Preferring continuity
  // avoids that regardless of which region of the workspace the
  // target falls in, and reproduces the same choice as the old
  // heuristic for every already-tested forward-reaching case (a normal
  // reach from a normal stance was never near a branch boundary to
  // begin with). Branch 0 (backward fold) is tried first and kept on
  // an exact tie.
  //
  // forceBranch skips the continuity comparison entirely and just
  // takes that one branch's solution -- same underlying math, just no
  // choice involved.
  float bestHip = 0, bestKnee = 0, bestAngleChange = -1;
  for (int branch = 0; branch < 2; branch++) {
    if (forceBranch >= 0 && branch != forceBranch) continue;
    float theta2 = (branch == 0) ? -kneeMag : kneeMag;
    float k1 = LEG_THIGH_MM + LEG_CALF_MM * cos(theta2);
    float k2 = LEG_CALF_MM * sin(theta2);
    float theta1 = atan2(x, y) - atan2(k2, k1);
    float hipCandidate  = degrees(theta1) + HIP_START[i];
    float kneeCandidate = degrees(theta2) + KNEE_START[i];
    float angleChange = fabs(hipCandidate - hipPos[i]) + fabs(kneeCandidate - kneePos[i]);
    if (bestAngleChange < 0 || angleChange < bestAngleChange) {
      bestAngleChange = angleChange;
      bestHip  = hipCandidate;
      bestKnee = kneeCandidate;
    }
  }

  hipAngleOut  = bestHip;
  kneeAngleOut = bestKnee;
  return true;
}

// Moves leg i's foot to (x, y) mm relative to its hip pivot. Returns
// false (leaving the servos untouched) if unreachable. Synchronizes
// the hip's and knee's move durations so they arrive together --
// otherwise whichever joint has the smaller move finishes first and
// the foot arcs through an unintended path for the rest of the move
// (each joint still eases independently in angle-space, so this
// isn't a true straight-line Cartesian path, just a closer
// approximation than leaving the durations independent).
bool setFoot(int i, float x, float y, int forceBranch = -1) {
  float hipAngle, kneeAngle;
  if (!solveLegIK(i, x, y, hipAngle, kneeAngle, forceBranch)) return false;
  setHip(i, (int)round(hipAngle));
  setKnee(i, (int)round(kneeAngle));
  unsigned long dur = max(hipMoveDurationMs[i], kneeMoveDurationMs[i]);
  hipMoveDurationMs[i]  = dur;
  kneeMoveDurationMs[i] = dur;
  return true;
}

void allHips(int angle) {
  for (int i = 0; i < NUM_HIPS; i++) setHip(i, angle);
}

// Solves the "amount" (degrees) that puts the FRONT foot at depth
// heightMM below its hip, given front hip/knee move by equal and
// opposite amounts so their rotations exactly cancel
// (theta1 + theta2 = 0), reducing height to
// LEG_THIGH_MM*cos(amount) + LEG_CALF_MM.
float frontAmountForHeight(float heightMM) {
  float c = (heightMM - LEG_CALF_MM) / LEG_THIGH_MM;
  c = constrain(c, -1.0, 1.0);
  return degrees(acos(c));
}

// Solves the "amount" (degrees) that puts the REAR foot at depth
// heightMM below its hip. Unlike the front, rear hip/knee rotations
// don't cancel (theta1 + theta2 = -2*amount), so via the double-angle
// identity height works out to a quadratic in cos(amount):
// 2*LEG_CALF_MM*u^2 + LEG_THIGH_MM*u - (LEG_CALF_MM + heightMM) = 0.
// This is why front and rear need different internal amounts to
// reach the same height -- rear's rotations compound, front's don't.
float rearAmountForHeight(float heightMM) {
  float A = 2.0 * LEG_CALF_MM;
  float B = LEG_THIGH_MM;
  float C = -(LEG_CALF_MM + heightMM);
  float disc = B * B - 4.0 * A * C;
  if (disc < 0) disc = 0;
  float u = (-B + sqrt(disc)) / (2.0 * A);
  u = constrain(u, -1.0, 1.0);
  return degrees(acos(u));
}

// Computes leg i's (FL/FR) paired hip/knee target for depth heightMM
// and validates BOTH against that leg's real HIP_MIN/MAX and
// KNEE_MIN/MAX before returning. Returns false (leaving hipOut/
// kneeOut untouched) if either joint would need to go past its real
// limit -- this is what setBodyHeight() used to skip, letting
// setHip()/setKnee() clamp just one of the pair and silently break
// the hip/knee relationship the height formula depends on.
bool computeFrontJointsForHeight(int i, float heightMM, int &hipOut, int &kneeOut) {
  float aFront = frontAmountForHeight(heightMM);
  int hip  = (int)round(HIP_START[i] + aFront);
  int knee = (int)round(KNEE_START[i] - aFront);
  if (hip < HIP_MIN[i] || hip > HIP_MAX[i] || knee < KNEE_MIN[i] || knee > KNEE_MAX[i]) return false;
  hipOut = hip;
  kneeOut = knee;
  return true;
}

// Same as computeFrontJointsForHeight(), for leg i (RL/RR).
bool computeRearJointsForHeight(int i, float heightMM, int &hipOut, int &kneeOut) {
  float bRear = rearAmountForHeight(heightMM);
  int hip  = (int)round(HIP_START[i] - bRear);
  int knee = (int)round(KNEE_START[i] - bRear);
  if (hip < HIP_MIN[i] || hip > HIP_MAX[i] || knee < KNEE_MIN[i] || knee > KNEE_MAX[i]) return false;
  hipOut = hip;
  kneeOut = knee;
  return true;
}

// Sets the body to heightMM, level front-to-back, while preserving
// the confirmed joint-bend directions (front hip+/knee-, rear
// hip-/knee-) -- replaces the earlier uniform setFoot(i, 0, height)
// version, which was level (identical formula for every leg) but
// made every leg's hip/knee move the SAME direction rather than the
// asymmetric relationship found by testing.
//
// Validates all four legs' computed hip/knee pairs against their real
// limits before moving anything -- if any leg can't reach heightMM
// without one of its two joints clamping (which would desync that
// leg's hip/knee from the paired relationship the formula assumes and
// leave it sitting higher than commanded, not actually reaching
// heightMM), the whole command is rejected and nothing moves, rather
// than silently producing an uneven, non-level stance.
float lastCommandedHeight = LEG_THIGH_MM + LEG_CALF_MM; // full extension -- references the constants directly so this can never drift out of sync with them again

bool setBodyHeight(float heightMM) {
  int hipFL, kneeFL, hipFR, kneeFR, hipRL, kneeRL, hipRR, kneeRR;
  bool ok = computeFrontJointsForHeight(FL, heightMM, hipFL, kneeFL) &&
            computeFrontJointsForHeight(FR, heightMM, hipFR, kneeFR) &&
            computeRearJointsForHeight(RL, heightMM, hipRL, kneeRL) &&
            computeRearJointsForHeight(RR, heightMM, hipRR, kneeRR);
  if (!ok) return false;

  lastCommandedHeight = heightMM;
  setHip(FL, hipFL);
  setKnee(FL, kneeFL);
  setHip(FR, hipFR);
  setKnee(FR, kneeFR);
  setHip(RL, hipRL);
  setKnee(RL, kneeRL);
  setHip(RR, hipRR);
  setKnee(RR, kneeRR);
  return true;
}

// Per-leg height correction (mm, relative to lastCommandedHeight)
// currently applied by updateBalance() below when self-balancing is
// enabled; stays at 0 (no effect) otherwise. Kept only for
// diagnostics -- updateBalance() recomputes it fresh every tick.
float legHeightCorrection[NUM_HIPS] = {0, 0, 0, 0};

// ============================================================
// STAND SEQUENCE (confirmed-low crouch <-> full standing)
// This is now the ONE mechanism behind every height/crouch/stand-type
// command -- replaces the old separate setCrouch() (its own +/-
// amount convention) and the "height <mm>" command (which routed
// through setBodyHeight()'s IK formulas and could reject or misbehave
// well before the hand-confirmed floor below, since those formulas
// have their own, tighter limits -- see computeFrontJointsForHeight()/
// computeRearJointsForHeight() above, still used internally by
// updateBalance() but no longer exposed as a user command).
//
// CROUCH_LOW_HIP/KNEE are the exact angles found by hand-jogging the
// robot to its lowest stance that still reads level (confirmed
// directly on hardware). standProgress (0 = that crouch, 1 = full
// standing at HIP_START/KNEE_START) blends every joint through the
// SAME single fraction, via plain per-joint linear interpolation
// between its own two confirmed endpoints -- one shared number
// convention for all eight joints, instead of each joint/function
// inventing its own sign/amount relationship.
//
// standStep() nudges that fraction toward standTargetProgress by
// STAND_STEP_FRACTION each call, in whichever direction is needed;
// updateStand() paces repeated calls on a timer so "stand <percent>"
// moves the robot gradually instead of snapping in one jump, and works
// the same way whether raising or lowering.
// ============================================================
const int CROUCH_LOW_HIP[NUM_HIPS]  = { 140, 140, 140, 140 }; // FL, FR, RL, RR -- confirmed by hand
const int CROUCH_LOW_KNEE[NUM_HIPS] = {  30,  20, 240, 250 }; // FL, FR, RL, RR -- confirmed by hand

// ------------------------------------------------------------
// ToF1 -> step geometry. ToF1 is mounted fixed to the chassis (not a
// leg), aimed level/forward -- confirmed by hand, not angled down. So
// the beam itself doesn't sweep the ground: as stand_sweep changes
// body height, ToF1's HEIGHT ABOVE THE GROUND changes while its beam
// stays horizontal, so it's the sensor's height sweeping through
// space, not its aim direction. Scanning crouch->stand raises the
// sensor; while it's below the step's top, the beam hits the step's
// front (riser) face at a roughly constant distance; the moment the
// sensor rises above the step's height, the beam clears the top edge
// and suddenly reads far (or nothing) -- that crossover is exactly
// the jump stand_sweep already detects.
//
// That means, at the jump:
//   step height  = (this leg's real hip-to-ground height at the
//                   jump's standProgress) + TOF1_HEIGHT_ABOVE_HIP_MM
//   step forward = (the last close reading before the jump) +
//                   TOF1_FORWARD_OFFSET_MM
// "Real hip-to-ground height at a given standProgress" doesn't need
// new calibration -- applyStandProgress() (right below) already
// interpolates hip/knee ANGLE directly between two confirmed endpoints
// (CROUCH_LOW and HIP_START/KNEE_START), so running that same
// interpolated angle through the same trig legForwardKinematics() uses
// gives the exact real height, not an approximation. That's also why
// this block sits here, before applyStandProgress(): it needs
// heightAtStandProgress() too, to keep lastCommandedHeight in sync.
//
// TOF1_HEIGHT_ABOVE_HIP_MM/TOF1_FORWARD_OFFSET_MM are rough hand
// measurements ("a few mm" / "~3mm") -- refine with calipers if a
// step attempt ends up consistently short/long by a small amount.
//
// UNVALIDATED: this is a hypothesis from the sensor's confirmed
// mounting, not yet confirmed against a real known step -- the
// toolbox test that motivated lowering STEP_CHANGE_THRESHOLD_MM never
// produced a jump at all, so treat the first few estimates as
// something to sanity-check by eye/tape measure, not trust blindly.
// ------------------------------------------------------------
#define TOF1_HEIGHT_ABOVE_HIP_MM 5.0 // measured: "a few mm" above the hip-pivot line
#define TOF1_FORWARD_OFFSET_MM   -58.0 // measured with calipers: 58mm BEHIND the front hip pivots (corrected from an earlier "~3mm forward" estimate, then a hand-measured "~60mm")
#define TOF1_HEIGHT_REF_LEG      FL  // any leg works (all move identically during the sweep); front leg chosen since ToF1 sits at the front

// Real hip-to-ground height (mm) leg i would have at a given
// standProgress, using the exact CROUCH_LOW<->HIP_START/KNEE_START
// angle interpolation applyStandProgress() itself commands -- not an
// approximation, since that interpolation IS what's actually driving
// the servos during the sweep.
float heightAtStandProgress(int i, float progress) {
  float hip  = CROUCH_LOW_HIP[i]  + (HIP_START[i]  - CROUCH_LOW_HIP[i])  * progress;
  float knee = CROUCH_LOW_KNEE[i] + (KNEE_START[i] - CROUCH_LOW_KNEE[i]) * progress;
  float theta1 = radians(hip - HIP_START[i]);
  float theta2 = radians(knee - KNEE_START[i]);
  return LEG_THIGH_MM * cos(theta1) + LEG_CALF_MM * cos(theta1 + theta2);
}

// Same interpolation as heightAtStandProgress(), but the x (forward)
// component instead of y -- i.e. leg i's foot position relative to
// its own hip, fore-aft, at a given standProgress.
//
// Since a planted foot doesn't move once on the ground (same
// assumption the weight-shift/support-polygon code above already
// relies on), a CHANGE in this value between two standProgress values
// means the hip -- and the whole rigid chassis, including ToF1 -- has
// physically translated by that same amount to compensate, not just
// changed height. stand_sweep's scan always starts at 0% (see
// startStepScan()), so the chassis's net shift since scan start at
// any later progress p is footXAtStandProgress(leg, 0) -
// footXAtStandProgress(leg, p) -- confirmed on hardware to be real
// (not hypothetical): at the crossing point in one test run (41%)
// this works out to ~17mm, a real but secondary contributor compared
// to the ToF1 mounting-offset correction.
float footXAtStandProgress(int i, float progress) {
  float hip  = CROUCH_LOW_HIP[i]  + (HIP_START[i]  - CROUCH_LOW_HIP[i])  * progress;
  float knee = CROUCH_LOW_KNEE[i] + (KNEE_START[i] - CROUCH_LOW_KNEE[i]) * progress;
  float theta1 = radians(hip - HIP_START[i]);
  float theta2 = radians(knee - KNEE_START[i]);
  return LEG_THIGH_MM * sin(theta1) + LEG_CALF_MM * sin(theta1 + theta2);
}

float standProgress = 0.0; // 0 = CROUCH_LOW stance, 1 = full standing

void applyStandProgress(float progress) {
  progress = constrain(progress, 0.0, 1.0);
  standProgress = progress;
  for (int i = 0; i < NUM_HIPS; i++) {
    int hip  = (int)round(CROUCH_LOW_HIP[i]  + (HIP_START[i]  - CROUCH_LOW_HIP[i])  * progress);
    int knee = (int)round(CROUCH_LOW_KNEE[i] + (KNEE_START[i] - CROUCH_LOW_KNEE[i]) * progress);
    setHip(i, hip);
    setKnee(i, knee);
  }
  // Keep lastCommandedHeight (the height-based systems' reference --
  // updateBalance(), step-placement's target-Y math) in sync whenever
  // THIS system moves the body, not just when the older setBodyHeight()
  // does. Without this, reaching a stance via "stand"/stand_sweep left
  // lastCommandedHeight stuck at its stale default/last setBodyHeight()
  // value, silently feeding the wrong reference height into anything
  // that assumed it tracked the robot's actual current stance.
  lastCommandedHeight = heightAtStandProgress(TOF1_HEIGHT_REF_LEG, progress);
}

// Goes directly to the confirmed-low crouch -- this is what setup()
// boots into, since it's the hand-verified safe floor rather than a
// theoretical one.
void enterCrouchLow() {
  applyStandProgress(0.0);
}

#define STAND_STEP_FRACTION 0.05 // ~20 steps end to end

float standTargetProgress = 0.0;

// Advances one step toward standTargetProgress, up or down. Returns
// false once close enough that it snaps straight to the target --
// caller polls this repeatedly (see updateStand()) to animate the
// move gradually rather than in one jump.
bool standStep() {
  float delta = standTargetProgress - standProgress;
  if (fabs(delta) < STAND_STEP_FRACTION) {
    applyStandProgress(standTargetProgress);
    return false;
  }
  applyStandProgress(standProgress + (delta > 0 ? STAND_STEP_FRACTION : -STAND_STEP_FRACTION));
  return true;
}

bool standMoveInProgress = false;
#define STAND_STEP_INTERVAL_MS 300 // time between steps -- gives each eased servo move time to mostly finish before the next nudge
unsigned long lastStandStepMs = 0;

// Steps the stand sequence forward -- call every loop() pass.
void updateStand() {
  if (!standMoveInProgress) return;
  if (millis() - lastStandStepMs < STAND_STEP_INTERVAL_MS) return;
  lastStandStepMs = millis();
  if (!standStep()) {
    standMoveInProgress = false;
    Serial.println(F("Stand target reached."));
  }
}

// Starts gradually moving toward targetProgress (0 = CROUCH_LOW, 1 =
// full standing, anything between is a linear blend of the two).
// Returns false if a move is already running or it's already there.
bool startStandMove(float targetProgress) {
  targetProgress = constrain(targetProgress, 0.0, 1.0);
  if (standMoveInProgress || fabs(targetProgress - standProgress) < STAND_STEP_FRACTION) return false;
  standTargetProgress = targetProgress;
  standMoveInProgress = true;
  lastStandStepMs = millis();
  return true;
}

// ============================================================
// STAND SWEEP (continuous scan: report ToF1 only when it jumps)
// Earlier version stopped at 5 checkpoints (0/25/50/75/100%) and
// printed every one -- confirmed pitch/roll stay level across the
// whole range (so that question is settled), but a step edge showed
// up as a smooth-looking climb across those coarse stops rather than
// a clean jump, because 5 stops 25% apart is too coarse and each one
// paused for 1.5s doing nothing.
//
// This version instead moves in FINE_STEP_FRACTION (1%) steps with no
// pause, taking a fresh ToF1 reading after every step (the sensor's
// own continuous-ranging cycle is ~100ms, matching
// FINE_STEP_INTERVAL_MS, so each step should have a genuinely new
// sample) -- smooth, continuous motion instead of coarse stop-and-go.
// Only prints when ToF1 changes by more than STEP_CHANGE_THRESHOLD_MM
// from the previous step's reading, or when it flips between reading
// a target and reading none -- i.e. only at a real discontinuity, not
// every sample. Each print includes the stand percentage at that
// moment as the "height" -- there's no mm calibration yet (that's a
// later piece), but percentage is enough to locate roughly where a
// jump happens.
// ============================================================
#define FINE_STEP_FRACTION 0.01 // 1% per step -- fine enough to localize a jump, unlike the old 25%-apart checkpoints
#define FINE_STEP_INTERVAL_MS 100 // matches the ToF's own ~100ms continuous-ranging cycle
#define STEP_CHANGE_THRESHOLD_MM 150 // ToF1 delta between consecutive steps worth reporting -- >=150mm is treated as a step edge

#define SCAN_REPORT_STEP_PERCENT 10 // heartbeat trace interval -- see note below

// See the "ToF1 -> step geometry" block above applyStandProgress()
// (TOF1_HEIGHT_ABOVE_HIP_MM/TOF1_FORWARD_OFFSET_MM/TOF1_HEIGHT_REF_LEG,
// heightAtStandProgress()) -- moved there since applyStandProgress()
// itself needs it too (to keep lastCommandedHeight in sync), and
// #define/enum/global declarations, unlike functions, aren't
// auto-prototyped by the Arduino builder -- they have to physically
// precede their first use in the file.

float lastDetectedStepForwardMM = 0, lastDetectedStepHeightMM = 0;
bool  lastDetectedStepValid = false;

enum ScanState { SCAN_IDLE, SCAN_TO_START, SCAN_STEPPING };
ScanState scanState = SCAN_IDLE;
unsigned long lastScanStepMs = 0;
int nextScanReportPercent = 0;

// Detection is against the scan's STARTING baseline, not the previous
// step's reading -- a real step/lip clears in one smooth climb over
// many small steps (confirmed on hardware: a real ~150mm step showed
// a gradual 461->941mm climb, never a single-step jump), so comparing
// only to the immediately preceding reading never crosses
// STEP_CHANGE_THRESHOLD_MM at all. Comparing cumulative drift from the
// baseline instead catches exactly the point where a lip below 150mm
// tall has been cleared. Latched to report ONCE per scan -- further
// drift past that point (e.g. seeing progressively farther down a
// staircase/hallway beyond the lip) is background, not a second step,
// and re-reporting it would just be noise.
uint16_t scanBaselineToF1 = 0;
bool scanBaselineToF1Ok = false;
bool scanStepReported = false;

// ------------------------------------------------------------
// AUTO-STEP: once stand_sweep finds a crossing (and it passes the
// reachability sanity check in printScanChange()), automatically
// attempt to place a foot on it -- no manual step_scan_* needed. Runs
// as its own small state machine (checked every loop() pass, like the
// scan/lift sequences) rather than blocking, since everything here is
// async servo motion.
//
// Sequence: stop the scan the instant a crossing is found (no reason
// to keep scanning once the answer's known) -> startPlaceOnStep()
// directly from whatever stance the sweep happened to stop at.
// Deliberately NOT a forced return to 100%/full stand first: at exactly
// full leg extension (thigh+calf == max reach) there is zero slack for
// the weight-shift's small per-leg nudge, so startLiftSequence() fails
// every time from there regardless of the step estimate (confirmed on
// hardware) -- lastCommandedHeight is kept in sync with wherever the
// sweep stopped (see applyStandProgress()), so the reach/support-
// polygon math is still correct without moving first.
//
// If the placement sequence aborts for any reason (unreachable
// target, support triangle not achieved), this just reports it and
// goes idle -- it does NOT retry or attempt a different leg.
//
// UNTESTED ON HARDWARE, same as the rest of the step-placement path --
// this removes the last manual checkpoint before a real attempt, so
// watch closely on the first several runs.
// ------------------------------------------------------------
#define AUTO_STEP_LEG FL // which leg attempts the placement -- front leg, matches TOF1_HEIGHT_REF_LEG

enum AutoStepState { AUTO_IDLE, AUTO_PLACING };
AutoStepState autoStepState = AUTO_IDLE;
// updateAutoStep() itself is defined further down, after LiftState/
// liftState (its enum/global declarations, not just function calls,
// need to physically precede it) -- see the LEG LIFT / STEP-PLACEMENT
// SEQUENCE section below.

// Starts the scan: goes to 0% first (if not already there), then
// steps up to 100% in fine increments. Returns false if a scan or a
// manual stand move is already in progress.
bool startStepScan() {
  if (scanState != SCAN_IDLE || standMoveInProgress) return false;
  scanState = SCAN_TO_START;
  startStandMove(0.0); // no-op if already at 0%; SCAN_TO_START handles either case
  return true;
}

// Called once, the first time cumulative drift from the scan's
// baseline crosses STEP_CHANGE_THRESHOLD_MM -- see scanBaselineToF1's
// comment for why baseline (not the previous step) is what's compared.
void printScanChange() {
  Serial.print(F("ToF1 cleared lip at ")); Serial.print((int)round(standProgress * 100)); Serial.print(F("%: baseline="));
  if (scanBaselineToF1Ok) { Serial.print(scanBaselineToF1); Serial.print(F("mm")); } else { Serial.print(F("---")); }
  Serial.print(F(" -> now="));
  if (tof1_ok) { Serial.print(tof1_mm); Serial.print(F("mm")); } else { Serial.print(F("---")); }
  Serial.print(F("  (possible step, cumulative delta >= ")); Serial.print(STEP_CHANGE_THRESHOLD_MM); Serial.print(F("mm)"));
  Serial.println();

  // Use the scan's BASELINE (0%) for distance, not lastScanToF1 (the
  // reading immediately before the flagged crossing). Confirmed on
  // hardware against a real, tape-measured box: readings stay
  // accurate to within a few mm through most of the climb (e.g. 20%
  // and 30% both landed within 5mm of the true distance), but by the
  // time the cumulative-drift threshold actually trips, the beam has
  // already been partway into the transition for several percent --
  // lastScanToF1 at that point can be 100mm+ off, already contaminated
  // by whatever's behind the step. The baseline is captured before any
  // climbing has started by definition, so it's the reading least
  // likely to already be mid-transition.
  if (scanBaselineToF1Ok) {
    float stepHeightMM  = heightAtStandProgress(TOF1_HEIGHT_REF_LEG, standProgress) + TOF1_HEIGHT_ABOVE_HIP_MM;
    float stepForwardMM = (float)scanBaselineToF1 + TOF1_FORWARD_OFFSET_MM;
    Serial.print(F("  -> estimated step: height~")); Serial.print(stepHeightMM, 0);
    Serial.print(F("mm at ~")); Serial.print(stepForwardMM, 0);
    Serial.println(F("mm forward of the hip. UNVALIDATED estimate -- sanity-check before trusting step_scan_*."));

    // Hard physical ceiling -- thigh+calf is the leg's absolute max
    // reach, full stop, regardless of body height. A forward
    // estimate anywhere near that (a 20mm margin here) can't ever be
    // placed on, so don't let auto-step/step_scan_* attempt it and
    // fail confusingly -- flag it as what it is: the distance estimate
    // itself is bad for this scan, most likely because the reading
    // never actually plateaued (a real flat step face gives a roughly
    // CONSTANT reading right up to the jump; a continuously climbing
    // reading -- confirmed on hardware even against a single isolated
    // box, not just a staircase -- means "last reading before the
    // threshold trips" isn't measuring a stable face distance at all).
    if (stepForwardMM > (LEG_THIGH_MM + LEG_CALF_MM) - 20.0) {
      Serial.print(F("  -> REJECTED: "));
      Serial.print(stepForwardMM, 0);
      Serial.print(F("mm exceeds this leg's max possible reach (")); Serial.print(LEG_THIGH_MM + LEG_CALF_MM, 0);
      Serial.println(F("mm) -- not usable, not stored. Distance estimate is unreliable for this scan, not just this leg's workspace."));
    } else {
      lastDetectedStepForwardMM = stepForwardMM;
      lastDetectedStepHeightMM  = stepHeightMM;
      lastDetectedStepValid = true;
    }
  }
}

// Steps the scan forward -- call every loop() pass.
void updateStepScan() {
  if (scanState == SCAN_IDLE) return;

  if (scanState == SCAN_TO_START) {
    if (standMoveInProgress) return; // still moving to 0%
    scanBaselineToF1 = tof1_mm;
    scanBaselineToF1Ok = tof1_ok;
    scanStepReported = false;
    lastDetectedStepValid = false; // don't let a rejected/absent result this run reuse a stale prior scan's estimate
    lastScanStepMs = millis();
    nextScanReportPercent = 0;
    scanState = SCAN_STEPPING;
    return;
  }

  // SCAN_STEPPING
  if (millis() - lastScanStepMs < FINE_STEP_INTERVAL_MS) return;
  lastScanStepMs = millis();

  if (!scanStepReported) {
    // Self-motion compensation: the scan always starts at 0% (see
    // startStepScan()), so the chassis (and ToF1 with it) has shifted
    // footXAtStandProgress(leg, 0) - footXAtStandProgress(leg, standProgress)
    // since the baseline was captured -- confirmed real on hardware,
    // up to ~180mm across the full 0-100% range. Adding that back to
    // the raw reading reconstructs what it would read if the chassis
    // hadn't moved, isolating genuine external-object distance change
    // from self-motion before comparing against the baseline. This is
    // ONLY for the crossing comparison -- the final distance-to-hip
    // estimate in printScanChange() already uses TOF1_FORWARD_OFFSET_MM,
    // a fixed offset anchored to wherever the hip is RIGHT NOW, so it
    // doesn't need (or want) this same correction applied again.
    float chassisShiftMM = footXAtStandProgress(TOF1_HEIGHT_REF_LEG, 0.0)
                         - footXAtStandProgress(TOF1_HEIGHT_REF_LEG, standProgress);
    float compensatedToF1 = (float)tof1_mm + chassisShiftMM;

    bool crossed = false;
    if (scanBaselineToF1Ok) {
      if (!tof1_ok) {
        crossed = true; // target disappeared entirely -- cleared it with nothing behind
      } else if (compensatedToF1 >= (float)scanBaselineToF1 + STEP_CHANGE_THRESHOLD_MM) {
        crossed = true; // cumulative drift from baseline, with self-motion compensated out, crossed the threshold
      }
    }
    if (crossed) {
      printScanChange();
      scanStepReported = true;
      scanState = SCAN_IDLE; // found the crossing (or rejected the estimate) either way -- nothing left to scan for
      if (lastDetectedStepValid) {
        // startLiftSequence() (called via startPlaceOnStep()) now
        // raises to LIFT_STAND_TARGET_PROGRESS itself before shifting --
        // lastCommandedHeight is kept in sync with wherever the sweep
        // stopped (see applyStandProgress()), so the reach/support-
        // polygon math is still correct regardless of the stance this
        // was triggered from.
        Serial.println(F("Auto-attempting step placement..."));
        if (startPlaceOnStep(AUTO_STEP_LEG, lastDetectedStepForwardMM, lastDetectedStepHeightMM)) {
          autoStepState = AUTO_PLACING;
        } else {
          Serial.println(F("Auto step placement could not start -- a lift/step sequence is already in progress (liftState != LIFT_IDLE). If the last attempt ended in a hold or a safety abort, send 'lower' first -- manually jogging hip_xx/knee_xx does NOT reset this."));
        }
      }
      return;
    }
  }

  // Heartbeat trace every SCAN_REPORT_STEP_PERCENT -- a scan that never
  // crosses STEP_CHANGE_THRESHOLD_MM prints NOTHING otherwise (which is
  // exactly what happened testing against the toolbox at the original
  // 100mm threshold: no single 1% step ever jumped that much, so the
  // scan looked like it "didn't work" even though it ran correctly and
  // just never saw a discontinuity that sharp -- the threshold has
  // since been tuned to 150mm, a deliberately unambiguous jump size for
  // a real step test). This gives visible confirmation the scan is
  // actually running and collecting readings, and shows the real trend
  // even when no single step counts
  // as a "jump".
  int curPercent = (int)round(standProgress * 100);
  if (curPercent >= nextScanReportPercent) {
    Serial.print(curPercent); Serial.print(F("%: ToF1="));
    if (tof1_ok) {
      Serial.print(tof1_mm);
      float shift = footXAtStandProgress(TOF1_HEIGHT_REF_LEG, 0.0) - footXAtStandProgress(TOF1_HEIGHT_REF_LEG, standProgress);
      Serial.print(F("mm (self-motion compensated: ")); Serial.print((float)tof1_mm + shift, 0); Serial.println(F("mm)"));
    } else {
      Serial.println(F("---"));
    }
    nextScanReportPercent += SCAN_REPORT_STEP_PERCENT;
  }

  if (standProgress >= 1.0) {
    scanState = SCAN_IDLE;
    if (!scanStepReported) {
      Serial.println(F("Scan complete. No step/lip crossing found in this range."));
    } else {
      Serial.println(F("Scan complete."));
    }
    return;
  }
  applyStandProgress(standProgress + FINE_STEP_FRACTION);
}

// ============================================================
// BODY GEOMETRY (for balance/support-polygon calculations)
// Body frame: origin at the robot's geometric center, approximating
// the center of mass (assumed roughly centered -- the servos are the
// heaviest components and are distributed fairly evenly across the
// four corners). x = forward (+), y = left (+).
//
// Each leg's IK only moves in its own sagittal plane (no
// ab/adduction, confirmed earlier), so a planted foot's body-frame Y
// is always exactly its hip's fixed offset -- only X (fore-aft) can
// be adjusted to shift the robot's weight.
// ============================================================
// Longitudinal distance from the body's center to the front hips and
// to the rear hips, measured independently -- NOT assumed symmetric.
// Both were 157.5mm (half of a 315mm front-to-rear spacing) until the
// rear hips were moved closer to the front, to shrink how far a lifted
// front leg's swing can shift the CoM before it exits the remaining
// 3-leg support triangle (see the LIFT-SEQUENCE TILT SAFETY NET
// comment -- catching a fall in progress is a stopgap; keeping the CoM
// inside the triangle in the first place is the actual fix). Measure
// fresh after any chassis change -- REAR_HIP_X_MM is the one that
// changes here, FRONT_HIP_X_MM shouldn't need to.
const float FRONT_HIP_X_MM = 157.5;
const float REAR_HIP_X_MM  = 157.5; // TODO: update to the real measured value once the rear hips are physically relocated
const float BODY_HALF_WIDTH_MM  = 102.5; // half of 205mm left-to-right hip spacing
const float HIP_OFFSET_X[NUM_HIPS] = {  FRONT_HIP_X_MM,  FRONT_HIP_X_MM, -REAR_HIP_X_MM, -REAR_HIP_X_MM }; // FL,FR,RL,RR
const float HIP_OFFSET_Y[NUM_HIPS] = {  BODY_HALF_WIDTH_MM,  -BODY_HALF_WIDTH_MM,   BODY_HALF_WIDTH_MM,  -BODY_HALF_WIDTH_MM };

// ============================================================
// SELF-BALANCING (closed-loop pitch/roll correction via IK)
// Re-levels the body by computing, per leg, how much that leg's
// stance HEIGHT needs to change to cancel the measured tilt, rather
// than nudging a raw hip-angle trim as before. Keeping all four feet
// planted while the body tilts by pitch/roll means each leg's height
// must shift by (that leg's distance from the body center along the
// tilt axis) * tan(tilt angle) -- the same plane-projection already
// used by footBodyPosition()/HIP_OFFSET_X/Y for the support-polygon
// check, just applied to height instead of position. The corrected
// height is then run back through frontAmountForHeight()/
// rearAmountForHeight() -- the same IK setBodyHeight() itself uses --
// so each leg still moves along its own confirmed hip/knee bend
// direction instead of an arbitrary angle offset.
//
// Re-applied on a timer independent of whether a height change is
// actively in progress or already settled -- so a transition (e.g.
// crouched -> standing) gets corrected throughout the move, not just
// checked at the end. Only affects setBodyHeight()'s stance --
// crouch() is the simpler manual tool and isn't corrected here.
//
// UNTESTED: the correction directions are a best-reasoned guess from
// the reported convention (positive pitch = front leaning forward,
// positive roll = tilting right), not something verified on
// hardware. If tilt gets WORSE over time instead of settling toward
// zero, that means a term's sign needs flipping, not a redesign --
// disable with "balance off" immediately if that happens.
// ============================================================
bool balanceEnabled = false;

#define BALANCE_INTERVAL_MS 200
#define BALANCE_GAIN 0.5             // fraction of the full geometric correction applied per tick -- conservative starting point
#define BALANCE_MAX_CORRECTION_MM 30 // safety clamp on how far any single leg's height can be pulled from lastCommandedHeight

unsigned long lastBalanceMs = 0;

void updateBalance() {
  if (!balanceEnabled) return;
  if (millis() - lastBalanceMs < BALANCE_INTERVAL_MS) return;
  lastBalanceMs = millis();

  float pitch, roll;
  if (!readMPU6050(pitch, roll)) return; // bad read -- skip this cycle rather than correct against garbage
  float tanPitch = tan(radians(pitch));
  float tanRoll  = tan(radians(roll));

  for (int i = 0; i < NUM_HIPS; i++) {
    float correction = BALANCE_GAIN * (HIP_OFFSET_X[i] * tanPitch - HIP_OFFSET_Y[i] * tanRoll);
    correction = constrain(correction, -BALANCE_MAX_CORRECTION_MM, BALANCE_MAX_CORRECTION_MM);
    float correctedHeight = lastCommandedHeight + correction;

    int hip, knee;
    bool ok = (i == FL || i == FR) ? computeFrontJointsForHeight(i, correctedHeight, hip, knee)
                                    : computeRearJointsForHeight(i, correctedHeight, hip, knee);
    // If this leg's hip or knee would have to pass its real limit to
    // reach the corrected height, leave it at its last commanded
    // position this tick rather than clamp just one of the pair --
    // that's exactly the mismatch that made setBodyHeight() silently
    // produce a non-level stance (see its validation above).
    if (!ok) continue;

    legHeightCorrection[i] = correction;
    setHip(i, hip);
    setKnee(i, knee);
  }
}

// Forward kinematics for leg i (the inverse of solveLegIK's math):
// current foot position relative to its own hip pivot, derived from
// its current settled hip/knee angles.
void legForwardKinematics(int i, float &xOut, float &yOut) {
  float theta1 = radians((float)(hipPos[i] - HIP_START[i]));
  float theta2 = radians((float)(kneePos[i] - KNEE_START[i]));
  xOut = LEG_THIGH_MM * sin(theta1) + LEG_CALF_MM * sin(theta1 + theta2);
  yOut = LEG_THIGH_MM * cos(theta1) + LEG_CALF_MM * cos(theta1 + theta2);
}

// Leg i's foot position in the body frame (x forward+, y left+).
void footBodyPosition(int i, float &bx, float &by) {
  float lx, ly;
  legForwardKinematics(i, lx, ly);
  bx = HIP_OFFSET_X[i] + lx;
  by = HIP_OFFSET_Y[i]; // fixed -- no lateral leg motion
}

// Standard sign/point-in-triangle test.
float triSign(float px, float py, float ax, float ay, float bx, float by) {
  return (px - bx) * (ay - by) - (ax - bx) * (py - by);
}

bool pointInTriangle(float px, float py,
                      float ax, float ay, float bx, float by, float cx, float cy) {
  float d1 = triSign(px, py, ax, ay, bx, by);
  float d2 = triSign(px, py, bx, by, cx, cy);
  float d3 = triSign(px, py, cx, cy, ax, ay);
  bool hasNeg = (d1 < 0) || (d2 < 0) || (d3 < 0);
  bool hasPos = (d1 > 0) || (d2 > 0) || (d3 > 0);
  return !(hasNeg && hasPos);
}

// Shortest distance from (px,py) to the segment a-b.
float distToSegment(float px, float py, float ax, float ay, float bx, float by) {
  float dx = bx - ax, dy = by - ay;
  float len2 = dx * dx + dy * dy;
  float t = ((px - ax) * dx + (py - ay) * dy) / len2;
  t = constrain(t, 0.0, 1.0);
  float projx = ax + t * dx, projy = ay + t * dy;
  return sqrt((px - projx) * (px - projx) + (py - projy) * (py - projy));
}

// Minimum clearance from (px,py) to any of the triangle's three
// edges, or -1 if (px,py) isn't inside the triangle at all. A plain
// "is it inside" boolean (the old isStableOn()) treats a point sitting
// exactly on an edge the same as one sitting dead center -- this
// distinguishes the two, since the former has zero real-world margin
// for error and the latter has plenty.
float stabilityMargin(float px, float py, float ax, float ay, float bx, float by, float cx, float cy) {
  if (!pointInTriangle(px, py, ax, ay, bx, by, cx, cy)) return -1.0;
  float d1 = distToSegment(px, py, ax, ay, bx, by);
  float d2 = distToSegment(px, py, bx, by, cx, cy);
  float d3 = distToSegment(px, py, cx, cy, ax, ay);
  return min(d1, min(d2, d3));
}

#define MIN_STABILITY_MARGIN_MM 20.0 // reject a lift if even the best achievable weight-shift can't clear this

#define STABILITY_SHIFT_SEARCH_RANGE_MM 250.0
#define STABILITY_SHIFT_SEARCH_STEP_MM  2.0

// Pure distance check, same reachability test solveLegIK() itself
// applies -- used here to pre-screen candidate shifts so the one this
// picks is guaranteed to actually succeed when setFoot() is called for
// real, instead of finding out only after committing to it.
bool footReachable(float localX, float localY) {
  float d = sqrt(localX * localX + localY * localY);
  return d <= (LEG_THIGH_MM + LEG_CALF_MM) && d >= fabs(LEG_THIGH_MM - LEG_CALF_MM);
}

// Searches for the single fore-aft shift (applied identically to all
// three stance legs' body-frame X -- the only degree of freedom, since
// these legs can't move laterally) that MAXIMIZES the body center's
// worst-case clearance from the resulting support triangle's edges.
//
// This replaces simply centering the triangle's average (the previous
// approach): centering the average is not the same as maximizing the
// minimum edge clearance for an asymmetric triangle, and the gap is
// real, not theoretical -- for a real lift-FL case (the same one that
// caused a hardware tip-over), the old centroid shift left only ~29mm
// of margin on the tightest edge, while the shift found here achieves
// ~55mm from the exact same starting geometry, confirmed by hand.
//
// bx[]/by[] are the 3 stance legs' CURRENT body-frame positions
// (before any shift); lx[]/ly[] are the SAME three legs' CURRENT
// positions in their own leg-local (hip-relative) frame -- needed
// because the margin math and the reachability check operate in
// different frames. Without the reachability screen, this could (and
// on real hardware did) pick a shift that maximizes the support
// triangle's margin on paper while pushing one stance leg's foot
// target past its own physical reach -- "unreachable" at setFoot()
// time, aborting a lift the geometry said was fine. A brute-force
// sweep rather than a closed-form solve -- this only runs once per
// lift/step-placement start, not in the control loop, so the cost is
// negligible, and it doesn't depend on the worst-case-edge staying the
// same one throughout the search the way a more clever approach might
// assume. shift=0 (the legs' current, already-valid position) is
// always reachable, so this never comes up empty.
void findBestStabilityShift(float bx[3], float by[3], float lx[3], float ly[3], float &bestShiftOut, float &bestMarginOut) {
  bestShiftOut = 0;
  bestMarginOut = -1.0;
  for (float shift = -STABILITY_SHIFT_SEARCH_RANGE_MM; shift <= STABILITY_SHIFT_SEARCH_RANGE_MM; shift += STABILITY_SHIFT_SEARCH_STEP_MM) {
    if (!footReachable(lx[0] - shift, ly[0]) ||
        !footReachable(lx[1] - shift, ly[1]) ||
        !footReachable(lx[2] - shift, ly[2])) {
      continue; // this shift is geometrically nice but physically impossible for at least one stance leg
    }
    float margin = stabilityMargin(0, 0, bx[0] - shift, by[0], bx[1] - shift, by[1], bx[2] - shift, by[2]);
    if (margin > bestMarginOut) {
      bestMarginOut = margin;
      bestShiftOut = shift;
    }
  }
}

// ============================================================
// LEG LIFT / STEP-PLACEMENT SEQUENCE (all four legs)
// With all four feet at their neutral stance, the geometry above
// works out to a real finding: lifting one leg leaves the body's
// center sitting exactly on the boundary of the remaining 3-leg
// support triangle -- zero margin, not just "a bit tight". A weight
// shift before lifting isn't optional here.
//
// Since these legs can only move fore-aft (no side-to-side/
// ab-adduction capability), findBestStabilityShift() searches for the
// single fore-aft shift (applied to all three stance legs alike) that
// MAXIMIZES the body center's worst-case clearance from the resulting
// support triangle -- not simply centering the triangle's average,
// which leaves real margin on the table for an asymmetric triangle
// (confirmed on hardware: the naive centroid approach caused an actual
// tip-over lifting FL, front-right-down, rear-right wheel lifting off
// the ground). stabilityMargin() gates the lift both before the shift
// (checking the best achievable margin) and after it settles (checking
// the real, settled pose) -- either check failing aborts with a
// message rather than proceeding on the shift alone.
//
// Sequence has two paths after the weight shift:
//   plain lift:   SHIFT -> TUCK (raise, x->0) -> HOLD -> (on "lower")
//                 UNTUCK (back to orig stance) -> restore stance -> IDLE
//   step place:   SHIFT -> TUCK (raise, x->0) -> CLEAR (move forward to
//                 the step's x while STAYING elevated above the step's
//                 own height, not just above the ground) -> REACH
//                 (vertical-only descent onto the step, now that the
//                 leading edge is already behind the foot) -> HOLD ->
//                 (on "lower") RISE (vertical-only ascent back to the
//                 same clear height) -> RETRACT (move back to x=0
//                 while still elevated) -> UNTUCK (descend to orig
//                 stance) -> restore stance -> IDLE
// The TUCK phase (foot pulled to directly under the hip while raised)
// is deliberate: the leg should stay as close to the body as possible
// while airborne and unsupported, rather than swinging forward first
// and only then lifting.
//
// The CLEAR/RISE phases exist because hip and knee each ease
// independently in angle-space between two setFoot() targets -- NOT
// along a straight Cartesian line -- so a single move straight from
// tucked-and-raised to the step target could dip the foot below the
// step's height while still short of it horizontally and clip the
// step's front face. Splitting horizontal and vertical motion into
// separate moves (move forward while elevated, THEN descend; ascend,
// THEN move back while elevated) keeps the foot provably above the
// step's height for the entire horizontal traverse.
//
// Step target Y uses the same body-height convention as everywhere
// else in this file (depth below the hip) -- a TALLER step needs a
// SMALLER y (it's closer to the hip), computed as
// lastCommandedHeight - stepHeightMM. See computeFrontJointsForHeight()
// above for the same relationship applied to whole-body height.
//
// UNTESTED ON HARDWARE. Watch closely and be ready to catch/support
// the robot the first several times this runs.
// ============================================================
#define LEG_LIFT_MM 30.0        // conservative -- thighs should not fully lift yet
#define STEP_CLEAR_MARGIN_MM 20.0 // extra clearance above the step's own top surface during the horizontal traverse

// How far PAST the step's measured edge the wheel needs to land to be
// firmly planted, not just barely touching. Was tried once before as
// an amount ADDED to the reach target after driving -- reverted after
// that made the approach-drive stop further away to "leave room" for
// the addition, which just moved the imprecision around rather than
// fixing anything. This time it's subtracted from the APPROACH-DRIVE's
// stopping point instead (see LIFT_TUCK): the wheels drive this much
// CLOSER than bare minimum reach requires, so that reaching to the
// leg's fixed near-max target (LIFT_APPROACH_REACH_MARGIN_MM below)
// overshoots the step's real edge by this amount, instead of just
// reaching up to it. Confirmed on hardware this was the real problem:
// the wheel got there, barely touched the edge, and slipped off under
// its own weight-shift mid-descend -- the resulting sudden roll (~21
// degrees) tripped the tilt-abort net.
//
// Raised 40->70: confirmed on hardware that 40 (about one wheel
// radius) got the CONTACT POINT past the edge but not the wheel's
// axle/center -- it landed right at the front corner, not solidly on
// the flat surface. 70 is deliberately more than a full wheel
// diameter (~90mm) short of that, so the axle itself ends up
// meaningfully past the edge, not just the wheel's leading point.
//
// Brought back down 70->40 by explicit request: 40mm of wheel on the
// step is the target depth, not a step along the way to something
// bigger -- keeps the approach-drive less aggressive (less distance
// to close blind) while still meaningfully past the edge. This does
// NOT reopen the "barely touching the corner" problem 40 caused
// before that prompted the 70 raise -- that was 40mm of OVERSHOOT
// PAST THE EDGE with the earlier (buggy) approach-drive math; the
// approach-drive itself has had real fixes since (ToF-invalid-stop,
// liftIsSecondLeg reset, the createStablePlatform() revert) that
// weren't in place when 40 first failed.
//
// Lowered 40->25 by explicit request (a 2-3cm placement), alongside
// the broader redesign that measures and positions the chassis BEFORE
// the leg ever lifts (see LIFT_REMEASURE_DOWN) instead of during the
// lift -- this moves below the previously hardware-confirmed 40mm, so
// watch the next several placements closely for the "barely touching
// the corner" symptom 40mm itself was raised twice to fix.
#define STEP_LANDING_DEPTH_MM 25.0

// The final descent onto the step used to be one commanded move
// straight to the nominal target Y (lastCommandedHeight -
// liftStepHeightMM). Confirmed on hardware that this tips the robot:
// once the wheel actually contacts the step's rigid surface, the foot
// physically cannot move any lower, so any remaining commanded
// descent (from a slightly-off height estimate, or just modeling
// slop) doesn't move the foot at all -- it torques the CHASSIS
// upward/sideways instead, since that's the only thing left free to
// move. The leg would visibly land on the step fine and then the
// robot kept "lowering" it, pulling itself off balance. Descending in
// small increments and checking the IMU after each one catches
// contact (tilt moving off its pre-descent baseline) as soon as it
// happens, instead of only noticing after the chassis has already
// rolled -- a small tilt shift right after contact is expected (a
// slight intentional push down, not a fault) but growing tilt means
// stop now.
//
// STEPS raised 6->12 and the tilt trigger tightened 4.0->2.0: confirmed
// on hardware that 6 coarse steps let each increment press in further
// than intended before the check fired, visibly lifting/pitching the
// chassis more than "a little pressure" -- finer steps mean less
// overshoot per increment, and the tighter threshold stops closer to
// first contact instead of letting the tilt build up first. This
// still only holds the CHASSIS closer to level, not perfectly flat --
// with FL resting higher than the other three feet, some pitch is
// physically unavoidable unless the stance legs also rise to
// compensate, which isn't implemented.
//
// Tilt trigger loosened back 2.0->4.0: with LIFT_APPROACH now driving
// the leg to a near-straight (margin=0) target instead of a tucked
// one, the descent covers a much bigger net motion than when 2.0 was
// tuned -- confirmed on hardware firing well before the leg reached
// anywhere near its target angle, freezing it still visibly bent every
// time ("stopped early: contact detected via tilt" on every logged
// run). This re-opens the ORIGINAL overshoot risk the 4.0->2.0 change
// was fixing (some extra chassis pitch/torque after genuine contact,
// before the next check fires) -- accepted for now since a leg that
// never finishes extending is the worse failure. Watch the first
// several placements for that overshoot returning; if so, the real
// fix is re-verifying liftStepHeightMM (still the original, never
// re-measured scan estimate) rather than re-tightening this.
#define LIFT_DESCEND_STEPS 12
#define LIFT_CONTACT_TILT_DELTA_DEG 4.0

// Every lift/step-placement now raises to a NEAR-full stand first
// (not exactly 1.0 -- that's the same zero-slack extreme that broke
// the weight-shift originally, since thigh+calf == max reach exactly
// at 100%). Confirmed by hand: standing taller shrinks how much the
// swinging leg has to additionally extend to reach a given step
// (targetY = bodyHeight - stepHeight gets smaller as bodyHeight grows
// toward the step's own height), keeping its mass on a shorter lever
// arm throughout TUCK/CLEAR/REACH -- less leverage to tip the body,
// on top of the extra weight-shift slack a taller stance leaves.
#define LIFT_STAND_TARGET_PROGRESS 0.9

// After the stance-leg shift settles (servos report done), hold there
// for this long BEFORE checking stability and committing to the lift --
// a servo reporting "done" only means it reached its commanded angle,
// not that the chassis has stopped physically settling (residual
// bounce/backlash from the shift). Checking the instant the servos
// stop can pass a stance that hasn't actually finished moving yet.
#define LIFT_SETTLE_DWELL_MS 3000

// Deliberate pause between createNewStablePlatform() settling and the
// leg actually starting to lift (LIFT_PRE_LIFT_PAUSE) -- requested
// directly, not yet hardware-tuned. Gives the deliberately-tilted
// stance a moment to fully settle before committing to the lift.
#define LIFT_PRE_LIFT_PAUSE_MS 2000

// Small forward nudge for FL's OWN step-place specifically (not
// second_fr, which never goes through createNewStablePlatform()/
// LIFT_REVERSE at all). Confirmed on hardware: switching the stance
// legs from the sunk measuring pose back to createNewStablePlatform()
// pulls the chassis away from the step a little as a side effect of
// the leg-angle change alone -- no wheels move during that switch, but
// the geometry shift still shows up as real lost ground. Requested
// directly: nudge forward once the leg is safely lifted and clear
// (see LIFT_FL_NUDGE_START/WAIT), compensating before the reach math
// (LIFT_FR_RISE) runs. Bumped 300ms -> 450ms by request ("a little
// more") -- still a starting guess, not yet hardware-tuned to the
// actual pull-away distance.
#define LIFT_FL_NUDGE_MS 450

// Before lifting/tucking the leg for a step-place, reverse away from
// the step by this much extra clearance (using the wheels, not the
// legs) -- creates room for the knee-safe/hip-lift swing to happen
// without the leg (or its wheel) clipping the step's face, instead of
// relying on how tightly the knee folds (confirmed on hardware to
// bump the step regardless of fold tightness -- see LIFT_REVERSE).
// liftStepForwardMM is bumped by exactly this much at the same time
// (see LIFT_REMEASURE_DOWN), so the rest of the reach sequence just
// reaches further -- no separate distance math needed.
//
// This has to close the loop against ToF1 from the SUNK stance (in
// LIFT_REMEASURE_DOWN, once tof1_ok is already proven true there), not
// straight out of LIFT_SETTLING at the tall platform stance -- ToF1's
// beam has already cleared over the step's top at that height (see
// REMEASURE_LOWER_DEG below), so a reverse target computed from a
// reading taken there is reading empty space/whatever's beyond the
// step, not the step itself. Confirmed on hardware: this drove the
// full LIFT_REVERSE_TIMEOUT_MS backward (liftStepForwardMM jumped from
// ~279mm to 615mm) instead of stopping ~150mm out, and the resulting
// reach was aborted as unreachable.
//
// UNTESTED ON HARDWARE: this distance is a starting guess, not
// measured. Watch closely and be ready to catch/support the robot.
#define LIFT_REVERSE_CLEARANCE_MM 150.0
#define LIFT_REVERSE_SPEED        -150
#define LIFT_REVERSE_TIMEOUT_MS   8000

// LIFT_TUCK always closes the gap to (leg's max reach - this margin)
// on the wheels before reaching, rather than only when
// LIFT_REVERSE's clearance happens to push liftStepForwardMM out of
// range (confirmed on hardware doing exactly that: 353mm measured +
// 150mm clearance = 503mm requested, past the 360mm physical ceiling).
// This margin is the direct "how much bend is left in the final knee
// angle" knob -- smaller margin = closer to full extension = less bend
// = further reach onto the step (a more solid placement) = more
// clearance for the SECOND front leg's own lift/tuck/reach later (see
// second_fr), all from the same trade-off. Was 20, then 10; set to 0 by
// explicit request -- fully straighten the knee, target the leg's
// exact physical reach ceiling with no margin held back. At d ==
// thigh+calf exactly, solveLegIK()'s law-of-cosines solve still
// resolves cleanly (cosKnee = 1, knee angle = 0 = dead straight), so
// this isn't a math failure case -- but it does mean zero slack left
// for ToF noise, offset-calibration error, or the approach-drive's own
// real-world stopping imprecision (no encoders) before a reach that
// overshoots the ceiling by even a fraction of a mm gets rejected as
// unreachable. If that starts happening, this is the first constant to
// bring back up.
#define LIFT_APPROACH_REACH_MARGIN_MM 0.0
#define LIFT_APPROACH_SPEED           150
#define LIFT_APPROACH_TIMEOUT_MS      8000

// ESTIMATE, not a real calibration: linearly scaled from the actual
// measured WHEEL_MM_PER_MS_AT_120 (0.106mm/ms, see WheelCalibration.ino
// / the RAISE REAR section) by PWM ratio (150/120), since no real
// wheel-calibration run exists at LIFT_APPROACH_SPEED specifically. DC
// motor speed isn't perfectly linear with PWM (stall/deadband at low
// values, diminishing returns near max), so this is a starting guess,
// not a hardware-confirmed number -- if the approach drive under- or
// overshoots, a real WheelCalibration.ino-style run at speed 150 is
// what should replace it. Used by LIFT_TUCK's approach-drive (see
// that state) now that ToF1 can no longer see the step once the
// pre-lift stance change tilts the chassis away from it.
#define WHEEL_MM_PER_MS_AT_APPROACH 0.1325

// How far (degrees) to sink all four legs, right after the stable
// platform settles and before the actual lift begins, to re-measure
// the step's forward distance with a fresh, LIVE ToF1 reading instead
// of trusting the scan-derived (and now chassis-motion-corrected, but
// still model-based) estimate. The platform stance stands tall enough
// that ToF1's beam has likely already cleared over the step's top
// (the same "cleared the lip" effect the scan itself watches for), so
// it can't see the step's front face from up there -- sinking a few
// cm first should drop the beam back below the step's height. Front
// legs lower by hip+/knee- (frontAmountForHeight()'s cancelling
// convention); rear legs lower by knee- alone, since PRECLIMB_HIP_RL/RR
// are already at HIP_MIN (0) and can't go any lower. See
// LIFT_REMEASURE_DOWN.
#define REMEASURE_LOWER_DEG 15

// The whole lift/step-placement sequence moves at this fraction of
// normal servo speed (see moveSpeedScale) -- confirmed manually that a
// slow, careful, incremental approach is what actually gets a foot
// onto a step without tipping; this is the automated equivalent of
// that same care, not just the geometry alone. Applies to every phase
// (raise, shift, tuck, clear, reach, and the retract path back down),
// not just the reach itself, since the shift and raise are just as
// capable of upsetting balance if done abruptly.
#define LIFT_MOVE_SPEED_SCALE 0.3

// ============================================================
// PRE-CLIMB STANCE (FL lift only)
// Commanded as ABSOLUTE angles, not IK targets -- the analytic FK/IK
// model has now produced a physically implausible result (foot above
// its own hip) for THREE separate hand-verified rear-leg
// configurations in a row, so it's demonstrably unreliable at the
// large angles this stance actually needs (well past what the
// small-angle IK derivation assumes). Trusting the model to reproduce
// or verify this stance would just repeat that failure -- these are
// the exact angles confirmed by hand (real IMU: Level) to keep the
// chassis flat while FL lifts, matching CLIMB_PREP_MID below. RL/RR
// hip=0 is the key finding here -- confirmed across every low/mid/tall
// test, not just this one, so it replaces the earlier hip=90/80 values
// this stance used before that pattern was established.
// ============================================================
#define PRECLIMB_HIP_FL   92
#define PRECLIMB_KNEE_FL  100
#define PRECLIMB_HIP_RL   0
#define PRECLIMB_KNEE_RL  50
#define PRECLIMB_HIP_RR   0
#define PRECLIMB_KNEE_RR  55
// PRECLIMB_HIP_FR lowered 92->90 by request: FR (as the stance leg
// during FL's own reach, and as FR's own starting pose before
// second_fr/raise_rear) was sitting a little too close to the step,
// leaving too little clearance for FR's own later swing. Same
// forward-kinematics formula the FL-branch chassis-shift correction
// above already uses (LEG_THIGH_MM*sin(t1) + LEG_CALF_MM*sin(t1+t2)):
// at hip=92/knee=108 the foot sits ~271mm forward of the hip; at
// hip=90 (knee unchanged) that's ~264mm -- about 7mm back. Couples
// with a small height change too (about 3mm taller at that corner,
// same t1/t2 math applied to the cos() term) -- not purely isolated
// like the hip+/knee- cancelling trick elsewhere, but small enough
// not to matter here. UNVERIFIED against real hardware at this exact
// value -- if it needs to be more or less than ~7mm, this is the
// number to adjust.
#define PRECLIMB_HIP_FR   90
#define PRECLIMB_KNEE_FR  108
// NEW_STABLE_* (below, near commandNewStableLift()) is the separate,
// newer FR/RL/RR pose commanded just before driving forward over the
// step -- NOT folded into PRECLIMB_* itself, so the original verified
// climb-start stance (FL's own lift, second_fr, etc.) stays untouched.

// ============================================================
// SAFE-KNEE LIFT (FL lift only)
// Originally: before the hip starts lifting the leg, first move the
// knee to a safe, verified position on its own and let that settle --
// confirmed by hand (CLIMB_PREP_TALL/CLIMB_LIFT_TALL) that knee=270
// is a safe fold to hold the leg at before the hip does any large
// motion, rather than moving hip and knee together into unknown
// combined territory.
//
// Changed 270->0 by explicit request: 270 (toward KNEE_MAX) swings the
// calf/wheel TOWARD the step during lift-off -- the reason the old
// step-place sequence needed to reverse away from the step first, for
// clearance to fold safely. 0 (toward KNEE_MIN) folds the OTHER
// direction, up and toward the body instead, same idea already
// confirmed working for FR's second_fr case (see SECOND_FR_SAFE_KNEE).
// With the leg no longer swinging toward the step during lift-off, the
// reversing clearance step is no longer needed -- see
// LIFT_REMEASURE_DOWN, which now measures and positions the chassis
// BEFORE the leg lifts instead. This also means hip and knee now move
// SIMULTANEOUSLY for the step-place path (see LIFT_PRE_LIFT_PAUSE),
// not knee-first-then-hip -- same reasoning as second_fr's own fix:
// folding toward the body first, while the hip is still down, would
// dip the foot before the hip gets a chance to lift it clear. The
// plain lift_fl command (LIFT_KNEE_SAFE) still uses the original
// sequential knee-then-hip staging with this same (now-new-direction)
// value.
//
// UNTESTED at this new value -- the OLD 270 was hardware-confirmed
// (CLIMB_PREP_TALL/CLIMB_LIFT_TALL, real IMU: Level); this is a
// reasoned change based on the same logic that worked for
// SECOND_FR_SAFE_KNEE, not yet independently verified for FL. Watch
// closely.
//
// LIFT_LIFTED_HIP_FL=150 matches CLIMB_LIFT_TALL exactly -- the one
// combination hardware-confirmed (real IMU: Level) to pair a genuinely
// lifted-looking hip angle with this same safe knee, regardless of
// which prep pose the leg started from (confirmed at the OLD knee
// direction -- unverified whether 150 is still the right hip pairing
// for the new fold direction).
//
// Declared here (not next to where it's used, further down) because
// startLower() -- also further down, but textually BEFORE
// updateLiftSequence() -- needs these as plain #defines, and macros
// (unlike functions) aren't auto-prototyped by Arduino: they must
// textually precede their first use in the file.
// ============================================================
#define LIFT_SAFE_KNEE_FL   0
#define LIFT_LIFTED_HIP_FL  200 // raised 150->200 by request: FL's step-place now uses the same staged hip-peak/knee-extend/hip-descend technique as second_fr (see LIFT_PRE_LIFT_PAUSE and LIFT_FR_RISE/EXTEND/DESCEND) -- the plain lift_fl command (LIFT_KNEE_SAFE path) uses this same raised value too now, since they share the constant

// FR's counterparts for the SECOND-LEG-ONTO-STEP maneuver (see
// startSecondLegOntoStep() below) -- NOT hardware-verified the way
// LIFT_SAFE_KNEE_FL/LIFT_LIFTED_HIP_FL were (many rounds of real
// testing). These just mirror FL's values, on the assumption that
// FR's near-identical PRECLIMB pose (92deg/108deg vs FL's 92deg/100deg)
// means a near-mirror-symmetric leg -- this project has repeatedly
// found that assumption unreliable leg-to-leg (RL and RR don't even
// agree with each other), so treat this as an untested starting guess
// and watch closely / be ready to catch the robot the first time it
// runs, the same as any other UNTESTED ON HARDWARE pose in this file.
#define LIFT_SAFE_KNEE_FR   270
#define LIFT_LIFTED_HIP_FR  150

// Alternative safe-knee direction for FR specifically when lifting to
// place it onto the step with FL ALREADY there (startSecondLegOntoStep()
// below) -- requested directly: folding the knee UP toward
// LIFT_SAFE_KNEE_FR's 270 (KNEE_MAX[FR]) swings the calf/wheel forward,
// toward the step and toward FL's already-placed foot, right in the
// one situation where something is actually sitting there to contact.
// Folding the OTHER direction, toward KNEE_MIN[FR] instead, swings the
// calf/wheel up and back, away from both -- a genuinely safer path for
// this specific maneuver rather than a different amount of the same
// fold. Targets the far extreme (0) for the same reason
// LIFT_SAFE_KNEE_FR itself targets its own far extreme (270): the fold
// DIRECTION is the point, not a specific partial angle. Hip still
// lifts to the existing LIFT_LIFTED_HIP_FR target -- only the knee's
// fold direction changes here.
//
// UNTESTED ON HARDWARE, same as LIFT_SAFE_KNEE_FR/LIFT_LIFTED_HIP_FR
// already were for this maneuver -- if ground clearance ends up
// insufficient with this fold direction, LIFT_LIFTED_HIP_FR (the hip
// target, unchanged here) is the next thing to raise.
//
// NOT directly used by second_fr's rise anymore (see LIFT_FR_TURN1/
// LIFT_FR_CLEAR_RISE): that now computes an IK clear-height lift
// instead of targeting these fixed angles. Left defined for reference
// -- the fold-toward-body DIRECTION these represent is still the
// right idea, just no longer hardcoded as the specific target.
#define SECOND_FR_SAFE_KNEE 0

// Peak hip angle for second_fr's tuck (replaces LIFT_LIFTED_HIP_FR for
// this maneuver specifically -- LIFT_LIFTED_HIP_FR itself is untouched,
// still used by FR's plain lift/first-leg step place). Confirmed on
// hardware that 150 wasn't high enough: the knee's subsequent extend
// (see LIFT_FR_RISE/LIFT_FR_EXTEND below) was happening while the foot
// was still only at step-face height, so the extending foot pushed
// straight into the step's front face and shoved the robot off it,
// instead of swinging over the top. Raising this peak first, before
// the knee ever extends, is what gives the extend phase clearance
// above the step's height. Tune this higher if the push recurs.
//
// NOT directly used by second_fr's rise anymore -- see
// SECOND_FR_SAFE_KNEE's comment just above; same reasoning.
#define SECOND_FR_HIP_PEAK 200

// Pre-lift pivot pulse right before FR lifts (see LIFT_FR_TURN1 in
// startSecondLegOntoStep()), requested directly: FR+RR drive backward
// while FL+RL drive FORWARD at the same time. An earlier version left
// FL+RL at 0 (a straight one-sided translation instead of a pivot, so
// FR would get real clearance instead of just rotating in place) --
// reverted after that was confirmed on hardware to make the robot
// fall over during the pulse, even with active wheel braking added.
// Driving FL+RL forward is also the safe direction for FL specifically
// (it's already resting on the step at this point): forward presses
// it further onto the step rather than risking rolling it back off.
//
// Two earlier versions tried extra wheel motion right after lift-off
// (a restore-forward pulse) and right after placement (a correction
// wiggle) -- both removed after the wiggle was confirmed on hardware
// to walk a placed foot back off the step (spinning ALL four wheels,
// including whichever were resting on the step, doesn't discriminate).
//
// Now, by request, FR's placement (LIFT_FR_DESCEND, second_fr only --
// see LIFT_FR_POST_DRIVE/LIFT_FR_POST_TURN) DOES drive again after the
// foot is down, in two steps: first a plain forward drive of all four
// wheels (SECOND_FR_POST_DRIVE_SPEED/MS) to bring the chassis further
// over the step, THEN a full reverse pivot (FL+RL backward, FR+RR
// forward -- the exact opposite of the pre-lift pivot above, same
// speed/duration) to turn the chassis back and undo that rotation.
// The forward drive runs first specifically so the turn-back has more
// margin from the step edge to work with -- requested directly after
// an earlier version that turned back immediately reversed the robot
// right off the step. UNTESTED: the turn-back still drives FL
// backward while it's resting on the step, the same motion flagged as
// risky everywhere else in this file -- watch closely.
//
// Speed hardcoded to match SQUARE_TURN_SPEED's own value (defined
// later in the file, after this section, so can't be referenced by
// name here -- macros must textually precede their first use).
// Duration 1300ms by request.
#define SECOND_FR_TURN_SPEED      150
#define SECOND_FR_TURN_MS         1300

// Small straight reverse of ALL FOUR wheels right before FR starts
// lifting (see LIFT_FR_TURN1/LIFT_FR_PRE_REVERSE), requested directly
// after the pivot alone still let FR's wheel catch on the step edge --
// a pivot rotates but doesn't give FR much real backward translation,
// so this adds a genuine straight-line reverse on top of it. Explicitly
// includes FL+RL this time (unlike the pivot, which deliberately
// avoided driving FL): FL is already resting on the step at this
// point, so this does carry the same back-off-the-step risk flagged
// everywhere else in this file -- accepted anyway for the extra
// clearance. Kept small ("a tiny amount") to limit that exposure.
#define SECOND_FR_PRE_REVERSE_SPEED 150
#define SECOND_FR_PRE_REVERSE_MS    300

// Plain forward drive of all four wheels, right after FR's placement,
// before the post-placement turn-back above -- requested directly.
// Shortened 3000ms -> 200ms to specifically compensate for the new
// SECOND_FR_PRE_REVERSE_MS reverse above (same order of magnitude),
// rather than being a large separate "drive the chassis over the
// step" move.
#define SECOND_FR_POST_DRIVE_SPEED 100
#define SECOND_FR_POST_DRIVE_MS    200

// RL/RR swap applied ONLY at the moment second_fr starts (see
// startSecondLegOntoStep()'s FR branch below) -- by request, this
// compensation must NOT be active any earlier (e.g. during FL's own
// createNewStablePlatform() move), only once FR actually starts
// lifting. RR takes the more braced stance (formerly RL's 4/50) to
// compensate on that side once FR lifts. Defined here (not next to
// NEW_STABLE_* further down) because macros must textually precede
// their first use, and startSecondLegOntoStep() uses these above that
// point in the file.
#define SECOND_FR_HIP_RL    0
#define SECOND_FR_KNEE_RL   30
#define SECOND_FR_HIP_RR    4
#define SECOND_FR_KNEE_RR   50

// LIFT_FR_DESCEND's own contact-tilt threshold, separate from
// LIFT_CONTACT_TILT_DELTA_DEG -- confirmed on hardware that reusing
// the shared 4.0deg threshold false-triggered ("stopped early: contact
// detected" with no real contact) during this hip-ALONE descent. The
// shared threshold was tuned for the original combined hip+knee IK
// descent's tilt signature; a single-joint hip swing over a bigger
// travel (SECOND_FR_HIP_PEAK down to secondFrFinalHip) likely produces
// more harmless transient tilt from the leg's own mass alone, not
// contact. Loosened as a first adjustment -- LIFT_FR_DESCEND now logs
// the actual pitch/roll delta when it trips, so this can be tuned with
// real numbers instead of guessed again blindly.
#define SECOND_FR_DESCEND_TILT_DELTA_DEG 8.0

enum LiftState { LIFT_IDLE, LIFT_RAISING, LIFT_SHIFTING, LIFT_SETTLING, LIFT_REVERSE, LIFT_REMEASURE_DOWN, LIFT_REMEASURE_UP, LIFT_PRE_LIFT_PAUSE, LIFT_FL_NUDGE_START, LIFT_FL_NUDGE_WAIT, LIFT_KNEE_SAFE, LIFT_TUCK, LIFT_CLEAR, LIFT_DESCEND, LIFT_REACH, LIFT_HOLDING, LIFT_RISE, LIFT_UNTUCK, LIFT_LOWERING, LIFT_FR_RISE, LIFT_FR_EXTEND, LIFT_FR_DESCEND, LIFT_FR_TURN1, LIFT_FR_PRE_REVERSE, LIFT_FR_CLEAR_RISE, LIFT_FR_POST_TURN, LIFT_FR_POST_DRIVE };
LiftState liftState = LIFT_IDLE;
unsigned long liftSettleStartMs = 0;
unsigned long liftPreLiftPauseStartMs = 0; // LIFT_PRE_LIFT_PAUSE's dwell start -- see LIFT_PRE_LIFT_PAUSE_MS
int liftLegIdx = -1;
int liftStanceIdx[3];
float liftStanceX[3], liftStanceY[3]; // stance-leg foot positions before the shift, to restore on lower
float liftOrigX, liftOrigY;           // the lifted leg's own foot position before the shift, to restore on lower
bool  liftIsStepPlace = false;
bool  liftTiltAborted = false; // set by checkLiftTiltSafety() -- distinguishes a genuine LIFT_HOLDING from a safety freeze, since both land in the same state
bool  liftUsingVerifiedStance = false; // true when LIFT_RAISING used the hardcoded PRECLIMB_* angles instead of the computed IK shift -- LIFT_SHIFTING skips the geometric margin check in that case, since the FK model is known unreliable at these angles
bool  liftIsSecondLeg = false; // true when this sequence is startSecondLegOntoStep() placing a SECOND leg while the first stays put on the step -- LIFT_UNTUCK skips restoring liftStanceIdx (stale from the first leg's own sequence, and nothing else actually moved this time)
float liftStepForwardMM = 0, liftStepHeightMM = 0;

// LIFT_DESCEND's incremental-contact-check bookkeeping -- see
// LIFT_DESCEND_STEPS/LIFT_CONTACT_TILT_DELTA_DEG above.
float liftDescendStartY = 0, liftDescendEndY = 0;
int liftDescendStepIdx = 0;
float liftDescendBasePitch = 0, liftDescendBaseRoll = 0;
bool liftDescendStoppedEarly = false;

// The staged reach target, solved once in LIFT_FR_RISE and reused by
// LIFT_FR_EXTEND/LIFT_FR_DESCEND -- see those states' comments. Named
// for second_fr (where this was first built), but this whole
// RISE/EXTEND/DESCEND path is now leg-agnostic -- FL's own step-place
// (LIFT_PRE_LIFT_PAUSE) uses it too, not just second_fr. Reuses
// liftDescendStepIdx/liftDescendBasePitch/liftDescendBaseRoll/
// liftDescendStoppedEarly above for its own incremental contact-check
// in LIFT_FR_DESCEND (that bookkeeping is idle at the same time this
// path runs, since nothing using it goes through the old LIFT_CLEAR/
// LIFT_DESCEND states anymore).
int secondFrFinalHip = 0, secondFrFinalKnee = 0;

// The hip angle this specific staged sequence rose to before the knee
// extended -- LIFT_FR_DESCEND interpolates FROM this value (not a
// hardcoded one), since different entry points reach different peaks:
// second_fr's is whatever the IK-computed clear-height lift actually
// landed on (see LIFT_FR_CLEAR_RISE), FL's own step-place uses the
// fixed LIFT_LIFTED_HIP_FL.
int liftHipPeak = 0;

// Returns to idle from anywhere in the sequence (abort or success) --
// centralizing this so moveSpeedScale can never be left slow after
// the sequence ends, forgotten in one abort path but not another.
// Also stops the wheels -- LIFT_APPROACH can be mid-drive when this
// fires (a reachability abort right after it), and leaving the wheels
// rolling toward the step with a leg still up is exactly the case this
// whole function exists to prevent.
void abortLiftSequence() {
  liftState = LIFT_IDLE;
  liftLegIdx = -1;
  liftIsSecondLeg = false;
  moveSpeedScale = 1.0;
  stopWheels();
}

// The elevated Y used for the horizontal CLEAR/RETRACT traverses:
// whichever is more elevated (smaller y) of the plain ground-clearance
// tuck height, or the step's own top surface plus a safety margin --
// so the foot clears BOTH the ground and the step top, whichever is higher.
float computeClearY() {
  float groundClearY = liftOrigY - LEG_LIFT_MM;
  float stepClearY = lastCommandedHeight - liftStepHeightMM - STEP_CLEAR_MARGIN_MM;
  return min(groundClearY, stepClearY);
}

bool legMoveDone(int i) {
  unsigned long now = millis();
  bool hipDone  = (now - hipMoveStartMs[i]) >= hipMoveDurationMs[i];
  bool kneeDone = !kneeInstalled[i] || ((now - kneeMoveStartMs[i]) >= kneeMoveDurationMs[i]);
  return hipDone && kneeDone;
}

// Common bookkeeping shared by startLift() and startPlaceOnStep().
// Only records which leg is being lifted and kicks off the raise to
// LIFT_STAND_TARGET_PROGRESS -- the actual weight-shift computation
// happens once that settles (see the LIFT_RAISING case in
// updateLiftSequence()), since it needs the POST-raise foot positions,
// not whatever they were at the stance this was called from. Returns
// false without doing anything if a sequence is already in progress.
bool startLiftSequence(int legToLift) {
  if (liftState != LIFT_IDLE) return false;

  moveSpeedScale = LIFT_MOVE_SPEED_SCALE; // slow, careful motion for the whole sequence -- reset in abortLiftSequence()
  liftTiltAborted = false;
  liftUsingVerifiedStance = false;
  // Explicitly cleared here, not just relied on from abortLiftSequence()
  // -- confirmed on hardware that a stale true from a prior
  // startSecondLegOntoStep() call can otherwise survive into a fresh
  // FIRST-leg sequence (any exit path that doesn't happen to route
  // through abortLiftSequence() leaves it set), silently sending
  // LIFT_TUCK down the "second leg, no wheel movement" branch for a
  // leg that was never actually placed yet -- exactly the "never drove
  // forwards, straight to unreachable" failure reported.
  liftIsSecondLeg = false;
  liftLegIdx = legToLift;
  int n = 0;
  for (int i = 0; i < NUM_HIPS; i++) {
    if (i == legToLift) continue;
    liftStanceIdx[n] = i;
    n++;
  }

  // A genuinely NEW climb starts here -- reset raise_rear's front-leg
  // capture flag (see its declaration/comment) so the next raise_rear
  // call on this climb captures a fresh reference, instead of leaving
  // a stale one from some earlier climb.
  raiseRearFrontKneeCaptured = false;

  startStandMove(LIFT_STAND_TARGET_PROGRESS); // no-op if already there/close, or already moving
  liftState = LIFT_RAISING;
  return true;
}

// Starts a plain lift-and-hold on legToLift (any leg).
bool startLift(int legToLift) {
  liftIsStepPlace = false;
  return startLiftSequence(legToLift);
}

// Starts a lift, weight shift, and reach onto a step at
// (stepForwardMM, stepHeightMM) relative to the ground/hip, on
// legToLift (any leg).
bool startPlaceOnStep(int legToLift, float stepForwardMM, float stepHeightMM) {
  liftIsStepPlace = true;
  liftStepForwardMM = stepForwardMM;
  liftStepHeightMM = stepHeightMM;
  return startLiftSequence(legToLift);
}

// Starts retracting/lowering the currently-held leg (whether plain-
// lifted or placed on a step) and restoring the shifted stance legs.
//
// Same fix as the ascent: this used to retract via setFoot()/IK
// (rise to a computed clear height, then move back, then return to
// liftOrigX/liftOrigY) -- confirmed on hardware that this tilted the
// robot, with FL extending further than the other legs, because
// solveLegIK()'s continuity-based branch selection was starting from
// wherever REACH had just left FL (an already-extreme, IK-computed
// placement angle) and had no reason to produce anything sane from
// there -- the same model unreliability already established for large
// angles, just hit on the way down instead of the way up. Retraction
// now mirrors the ascent exactly, in reverse, with the same two
// verified absolute-angle waypoints: first back to the safe lifted
// pose (LIFT_LIFTED_HIP_FL/LIFT_SAFE_KNEE_FL), then back to the
// original stable-platform prep angle (PRECLIMB_HIP_FL/KNEE_FL) --
// no IK involved for FL at any point. Both the step-place and plain-
// lift cases now do the same thing: for a plain lift FL is already at
// the safe lifted pose, so that first move is a harmless no-op.
bool startLower() {
  if (liftState != LIFT_HOLDING) return false;
  setHip(liftLegIdx, (liftLegIdx == FR) ? LIFT_LIFTED_HIP_FR : LIFT_LIFTED_HIP_FL);
  setKnee(liftLegIdx, (liftLegIdx == FR) ? LIFT_SAFE_KNEE_FR : LIFT_SAFE_KNEE_FL);
  liftState = LIFT_RISE;
  return true;
}

// Places a SECOND leg onto the step while the first (liftLegIdx,
// already resting there in LIFT_HOLDING) stays exactly where it is --
// unlike startPlaceOnStep(), this does NOT raise/weight-shift/
// createStablePlatform() first, since all of that would disturb the
// first leg's already-placed position (createStablePlatform()
// specifically would snap it straight back down to its PRECLIMB
// angles). Requires the first leg to already be down-and-holding, not
// mid-sequence or frozen by a safety abort. liftStepForwardMM is left
// as whatever the first leg's (already live-re-measured) reach used --
// FL and FR share the same fore-aft hip position (HIP_OFFSET_X), so
// the same forward distance to the step applies; only their left/
// right offset differs, which doesn't affect forward reach at all.
//
// No verified stance exists yet for "one front leg already on the
// step, the other reaching up next to it" -- liftUsingVerifiedStance
// is set so the geometric margin check (which assumes a symmetric,
// all-feet-on-the-ground base) is skipped the same way it is for the
// first leg's own verified stance; the real IMU-based tilt safety net
// stays fully active as the actual backstop.
bool startSecondLegOntoStep(int legToLift) {
  if (liftState != LIFT_HOLDING || liftTiltAborted) return false;
  if (legToLift == liftLegIdx) return false;

  moveSpeedScale = LIFT_MOVE_SPEED_SCALE;
  liftIsStepPlace = true;
  liftIsSecondLeg = true;
  liftUsingVerifiedStance = true;
  liftLegIdx = legToLift;
  if (legToLift == FR) {
    // FR's rise starts with an IK-computed vertical lift (LIFT_FR_TURN1
    // -> LIFT_FR_CLEAR_RISE, see that state's comment) to a height
    // verified clear of both the step and the floor -- replacing an
    // earlier version that moved hip and knee together to fixed
    // SECOND_FR_HIP_PEAK/SECOND_FR_SAFE_KNEE angles, picked without
    // reference to the actual step geometry. That fixed-angle version
    // itself replaced sequential knee-then-hip staging (confirmed on
    // hardware: folding the knee down FIRST, while the hip was still
    // down at its step-place angle, dipped the foot/wheel toward the
    // ground before the hip ever got a chance to lift it clear --
    // "gripped the bottom of the step") -- moving hip and knee at the
    // same time is still the reason this doesn't go through the normal
    // LIFT_TUCK->LIFT_CLEAR->LIFT_DESCEND path.
    //
    // After the clear-height lift: LIFT_FR_RISE solves the real final
    // target, LIFT_FR_EXTEND reaches the knee forward alone (hip held
    // at the verified-clear height), LIFT_FR_DESCEND lowers the hip
    // alone onto the step (knee held) -- so the knee never extends
    // until the hip has already cleared, and the final descent never
    // moves the knee at all once it's already reaching over the step.
    //
    // Before any of that: RL/RR swap to SECOND_FR_HIP/KNEE_RL/RR,
    // applied right here (not any earlier -- e.g. not during FL's own
    // createNewStablePlatform() move) so RR takes the more braced
    // stance to compensate the instant FR starts lifting.
    setHip(RL, SECOND_FR_HIP_RL);   setKnee(RL, SECOND_FR_KNEE_RL);
    setHip(RR, SECOND_FR_HIP_RR);   setKnee(RR, SECOND_FR_KNEE_RR);

    // Pre-lift pivot pulse (see the comment on SECOND_FR_TURN_SPEED
    // above): FR+RR back up while FL+RL drive FORWARD at the same
    // time, requested directly -- a one-sided reverse (FL+RL just
    // braked, not driven) kept causing the robot to fall over during
    // this pulse, even with active braking. Driving FL+RL forward is
    // also the safe direction for FL specifically: it presses FL
    // further onto the step instead of risking rolling it back off.
    // LIFT_FR_TURN1 waits for this to finish, then starts a small
    // straight all-four-wheel reverse (LIFT_FR_PRE_REVERSE, see
    // SECOND_FR_PRE_REVERSE_SPEED/MS) before the lift actually starts.
    // Once FR is placed (LIFT_FR_DESCEND), it drives again, forward
    // THEN turn: LIFT_FR_POST_DRIVE (plain forward drive, compensating
    // for the pre-lift reverse and giving more margin from the edge
    // before turning) -> LIFT_FR_POST_TURN (full reverse pivot, undoing
    // the pre-lift pivot) -> LIFT_HOLDING. FL's own placement never
    // goes through any of these extra states -- see LIFT_FR_DESCEND's
    // liftIsSecondLeg check.
    if (!startTurnTestLR(SECOND_FR_TURN_SPEED, -SECOND_FR_TURN_SPEED, SECOND_FR_TURN_MS)) {
      Serial.println(F("second_fr aborted: could not start the pre-lift reverse (something else active)."));
      liftLegIdx = -1;
      liftIsSecondLeg = false;
      return false;
    }
    liftState = LIFT_FR_TURN1;
  } else {
    // FL (the other possible legToLift here, if a future second_fl is
    // added) keeps the original knee-first-then-hip sequencing --
    // nothing about FL placing next to an already-held FR has been
    // flagged as a problem.
    setKnee(liftLegIdx, LIFT_SAFE_KNEE_FL);
    liftState = LIFT_KNEE_SAFE;
  }
  return true;
}

// ============================================================
// LIFT-SEQUENCE TILT SAFETY NET
// The stability math above (stabilityMargin()/findBestStabilityShift())
// assumes the body's centre of mass stays fixed at the geometric
// centre -- it doesn't model the lifted leg's OWN mass swinging
// through TUCK/CLEAR/REACH, and that's not a small fraction: the legs
// carry the servos and wheel motors, the heaviest parts of the robot.
// Rather than try to precisely model that shift (which needs mass
// numbers we don't have yet), watch the real consequence instead: if
// the body actually starts tipping past a safe threshold, freeze every
// leg exactly where it is, right now, and drop into LIFT_HOLDING so
// the existing 'lower' recovery path takes over. This catches the CoM
// shift, uneven ground, or anything else that tips it, without needing
// to know leg mass at all.
// ============================================================
// Trip point -- LEVEL_TOLERANCE_DEG (3deg) just means "not level", this
// means "actually going wrong". Was 8.0, raised after a real hardware
// run tripped this at pitch=4.3/roll=10.1 during a step-place attempt
// on the verified pre-climb stance -- a stance already confirmed by
// hand to carry real load and resist a push -- while the confirmed
// genuine falls seen so far reached 35-41 degrees. Raised again from 15
// to 20 to give more headroom for the newer wheel-driven maneuvers
// (LIFT_REVERSE/LIFT_APPROACH), which are more dynamic than a plain
// leg-only reposition and more likely to produce real-but-harmless
// transient tilt. Still leaves 15-21 degrees of margin below the
// confirmed genuine falls, same "clears nuisance with room to spare,
// stays well below an actual topple" reasoning as before.
#define LIFT_TILT_ABORT_DEG 20.0
#define LIFT_TILT_CHECK_MS  50  // how often to poll the IMU while a sequence is active

// Disabled by explicit request: this net fired on pitch=24.3/roll=3.3
// mid-approach-drive with no adverse tilt visible on the robot, and
// the aborts (real and otherwise) were getting in the way more than
// they were helping. Left as a flag, not deleted -- this same net DID
// also catch two confirmed genuine near-falls this session (roll=31.8
// when FR's knee caught the step's lip; pitch=39.3/roll=21.3 when FL's
// marginal contact slipped), so flip this back to 1 to re-arm it if
// aborts stop being a nuisance and start being the only thing between
// the robot and a real fall again. The MPU6050 itself stays fully in
// use elsewhere for more targeted checks -- the one-shot pre-lift gate
// in LIFT_SETTLING and the per-step contact detection in LIFT_DESCEND
// are unaffected by this flag.
#define LIFT_REACTIVE_TILT_NET_ENABLED 0

// Tighter than LIFT_TILT_ABORT_DEG on purpose: this is the "are we
// actually stable enough to COMMIT to lifting a leg off the ground"
// check, not the "is it actively falling over" check -- want to catch
// a stance that's already leaning before removing one of its four
// points of contact, not just once it's clearly too late. Raised from
// 5 to 8 alongside LIFT_TILT_ABORT_DEG above, keeping the same ratio
// of "commit gate is meaningfully tighter than the active-abort net."
#define LIFT_PRELIFT_TILT_LIMIT_DEG 8.0

unsigned long lastLiftTiltCheckMs = 0;

// Halts a leg exactly where it is, mid-move or not -- re-issuing its
// own current angle as the move target makes updateServoMotion() settle
// on it almost immediately (duration collapses to MOVE_MIN_MS since the
// from/to angles are equal).
void freezeLeg(int i) {
  startHipMove(i, hipPos[i]);
  startKneeMove(i, kneePos[i]);
}

// Polls pitch/roll (throttled to LIFT_TILT_CHECK_MS) and, if either
// exceeds LIFT_TILT_ABORT_DEG, freezes all four legs in place and forces
// the sequence into LIFT_HOLDING. Returns true if it did so -- caller
// should skip its normal state-machine step for this tick when true.
bool checkLiftTiltSafety() {
  if (millis() - lastLiftTiltCheckMs < LIFT_TILT_CHECK_MS) return false;
  lastLiftTiltCheckMs = millis();

  float pitch, roll;
  if (!readMPU6050(pitch, roll)) return false; // bad read -- see readMPU6050()'s comment; don't abort on garbage, wait for the next poll
  if (fabs(pitch) < LIFT_TILT_ABORT_DEG && fabs(roll) < LIFT_TILT_ABORT_DEG) return false;

  Serial.print(F("LIFT SAFETY ABORT: body tilt pitch="));
  Serial.print(pitch, 1);
  Serial.print(F(" roll="));
  Serial.print(roll, 1);
  Serial.println(F(" exceeded the safety threshold -- freezing all legs where they are."));

  freezeLeg(liftLegIdx);
  for (int k = 0; k < 3; k++) freezeLeg(liftStanceIdx[k]);
  stopWheels(); // LIFT_APPROACH can be mid-drive when this trips -- don't keep rolling toward the step with a leg frozen mid-air
  liftTiltAborted = true;
  liftState = LIFT_HOLDING; // existing 'lower' recovery path takes over from here
  return true;
}

// ============================================================
// REAR KNEE-LURCH START -- picks up AFTER FL and FR are already
// placed on the step (via auto step placement + second_fr) and the
// chassis has driven forward over it: extending RL's knee alone (not
// a combined hip+knee move) deliberately lets the body's centre of
// mass lurch forward, rather than trying to keep the body perfectly
// controlled/balanced through the whole rear lift.
//
// Split into two explicit stages by request, rather than one combined
// jump straight to RL's already-extended target: REAR_LEGS_SHARED_START
// gets RL and RR to the SAME shared pose together first (both at RR's
// stance value, 50/140) -- wait for "Climb pose reached." (which only
// prints once ALL FOUR legs, including FL/FR, have finished settling)
// before doing anything else. Only THEN, as a separate deliberate
// step, does RL alone extend its knee toward 270 to start the lurch
// -- use knee_rl 270 directly (a single-joint jog is all this needs,
// no new command required).
// ============================================================
const ClimbPose REAR_LEGS_SHARED_START = { 65, 0, 65, 0, 50, 140, 50, 140 }; // both front legs up on the step; RL and RR at the SAME shared pose, not yet diverging

bool climbMoveActive = false;
unsigned long lastClimbTiltCheckMs = 0;

// Commands all four legs to a ClimbPose at once, duration-synced so
// they arrive together, at the same careful moveSpeedScale the lift
// sequence uses. Sets climbMoveActive so updateClimbMoveTracking()
// starts watching for tilt and for the move settling.
void commandClimbPose(const ClimbPose &p) {
  moveSpeedScale = LIFT_MOVE_SPEED_SCALE;
  setHip(FL, p.hipFL);   setKnee(FL, p.kneeFL);
  setHip(FR, p.hipFR);   setKnee(FR, p.kneeFR);
  setHip(RL, p.hipRL);   setKnee(RL, p.kneeRL);
  setHip(RR, p.hipRR);   setKnee(RR, p.kneeRR);
  unsigned long dur = 0;
  for (int i = 0; i < NUM_HIPS; i++) dur = max(dur, max(hipMoveDurationMs[i], kneeMoveDurationMs[i]));
  for (int i = 0; i < NUM_HIPS; i++) { hipMoveDurationMs[i] = dur; kneeMoveDurationMs[i] = dur; }
  climbMoveActive = true;
}

// ============================================================
// NEW STABLE LIFT (FR/RL/RR only, FL untouched) -- commanded just
// before driving forward over the step, transitioning from the
// original PRECLIMB stance into this newer, more forward-shifted
// stance: pulling FR's wheel closer to center leaves more clearance
// for FL's own reach to get further onto the step without bumping it.
// FL is deliberately left alone here -- it's already wherever its own
// lift/step-place sequence put it, not part of this stance change.
//
// Deliberately NOT commandClimbPose()/climbMoveActive -- by request,
// this move is NOT monitored against the MPU6050 for tilt/stability
// the way every other climb-pose move in this file is. Only waits for
// the three legs' servo moves to physically finish, nothing else.
// ============================================================
#define NEW_STABLE_HIP_FR   45
#define NEW_STABLE_KNEE_FR  170
#define NEW_STABLE_HIP_RL   4
#define NEW_STABLE_KNEE_RL  50
#define NEW_STABLE_HIP_RR   0
#define NEW_STABLE_KNEE_RR  30
// SECOND_FR_HIP/KNEE_RL/RR (the swapped compensation applied only at
// second_fr) are defined earlier in the file, next to
// SECOND_FR_TURN_SPEED -- macros must textually precede their first
// use, and startSecondLegOntoStep() uses them above this point.

bool newStableLiftActive = false;

void commandNewStableLift() {
  moveSpeedScale = LIFT_MOVE_SPEED_SCALE;
  setHip(FR, NEW_STABLE_HIP_FR);   setKnee(FR, NEW_STABLE_KNEE_FR);
  setHip(RL, NEW_STABLE_HIP_RL);   setKnee(RL, NEW_STABLE_KNEE_RL);
  setHip(RR, NEW_STABLE_HIP_RR);   setKnee(RR, NEW_STABLE_KNEE_RR);
  unsigned long dur = 0;
  dur = max(dur, max(hipMoveDurationMs[FR], kneeMoveDurationMs[FR]));
  dur = max(dur, max(hipMoveDurationMs[RL], kneeMoveDurationMs[RL]));
  dur = max(dur, max(hipMoveDurationMs[RR], kneeMoveDurationMs[RR]));
  hipMoveDurationMs[FR] = dur; kneeMoveDurationMs[FR] = dur;
  hipMoveDurationMs[RL] = dur; kneeMoveDurationMs[RL] = dur;
  hipMoveDurationMs[RR] = dur; kneeMoveDurationMs[RR] = dur;
  newStableLiftActive = true;
}

// Call every loop() pass -- no tilt check by request, just waits for
// FR/RL/RR to physically finish moving, then restores normal speed.
void updateNewStableLift() {
  if (!newStableLiftActive) return;
  if (!legMoveDone(FR) || !legMoveDone(RL) || !legMoveDone(RR)) return;
  newStableLiftActive = false;
  moveSpeedScale = 1.0;
  Serial.println(F("New stable lift pose reached (FR/RL/RR only, FL untouched -- not monitored for tilt)."));
}

// Same reasoning as checkLiftTiltSafety(), reused here since a climb
// pose move is just as capable of tipping the robot -- freezes all
// four legs (not three, since none of them are "the lifted leg" in
// this scheme) if tilt exceeds LIFT_TILT_ABORT_DEG.
bool checkClimbTiltSafety() {
  if (millis() - lastClimbTiltCheckMs < LIFT_TILT_CHECK_MS) return false;
  lastClimbTiltCheckMs = millis();

  float pitch, roll;
  if (!readMPU6050(pitch, roll)) return false; // bad read -- see readMPU6050()'s comment; don't abort on garbage, wait for the next poll
  if (fabs(pitch) < LIFT_TILT_ABORT_DEG && fabs(roll) < LIFT_TILT_ABORT_DEG) return false;

  Serial.print(F("CLIMB SAFETY ABORT: body tilt pitch="));
  Serial.print(pitch, 1);
  Serial.print(F(" roll="));
  Serial.print(roll, 1);
  Serial.println(F(" exceeded the safety threshold -- freezing all legs where they are."));

  for (int i = 0; i < NUM_HIPS; i++) freezeLeg(i);
  climbMoveActive = false;
  moveSpeedScale = 1.0;
  return true;
}

// Call every loop() pass -- watches an active climb-pose move for
// excess tilt, and clears climbMoveActive/restores normal speed once
// the move has genuinely settled.
void updateClimbMoveTracking() {
  if (!climbMoveActive) return;
  if (checkClimbTiltSafety()) return;

  bool allDone = true;
  for (int i = 0; i < NUM_HIPS; i++) allDone = allDone && legMoveDone(i);
  if (!allDone) return;

  climbMoveActive = false;
  moveSpeedScale = 1.0;
  Serial.println(F("Climb pose reached."));
}

// ============================================================
// STABLE PLATFORM (FL lift only)
// Split out on request into its own step, distinct from measuring the
// step (updateStepScan()/printScanChange()) and from actually lifting
// the leg (below): this is JUST "get all four legs into the
// hand-verified, IMU-confirmed-level stance first," commanded as
// absolute angles for the same reason as PRECLIMB_* above -- the FK/IK
// model can't be trusted to reproduce or verify this stance itself.
// ============================================================
void createStablePlatform() {
  setHip(FL, PRECLIMB_HIP_FL); setKnee(FL, PRECLIMB_KNEE_FL);
  setHip(FR, PRECLIMB_HIP_FR); setKnee(FR, PRECLIMB_KNEE_FR);
  setHip(RL, PRECLIMB_HIP_RL); setKnee(RL, PRECLIMB_KNEE_RL);
  setHip(RR, PRECLIMB_HIP_RR); setKnee(RR, PRECLIMB_KNEE_RR);
  unsigned long dur = 0;
  for (int i = 0; i < NUM_HIPS; i++) dur = max(dur, max(hipMoveDurationMs[i], kneeMoveDurationMs[i]));
  for (int i = 0; i < NUM_HIPS; i++) { hipMoveDurationMs[i] = dur; kneeMoveDurationMs[i] = dur; }
}

// Same as createStablePlatform(), but FR/RL/RR go to NEW_STABLE_*
// instead of PRECLIMB_HIP_FR/RL/RR -- used ONLY at the
// LIFT_REVERSE->LIFT_REMEASURE_UP transition (see that state), i.e.
// specifically "just before lifting the leg", NOT the initial
// LIFT_RAISING stance (which stays plain createStablePlatform(), "the
// original standalone sweep stance"). FL unaffected either way, still
// PRECLIMB_HIP_FL/KNEE_FL. Deliberately tilts the chassis on purpose
// to help the lift -- see LIFT_PRE_LIFT_PAUSE's exclusion from the
// reactive tilt net above.
void createNewStablePlatform() {
  setHip(FL, PRECLIMB_HIP_FL);   setKnee(FL, PRECLIMB_KNEE_FL);
  setHip(FR, NEW_STABLE_HIP_FR); setKnee(FR, NEW_STABLE_KNEE_FR);
  setHip(RL, NEW_STABLE_HIP_RL); setKnee(RL, NEW_STABLE_KNEE_RL);
  setHip(RR, NEW_STABLE_HIP_RR); setKnee(RR, NEW_STABLE_KNEE_RR);
  unsigned long dur = 0;
  for (int i = 0; i < NUM_HIPS; i++) dur = max(dur, max(hipMoveDurationMs[i], kneeMoveDurationMs[i]));
  for (int i = 0; i < NUM_HIPS; i++) { hipMoveDurationMs[i] = dur; kneeMoveDurationMs[i] = dur; }
}

// Sinks all four legs to get ToF1's beam back below the step's height
// for a fresh reading, then moves on to LIFT_REMEASURE_DOWN -- see
// REMEASURE_LOWER_DEG's comment. Called from LIFT_SETTLING for a
// step-place; LIFT_REMEASURE_DOWN itself triggers the pre-lift reverse
// once THAT reading confirms ToF1 can actually see the step (see
// LIFT_REVERSE_CLEARANCE_MM's comment).
void startLiftSink() {
  setHip(FL, hipPos[FL] + REMEASURE_LOWER_DEG);
  setKnee(FL, kneePos[FL] - REMEASURE_LOWER_DEG);
  setHip(FR, hipPos[FR] + REMEASURE_LOWER_DEG);
  setKnee(FR, kneePos[FR] - REMEASURE_LOWER_DEG);
  setKnee(RL, kneePos[RL] - REMEASURE_LOWER_DEG);
  setKnee(RR, kneePos[RR] - REMEASURE_LOWER_DEG);
  unsigned long dur = 0;
  for (int i = 0; i < NUM_HIPS; i++) dur = max(dur, max(hipMoveDurationMs[i], kneeMoveDurationMs[i]));
  for (int i = 0; i < NUM_HIPS; i++) { hipMoveDurationMs[i] = dur; kneeMoveDurationMs[i] = dur; }
  liftState = LIFT_REMEASURE_DOWN;
}

// Moves liftLegIdx forward to the step's x while staying at the
// elevated clear height (computeClearY()) -- NOT yet the step's own
// target y -- so the foot is already past the leading edge before it
// ever descends to tread height. Shared by LIFT_TUCK (leg already
// within reach) and LIFT_APPROACH (after closing the gap on the
// wheels) so both land in the same next state the same way.
//
// forceBranch pins FL/FR to ONE elbow branch for both this move and
// LIFT_CLEAR below -- confirmed on hardware that without forcing, the
// reach can flip branch between the two calls (visually: "swaps which
// way the elbow is held" and places the foot back on the floor
// instead of the step), since legMoveDone() only checks the move
// finished, not which branch it finished in.
//
// Branch was first tried as 1 (matching the safe-knee step's knee=270,
// "above KNEE_START folds forward"), but confirmed on hardware to
// still land wrong: for this far-forward, shallow-y reach, branch 1
// (theta2 = +kneeMag) swings the calf the SAME rotational direction as
// the hip's forward lean -- a hyper-extended curl that folds back up
// near the hip instead of reaching down onto the step. Branch 0
// (theta2 = -kneeMag) folds the calf the OPPOSITE way from the hip's
// lean -- thigh forward, shin bent back down to the foot, the natural
// "reaching forward onto a step" shape -- so that's forced here
// instead. The safe-knee step itself (knee=270, branch 1) is a
// different target (tucked near the hip) and is left alone.
void startTraverseToStep() {
  int forceBranch = (liftLegIdx == FL || liftLegIdx == FR) ? 0 : -1;
  if (!setFoot(liftLegIdx, liftStepForwardMM, computeClearY(), forceBranch)) {
    Serial.println(F("Step placement aborted: clear-traverse target unreachable -- check step distance against this leg's workspace."));
    abortLiftSequence();
    return;
  }
  liftState = LIFT_CLEAR;
}

// Steps the lift/reach/lower sequence forward -- call every loop() pass.
void updateLiftSequence() {
  // The reactive tilt-abort net is deliberately OFF during LIFT_RAISING,
  // LIFT_SHIFTING, and LIFT_SETTLING -- confirmed on hardware that
  // repositioning into the pre-climb stance produces real but harmless
  // transient tilt (here: roll=15.9 right after "Stand target
  // reached", tripping the net before the stance had even settled),
  // the same way manually jogging into position never had anything
  // watching for transient wobble mid-move -- only a single 'level'
  // check once everything had actually settled. LIFT_SETTLING's own
  // dwell-then-check (LIFT_SETTLE_DWELL_MS, then the pre-lift IMU gate)
  // IS that same settled-state check, still fully active -- so a
  // stance that's ACTUALLY unstable once settled is still caught
  // there, just not mid-reposition. The reactive net re-arms from
  // LIFT_TUCK onward, where the real risk (FL's own mass swinging
  // through the reach) actually lives.
  // LIFT_REVERSE is excluded too -- it's a wheeled reposition with
  // every leg still planted/unmoving, the same "expected transient,
  // not a real fall" category as the states above it, not the
  // leg-swinging risk the net exists to catch.
  // LIFT_PRE_LIFT_PAUSE excluded by explicit request: createNewStablePlatform()
  // (commanded at the LIFT_REVERSE->LIFT_REMEASURE_UP transition, right
  // before this pause) deliberately tilts the chassis on purpose to
  // help the upcoming lift -- not something to flag as a fault.
  if (LIFT_REACTIVE_TILT_NET_ENABLED &&
      liftState != LIFT_IDLE && liftState != LIFT_HOLDING &&
      liftState != LIFT_RAISING && liftState != LIFT_SHIFTING &&
      liftState != LIFT_SETTLING && liftState != LIFT_REVERSE &&
      liftState != LIFT_REMEASURE_DOWN && liftState != LIFT_REMEASURE_UP &&
      liftState != LIFT_PRE_LIFT_PAUSE) {
    if (checkLiftTiltSafety()) return;
  }

  if (liftState == LIFT_RAISING) {
    if (standMoveInProgress) return; // still rising to LIFT_STAND_TARGET_PROGRESS

    // NOW capture foot positions -- after the raise, not before it --
    // since standing taller moves every foot's position relative to
    // its hip (see applyStandProgress()/heightAtStandProgress()).
    legForwardKinematics(liftLegIdx, liftOrigX, liftOrigY);
    for (int k = 0; k < 3; k++) {
      legForwardKinematics(liftStanceIdx[k], liftStanceX[k], liftStanceY[k]);
    }

    if (liftLegIdx == FL) {
      // Step 2 of 3 (measure / stable platform / lift): get all four
      // legs into the hand-verified stance -- see createStablePlatform()
      // above. Commanded as absolute angles, not through setFoot()/IK,
      // since the model that would validate an IK target has already
      // proven unreliable at these angles. The geometric
      // stability-margin check in LIFT_SHIFTING is skipped for the same
      // reason (liftUsingVerifiedStance) -- the real IMU-based checks
      // (checkLiftTiltSafety(), the pre-lift tilt gate) stay fully
      // active regardless, since those measure the actual robot, not a
      // model of it.
      createStablePlatform();
      liftUsingVerifiedStance = true;
      // createStablePlatform() just moved the chassis to a fixed,
      // hand-verified stance instead of interpolating through
      // applyStandProgress() -- so two things anchored to the OLD pose
      // are now stale: lastCommandedHeight, and liftStepForwardMM (the
      // box's distance, measured by printScanChange() from the ToF1
      // reading taken at the scan's 0%-progress baseline -- i.e. from
      // wherever the hip WAS then, not wherever it ends up after the
      // subsequent 0.9 stand raise and this stance's own large joint
      // jump). Both are corrected here using this stance's own KNOWN
      // target angles -- not a hipPos[]/kneePos[] read (the servos
      // haven't physically arrived yet) and not the FK model applied
      // generally (unreliable at these angles), just this one specific
      // already-known target, evaluated with the same trig the rest of
      // the file uses.
      {
        float t1 = radians((float)(PRECLIMB_HIP_FL - HIP_START[FL]));
        float t2 = radians((float)(PRECLIMB_KNEE_FL - KNEE_START[FL]));
        float footXFinal = LEG_THIGH_MM * sin(t1) + LEG_CALF_MM * sin(t1 + t2);
        lastCommandedHeight = LEG_THIGH_MM * cos(t1) + LEG_CALF_MM * cos(t1 + t2);

        // Same self-motion-compensation idea updateStepScan() already
        // applies mid-sweep, extended past where the scan stopped:
        // this stance keeps FL's foot planted (createStablePlatform()
        // is a stance, not a lift), so any change in its position
        // relative to the hip between the scan's 0% baseline and this
        // final stance is exactly how far the hip -- and ToF1, fixed
        // to the same chassis -- has shifted. Confirmed on hardware as
        // a real, previously-missing correction: without it the reach
        // undershoots the box's true remaining distance, landing short
        // (through empty space, past the box's edge) instead of onto
        // its face.
        float chassisForwardShift = footXAtStandProgress(FL, 0.0) - footXFinal;
        liftStepForwardMM -= chassisForwardShift;
      }
      liftState = LIFT_SHIFTING;
      return;
    }

    float bx[3], by[3];
    for (int k = 0; k < 3; k++) footBodyPosition(liftStanceIdx[k], bx[k], by[k]);

    float bestShift, bestMargin;
    findBestStabilityShift(bx, by, liftStanceX, liftStanceY, bestShift, bestMargin);
    if (bestMargin < MIN_STABILITY_MARGIN_MM) {
      Serial.print(F("Lift aborted: best achievable stability margin is "));
      Serial.print(bestMargin, 0);
      Serial.print(F("mm, below the "));
      Serial.print(MIN_STABILITY_MARGIN_MM, 0);
      Serial.println(F("mm safety floor -- not attempting a shift from this stance."));
      abortLiftSequence();
      return;
    }

    // A planted foot is fixed on the ground -- commanding it "forward"
    // in body-local terms moves the body backward relative to it, and
    // vice versa, so shift each stance leg the opposite way to move
    // the body toward the shift point that maximizes real margin.
    //
    // REAR_KNEE_FOLD_EXPERIMENT: forces RL/RR onto the forward-fold
    // elbow branch (see solveLegIK()'s forceBranch) instead of the
    // default backward fold, for this same foot point -- same ground
    // contact, same stability-margin math, just a different internal
    // hip/knee combination. This is the concrete code-level meaning of
    // "bring the rear knees inward a bit more": doesn't touch geometry
    // at all, only which of the two valid joint configurations holds
    // the stance. UNCONFIRMED whether this actually resists forward
    // tipping better -- flash and watch the rear legs' fold direction
    // during a plain lift_fl to check it moved the way expected before
    // trusting it on a full step-place reach.
    for (int k = 0; k < 3; k++) {
      int i = liftStanceIdx[k];
      int forceBranch = (i == RL || i == RR) ? 1 : -1;
      if (!setFoot(i, liftStanceX[k] - bestShift, liftStanceY[k], forceBranch)) {
        Serial.println(F("Lift aborted: weight-shift target unreachable."));
        abortLiftSequence();
        return;
      }
    }
    liftState = LIFT_SHIFTING;

  } else if (liftState == LIFT_SHIFTING) {
    for (int k = 0; k < 3; k++) if (!legMoveDone(liftStanceIdx[k])) return;
    // Servos reporting "done" only means they reached their commanded
    // angle, not that the chassis has stopped physically settling --
    // hold here for LIFT_SETTLE_DWELL_MS before trusting any stability
    // check, rather than reading one the instant the servos stop.
    liftSettleStartMs = millis();
    liftState = LIFT_SETTLING;

  } else if (liftState == LIFT_SETTLING) {
    if (millis() - liftSettleStartMs < LIFT_SETTLE_DWELL_MS) return; // still holding, letting things settle

    // Skipped for the verified pre-climb stance -- footBodyPosition()'s
    // forward-kinematics model has proven unreliable (implausible
    // above-hip results) at the large angles that stance uses, so
    // trusting it here would risk a false abort on a stance already
    // confirmed by hand.
    if (!liftUsingVerifiedStance) {
      float ax, ay, bx2, by2, cx, cy;
      footBodyPosition(liftStanceIdx[0], ax, ay);
      footBodyPosition(liftStanceIdx[1], bx2, by2);
      footBodyPosition(liftStanceIdx[2], cx, cy);
      float settledMargin = stabilityMargin(0, 0, ax, ay, bx2, by2, cx, cy);
      if (settledMargin < MIN_STABILITY_MARGIN_MM) {
        Serial.print(F("Lift aborted: settled stability margin is "));
        Serial.print(settledMargin, 0);
        Serial.println(F("mm after shifting -- below the safety floor, not proceeding to lift."));
        abortLiftSequence();
        return;
      }
    }
    // The margin check above is purely geometric (projected support
    // triangle) -- it says the shift SHOULD be stable, not that the
    // real robot IS level right now. All four feet are still grounded
    // at this point, so it's cheap and safe to actually check with the
    // IMU before committing to remove one of them.
    {
      float pitch, roll;
      // Bad read -- see readMPU6050()'s comment. liftState stays
      // LIFT_SETTLING (still past its dwell), so this just retries on
      // the next loop() pass instead of acting on garbage.
      if (!readMPU6050(pitch, roll)) return;
      if (fabs(pitch) > LIFT_PRELIFT_TILT_LIMIT_DEG || fabs(roll) > LIFT_PRELIFT_TILT_LIMIT_DEG) {
        Serial.print(F("Lift aborted: body tilt pitch="));
        Serial.print(pitch, 1);
        Serial.print(F(" roll="));
        Serial.print(roll, 1);
        Serial.println(F(" already exceeds the pre-lift stability check -- not safe to lift from this stance."));
        abortLiftSequence();
        return;
      }
    }
    if (liftIsStepPlace) {
      startLiftSink();
    } else {
      // Step 3 of 3 (measure / stable platform / lift), part A: move
      // the knee ALONE to its safe position first -- see SAFE-KNEE
      // LIFT above -- before the hip does anything. Not IK/setFoot():
      // a single-joint absolute move, so there's no risk of the model
      // picking a different knee angle than the verified-safe one.
      setKnee(liftLegIdx, LIFT_SAFE_KNEE_FL);
      liftState = LIFT_KNEE_SAFE;
    }

  } else if (liftState == LIFT_REMEASURE_DOWN) {
    {
      bool allDone = true;
      for (int i = 0; i < NUM_HIPS; i++) allDone = allDone && legMoveDone(i);
      if (!allDone) return;
    }
    // Settled at the lower height -- take the live reading now, while
    // the beam should actually be able to see the step's front face.
    pollTofSensors();
    if (liftIsStepPlace) {
      // REDESIGNED by explicit request: compute the real final reach
      // target and drive to it NOW, while ToF1 still has a clear line
      // of sight to the step -- moved here from the old LIFT_TUCK, and
      // replacing the old "reverse away for clearance" step entirely.
      // That reverse only ever existed because the OLD knee-fold
      // direction (LIFT_SAFE_KNEE_FL toward KNEE_MAX) swung the leg
      // TOWARD the step during lift-off, needing room to do that
      // safely. LIFT_SAFE_KNEE_FL now folds toward the body instead
      // (see that constant's comment) -- the leg never swings toward
      // the step during lift-off anymore, so there's no reason left to
      // back away first. Measure now, then move to the precise
      // standoff for a shallow ~STEP_LANDING_DEPTH_MM placement,
      // BEFORE the leg ever lifts and takes ToF1's view of the step
      // away with it (tilting the chassis, then lifting FL, both move
      // ToF1's aim off the step -- confirmed on hardware).
      //
      // Same maxReach^2 = x^2 + y^2 reach-ceiling math the old
      // LIFT_TUCK used, sized against whichever of computeClearY()
      // (the elevated traverse height) or the final step height has
      // the larger magnitude, so the chosen x is safely reachable at
      // both.
      float maxReach = LEG_THIGH_MM + LEG_CALF_MM;
      float yClear = computeClearY();
      float yFinal = lastCommandedHeight - liftStepHeightMM;
      float yLimiting = (fabs(yFinal) > fabs(yClear)) ? yFinal : yClear;
      float targetForwardMM = sqrt(max(0.0f, maxReach * maxReach - yLimiting * yLimiting))
                               - LIFT_APPROACH_REACH_MARGIN_MM;
      float driveStopMM = targetForwardMM - STEP_LANDING_DEPTH_MM;

      if (tof1_ok) {
        float freshForwardMM = (float)tof1_mm + TOF1_FORWARD_OFFSET_MM;
        Serial.print(F("Re-measured step distance: ")); Serial.print(freshForwardMM, 0);
        Serial.println(F("mm forward."));
        Serial.print(F("Targeting ")); Serial.print(targetForwardMM, 0);
        Serial.print(F("mm forward (near-max reach) -- closing the gap on the wheels to "));
        Serial.print(driveStopMM, 0);
        Serial.println(F("mm (ToF-guided) so the reach lands the requested depth onto the step, not just touches it."));
        startDriveToTof(LIFT_APPROACH_SPEED, driveStopMM - TOF1_FORWARD_OFFSET_MM, LIFT_APPROACH_TIMEOUT_MS);
      } else {
        // No live ToF1 reading to steer by -- fall back to a computed,
        // timed drive from the last known good distance (the scan
        // estimate already sitting in liftStepForwardMM). Same
        // estimate-based approach WHEEL_MM_PER_MS_AT_APPROACH was
        // built for (see that constant).
        float approachDistanceMM = liftStepForwardMM - driveStopMM;
        unsigned long approachDriveMs = (approachDistanceMM > 0)
          ? (unsigned long)round(approachDistanceMM / WHEEL_MM_PER_MS_AT_APPROACH)
          : 0;
        Serial.println(F("Re-measure: ToF1 reading invalid -- falling back to a timed drive from the scan estimate."));
        Serial.print(F("Targeting ")); Serial.print(targetForwardMM, 0);
        Serial.print(F("mm forward -- estimated ")); Serial.print(approachDistanceMM, 0);
        Serial.print(F("mm / ")); Serial.print(approachDriveMs);
        Serial.println(F("ms (timed, no ToF steering)."));
        startDrive(LIFT_APPROACH_SPEED, approachDriveMs);
      }
      liftStepForwardMM = targetForwardMM;
      // Name kept from the old reverse step -- this now just waits for
      // the pre-lift positioning drive above to finish, see that state.
      liftState = LIFT_REVERSE;
    } else {
      // Plain lifts never actually reach LIFT_REMEASURE_DOWN (see
      // LIFT_SETTLING -- only liftIsStepPlace routes here via
      // startLiftSink()), but handled defensively regardless.
      if (!tof1_ok) Serial.println(F("Re-measure: ToF1 reading invalid, keeping the scan-derived estimate."));
      createNewStablePlatform();
      liftState = LIFT_REMEASURE_UP;
    }

  } else if (liftState == LIFT_REVERSE) {
    if (driveActive) return; // still closing the gap to the pre-lift standoff (or timed out -- either way driveActive clears on its own)
    // Re-snap through createNewStablePlatform() (not createStablePlatform())
    // before continuing -- by request, this is specifically "just
    // before lifting the leg", the point where the newer FR/RL/RR
    // stance (pulled closer to center) should apply, deliberately
    // tilting the chassis to help the upcoming lift. The FIRST stance
    // (LIFT_RAISING, above) stays plain createStablePlatform() -- "the
    // original standalone sweep stance".
    createNewStablePlatform();
    liftState = LIFT_REMEASURE_UP;

  } else if (liftState == LIFT_REMEASURE_UP) {
    {
      bool allDone = true;
      for (int i = 0; i < NUM_HIPS; i++) allDone = allDone && legMoveDone(i);
      if (!allDone) return;
    }
    // Deliberate pause here before lifting -- requested directly, lets
    // the deliberately-tilted stance above actually settle first.
    liftPreLiftPauseStartMs = millis();
    liftState = LIFT_PRE_LIFT_PAUSE;

  } else if (liftState == LIFT_PRE_LIFT_PAUSE) {
    if (millis() - liftPreLiftPauseStartMs < LIFT_PRE_LIFT_PAUSE_MS) return;
    // Hip and knee move SIMULTANEOUSLY here, not knee-first-then-hip
    // (the LIFT_KNEE_SAFE path plain lift_fl still uses) -- same
    // reasoning as second_fr's own fix: folding the knee toward the
    // body FIRST, while the hip is still down, would dip the foot
    // toward the ground before the hip gets a chance to lift it clear.
    //
    // Goes to LIFT_FL_NUDGE_START now (not straight to LIFT_FR_RISE) --
    // by request, FL's own step-place uses the same staged technique
    // already proven for second_fr: hip rises to a peak FIRST
    // (LIFT_LIFTED_HIP_FL, now 200), only THEN does the knee extend
    // out, only THEN does the foot get placed (hip alone lowers). See
    // LIFT_FR_RISE/EXTEND/DESCEND -- that whole path is leg-agnostic
    // despite the FR-derived name. LIFT_FL_NUDGE_START/WAIT sits in
    // between for FL specifically -- see that state's comment.
    setKnee(liftLegIdx, LIFT_SAFE_KNEE_FL);
    setHip(liftLegIdx, LIFT_LIFTED_HIP_FL);
    liftHipPeak = LIFT_LIFTED_HIP_FL;
    liftState = LIFT_FL_NUDGE_START;

  } else if (liftState == LIFT_FL_NUDGE_START) {
    if (!legMoveDone(liftLegIdx)) return; // wait for the leg to finish lifting first
    // Small forward nudge, requested directly -- see LIFT_FL_NUDGE_MS's
    // comment for why: the createNewStablePlatform() stance-switch
    // pulls the chassis away from the step a little on its own, and
    // this compensates for it once the leg is safely up and clear,
    // before the reach math (LIFT_FR_RISE) runs.
    startDrive(LIFT_APPROACH_SPEED, LIFT_FL_NUDGE_MS);
    liftState = LIFT_FL_NUDGE_WAIT;

  } else if (liftState == LIFT_FL_NUDGE_WAIT) {
    if (driveActive) return;
    liftState = LIFT_FR_RISE;

  } else if (liftState == LIFT_KNEE_SAFE) {
    if (!legMoveDone(liftLegIdx)) return; // knee still settling into its safe position
    // Step 3 of 3, part B: NOW lift the hip, with the knee already
    // safely folded and holding still -- see LIFT_LIFTED_HIP_FL above.
    // Only reached by the plain lift_fl/lift_fr path -- step-place
    // (LIFT_PRE_LIFT_PAUSE above) skips this and goes straight to
    // LIFT_FR_RISE instead.
    setHip(liftLegIdx, (liftLegIdx == FR) ? LIFT_LIFTED_HIP_FR : LIFT_LIFTED_HIP_FL);
    liftState = LIFT_TUCK;

  } else if (liftState == LIFT_TUCK) {
    if (!legMoveDone(liftLegIdx)) return; // hip still lifting
    // Only reached by the plain lift_fl/fr/rl/rr path now -- both
    // step-place cases (FL's own placement via LIFT_PRE_LIFT_PAUSE, and
    // second_fr) go straight to LIFT_FR_RISE instead, bypassing this
    // state entirely.
    Serial.println(F("Leg lifted (tucked)."));
    liftState = LIFT_HOLDING;

  } else if (liftState == LIFT_CLEAR) {
    if (!legMoveDone(liftLegIdx)) return;
    // Now purely a vertical descent at a fixed x -- the leading edge
    // is already behind the foot, so this can't clip the step face.
    // Set up LIFT_DESCEND's incremental steps rather than commanding
    // the full descent in one move -- see LIFT_DESCEND_STEPS above.
    liftDescendStartY = computeClearY();
    liftDescendEndY = lastCommandedHeight - liftStepHeightMM;
    liftDescendStepIdx = 0;
    liftDescendStoppedEarly = false;
    // Bad read -- see readMPU6050()'s comment. Retry next loop() pass
    // rather than starting the descent with a stale/wrong baseline
    // (liftDescendBasePitch/Roll would otherwise keep whatever they
    // were left at by the previous leg's descent).
    if (!readMPU6050(liftDescendBasePitch, liftDescendBaseRoll)) return;
    liftState = LIFT_DESCEND;

  } else if (liftState == LIFT_DESCEND) {
    if (!legMoveDone(liftLegIdx)) return;

    if (liftDescendStepIdx > 0) {
      // Only compare once at least one increment has actually landed --
      // this is the contact check described in LIFT_DESCEND_STEPS's
      // comment above: stop pressing as soon as tilt moves off its
      // pre-descent baseline by more than a small amount, rather than
      // waiting for the full LIFT_TILT_ABORT_DEG safety net to trip.
      float pitch, roll;
      // Bad read -- see readMPU6050()'s comment. Skip this poll rather
      // than reading a false contact (or silently missing one) off
      // garbage data.
      if (!readMPU6050(pitch, roll)) return;
      if (fabs(pitch - liftDescendBasePitch) > LIFT_CONTACT_TILT_DELTA_DEG ||
          fabs(roll - liftDescendBaseRoll) > LIFT_CONTACT_TILT_DELTA_DEG) {
        liftDescendStoppedEarly = (liftDescendStepIdx < LIFT_DESCEND_STEPS);
        liftState = LIFT_REACH;
        return;
      }
    }

    if (liftDescendStepIdx >= LIFT_DESCEND_STEPS) {
      liftState = LIFT_REACH; // reached the full nominal descent with no contact signal along the way
      return;
    }

    liftDescendStepIdx++;
    float t = (float)liftDescendStepIdx / (float)LIFT_DESCEND_STEPS;
    float stepY = liftDescendStartY + (liftDescendEndY - liftDescendStartY) * t;
    // Same forceBranch reasoning as LIFT_TUCK above.
    int forceBranch = (liftLegIdx == FL || liftLegIdx == FR) ? 0 : -1;
    if (!setFoot(liftLegIdx, liftStepForwardMM, stepY, forceBranch)) {
      Serial.println(F("Step placement aborted: descent target unreachable -- check step height against this leg's workspace."));
      liftState = LIFT_HOLDING; // still elevated and clear of the step; leave it there, not mid-fault
      return;
    }

  } else if (liftState == LIFT_REACH) {
    if (!legMoveDone(liftLegIdx)) return;
    Serial.println(liftDescendStoppedEarly
      ? "Foot placed on step (stopped early: contact detected via tilt before reaching the full nominal descent)."
      : "Foot placed on step.");
    liftState = LIFT_HOLDING;

  } else if (liftState == LIFT_FR_TURN1) {
    if (turnTestActive) return; // still pivoting to create lift clearance
    // Small straight reverse of all four wheels, requested directly --
    // see SECOND_FR_PRE_REVERSE_SPEED/MS's comment: the pivot alone
    // still let FR catch on the step, so this adds real backward
    // translation on top of it, right before the leg actually lifts.
    startDrive(-SECOND_FR_PRE_REVERSE_SPEED, SECOND_FR_PRE_REVERSE_MS);
    liftState = LIFT_FR_PRE_REVERSE;

  } else if (liftState == LIFT_FR_PRE_REVERSE) {
    if (driveActive) return; // still reversing a tiny amount before the lift
    // IK-computed vertical lift, not a fixed hip/knee angle pair --
    // requested directly after the wheel kept catching on the step
    // edge despite the pivot: reusing SECOND_FR_HIP_PEAK/SAFE_KNEE
    // (arbitrary constants, never checked against the actual step
    // geometry) let the foot's real path clip the step on the way up.
    // computeClearY() -- already used by the original LIFT_CLEAR/
    // LIFT_TUCK path -- gives a height verified clear of BOTH the
    // step's own top surface AND the floor. Re-capture liftOrigX/Y for
    // THIS leg first: they're only ever set for the first leg's own
    // sequence (LIFT_RAISING), so during second_fr they'd still be
    // stale from FL otherwise, and computeClearY()'s ground-clearance
    // term needs FR's real current position, not FL's.
    legForwardKinematics(liftLegIdx, liftOrigX, liftOrigY);
    if (!setFoot(liftLegIdx, liftOrigX, computeClearY(), 0)) {
      Serial.println(F("second_fr aborted: clearance-lift target unreachable."));
      abortLiftSequence();
      return;
    }
    // Also drive the knee explicitly to 0 (fully folded toward the
    // body) at the same time as the hip lift above, requested
    // directly -- makes sure the knee reaches this fold BEFORE the leg
    // finishes lifting, not left wherever setFoot()'s IK solve happened
    // to put it. This overrides setFoot()'s own knee target; the hip
    // target above was solved assuming a different knee angle, so the
    // foot won't land at EXACTLY computeClearY() once the knee reaches
    // 0 instead of that -- close enough in practice, and the fold-
    // toward-body direction matters more here than the exact height.
    setKnee(liftLegIdx, 0);
    liftState = LIFT_FR_CLEAR_RISE;

  } else if (liftState == LIFT_FR_CLEAR_RISE) {
    if (!legMoveDone(liftLegIdx)) return; // still rising straight up to the verified-clear height
    liftHipPeak = hipPos[liftLegIdx]; // whatever hip angle the IK solve actually landed on -- LIFT_FR_DESCEND interpolates from here
    liftState = LIFT_FR_RISE;

  } else if (liftState == LIFT_FR_RISE) {
    if (!legMoveDone(liftLegIdx)) return; // LIFT_FR_CLEAR_RISE's lift should already be done by the time this is reached, checked again defensively
    // Solve the real final target ONCE here -- same geometry the
    // normal LIFT_CLEAR/LIFT_DESCEND path already uses
    // (liftStepForwardMM, lastCommandedHeight - liftStepHeightMM), just
    // solved directly instead of via setFoot() so the hip/knee values
    // can be driven one at a time instead of together.
    float targetHip, targetKnee;
    int forceBranch = 0; // same fixed elbow branch LIFT_TUCK/LIFT_DESCEND force for FL/FR
    if (!solveLegIK(liftLegIdx, liftStepForwardMM, lastCommandedHeight - liftStepHeightMM, targetHip, targetKnee, forceBranch)) {
      Serial.println(F("Step placement aborted: final step target unreachable."));
      abortLiftSequence();
      return;
    }
    secondFrFinalHip  = (int)round(targetHip);
    secondFrFinalKnee = (int)round(targetKnee);
    // Knee alone extends from here -- hip stays at liftHipPeak, already
    // above the step's height, so the extending foot swings over the
    // top instead of into the step's front face.
    setKnee(liftLegIdx, secondFrFinalKnee);
    liftState = LIFT_FR_EXTEND;

  } else if (liftState == LIFT_FR_EXTEND) {
    if (!legMoveDone(liftLegIdx)) return; // knee still extending, hip held at the peak
    // Now lower the hip ALONE onto the step, knee held at
    // secondFrFinalKnee -- same incremental contact-check as
    // LIFT_DESCEND (reusing its bookkeeping vars), just driving hip
    // only instead of a combined setFoot() move.
    liftDescendStepIdx = 0;
    liftDescendStoppedEarly = false;
    if (!readMPU6050(liftDescendBasePitch, liftDescendBaseRoll)) return;
    liftState = LIFT_FR_DESCEND;

  } else if (liftState == LIFT_FR_DESCEND) {
    if (!legMoveDone(liftLegIdx)) return;

    if (liftDescendStepIdx > 0) {
      float pitch, roll;
      if (!readMPU6050(pitch, roll)) return;
      float pitchDelta = fabs(pitch - liftDescendBasePitch);
      float rollDelta  = fabs(roll - liftDescendBaseRoll);
      if (pitchDelta > SECOND_FR_DESCEND_TILT_DELTA_DEG || rollDelta > SECOND_FR_DESCEND_TILT_DELTA_DEG) {
        liftDescendStoppedEarly = (liftDescendStepIdx < LIFT_DESCEND_STEPS);
        Serial.print(F("Foot placed on step"));
        if (liftDescendStoppedEarly) {
          Serial.print(F(" (stopped early: contact detected via tilt -- pitchDelta="));
          Serial.print(pitchDelta, 1);
          Serial.print(F(" rollDelta="));
          Serial.print(rollDelta, 1);
          Serial.print(F(" vs threshold "));
          Serial.print(SECOND_FR_DESCEND_TILT_DELTA_DEG, 1);
          Serial.print(F(" -- if this fires with no real contact, raise SECOND_FR_DESCEND_TILT_DELTA_DEG above these delta values)"));
        }
        Serial.println(F("."));
        // FL's own placement must NOT move at all once placed --
        // confirmed on hardware that any wheel motion here can walk it
        // back off the step. second_fr gets the requested post-
        // placement compensation drive+turn instead (LIFT_FR_POST_DRIVE
        // then LIFT_FR_POST_TURN -- see their comments for why in that
        // order).
        if (liftIsSecondLeg) {
          startDrive(SECOND_FR_POST_DRIVE_SPEED, SECOND_FR_POST_DRIVE_MS);
          liftState = LIFT_FR_POST_DRIVE;
        } else {
          liftState = LIFT_HOLDING;
        }
        return;
      }
    }

    if (liftDescendStepIdx >= LIFT_DESCEND_STEPS) {
      Serial.println(F("Foot placed on step."));
      if (liftIsSecondLeg) {
        startDrive(SECOND_FR_POST_DRIVE_SPEED, SECOND_FR_POST_DRIVE_MS);
        liftState = LIFT_FR_POST_DRIVE;
      } else {
        liftState = LIFT_HOLDING;
      }
      return;
    }

    liftDescendStepIdx++;
    float t = (float)liftDescendStepIdx / (float)LIFT_DESCEND_STEPS;
    int stepHip = liftHipPeak + (int)round((secondFrFinalHip - liftHipPeak) * t);
    setHip(liftLegIdx, stepHip);

  } else if (liftState == LIFT_FR_POST_DRIVE) {
    if (driveActive) return; // still driving forward -- runs BEFORE the turn-back now, so the turn has more margin from the step edge to work with
    // Turn back now that the forward drive has moved both front feet
    // further onto the step first, requested directly -- FL+RL
    // reverse, FR+RR forward, undoing the pre-lift pivot.
    if (!startTurnTestLR(-SECOND_FR_TURN_SPEED, SECOND_FR_TURN_SPEED, SECOND_FR_TURN_MS)) {
      Serial.println(F("second_fr: could not start the turn-back (something else active) -- staying as-is."));
      liftState = LIFT_HOLDING;
      return;
    }
    liftState = LIFT_FR_POST_TURN;

  } else if (liftState == LIFT_FR_POST_TURN) {
    if (turnTestActive) return; // still turning back
    liftState = LIFT_HOLDING;

  } else if (liftState == LIFT_RISE) {
    if (!legMoveDone(liftLegIdx)) return; // settling into the safe lifted pose
    // Back to the original stable-platform prep angle -- see
    // startLower()'s comment. No IK: a direct, verified absolute move,
    // same as the ascent.
    setHip(liftLegIdx, (liftLegIdx == FR) ? PRECLIMB_HIP_FR : PRECLIMB_HIP_FL);
    setKnee(liftLegIdx, (liftLegIdx == FR) ? PRECLIMB_KNEE_FR : PRECLIMB_KNEE_FL);
    liftState = LIFT_UNTUCK;

  } else if (liftState == LIFT_UNTUCK) {
    if (!legMoveDone(liftLegIdx)) return;
    // For a second leg (see startSecondLegOntoStep()), liftStanceIdx is
    // stale from the FIRST leg's own sequence, and nothing else
    // actually moved this time (the first leg stayed fixed the whole
    // time) -- so there's nothing to restore, skip straight to
    // LIFT_LOWERING.
    if (!liftIsSecondLeg) {
      for (int k = 0; k < 3; k++) setFoot(liftStanceIdx[k], liftStanceX[k], liftStanceY[k]);
    }
    liftState = LIFT_LOWERING;

  } else if (liftState == LIFT_LOWERING) {
    bool allDone = legMoveDone(liftLegIdx);
    if (!liftIsSecondLeg) {
      for (int k = 0; k < 3; k++) allDone = allDone && legMoveDone(liftStanceIdx[k]);
    }
    if (!allDone) return;
    Serial.println(F("Leg lowered, stance restored."));
    abortLiftSequence(); // successful completion, not actually an abort -- just reuses the same "return to idle, reset speed" bookkeeping
  }
}

// See AUTO-STEP's enum/comment near the stand_sweep section above --
// this function itself has to sit here, after LiftState/liftState, so
// the LIFT_HOLDING/LIFT_IDLE checks below actually compile.
// startPlaceOnStep() is called directly from updateStepScan() at the
// moment a crossing is found (no forced return-to-stand first -- see
// its comment), so this only ever needs to track the AUTO_PLACING
// sequence through to completion or abort.
void updateAutoStep() {
  if (autoStepState != AUTO_PLACING) return;
  if (liftState == LIFT_HOLDING) {
    if (liftTiltAborted) {
      Serial.println(F("Auto step placement ABORTED -- tilt safety triggered mid-sequence, foot NOT reliably placed. Send 'lower' to retract."));
    } else {
      Serial.println(F("Auto step placement complete -- foot on step. Send 'lower' when ready to retract."));
    }
    autoStepState = AUTO_IDLE;
  } else if (liftState == LIFT_IDLE) {
    // The sequence aborted somewhere along the way (support triangle
    // check, an unreachable target) -- already reported by whichever
    // step caused it; nothing more to do here.
    autoStepState = AUTO_IDLE;
  }
}

// ============================================================
// MPU6050 — raw Wire
// ============================================================
void setupMPU6050() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(PWR_MGMT_1);
  Wire.write(0x00);
  Wire.endTransmission(true);
  delay(100);
  Serial.println(F("MPU6050 ready."));
}

// Returns false on a failed/short I2C read, OR a full read that comes
// back physically implausible, instead of trusting whatever's in the
// buffer -- both confirmed on hardware as real failure modes, not
// theoretical.
//
// The short-read case: a glitched read leaves Wire.read() returning -1
// for every unreceived byte, so AcX=AcY=AcZ=-1, and BOTH pitch and
// roll formulas collapse to atan2(-1, sqrt(2))*180/PI = -35.26 degrees
// -- a plausible-looking but completely bogus tilt on both axes at
// once, identical to one decimal place. That's what tripped one abort
// with the robot barely leaning.
//
// The full-but-corrupted case: a SEPARATE abort logged pitch=76.6
// roll=-9.4 (not identical, so not the case above) while the user
// confirmed there was no real tilt anywhere close to that -- a read
// that came back as 6 real bytes, just not the RIGHT 6 bytes (e.g. an
// I2C glitch during the wheel motors' own PWM/current-draw noise,
// separate from the already-documented servo noise, shifting which
// byte lands where). ACCEL_CONFIG is never written, so this sensor
// stays at its power-on default +-2g range (16384 LSB/g) -- whatever
// orientation the robot is actually in, sqrt(AcX^2+AcY^2+AcZ^2) should
// sit close to that 1g magnitude (real dynamic motion/vibration can
// push it off exact 1g, hence the wide band, not a tight one).
// Corrupted register bytes have no reason to coincidentally reproduce
// that magnitude, so this catches them without needing to guess a
// tilt-angle ceiling -- which would risk masking a genuine severe fall
// (a real fall still reads ~1g, just pointed a different way) instead
// of just rejecting noise.
//
// This project already has documented I2C noise (see
// Wire.setWireTimeout() in setup()) -- that protects against the bus
// HANGING, not against either kind of bad read riding on top of it.
// Callers must check the return value before trusting pitch/roll --
// they're left untouched on failure.
#define MPU_ACCEL_LSB_PER_G     16384.0 // power-on default +-2g range, ACCEL_CONFIG never written
#define MPU_ACCEL_MAG_MIN_G     0.5
#define MPU_ACCEL_MAG_MAX_G     2.0
bool readMPU6050(float &pitch, float &roll) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, 6, true) < 6) return false;

  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();

  float magG = sqrt((float)AcX * AcX + (float)AcY * AcY + (float)AcZ * AcZ) / MPU_ACCEL_LSB_PER_G;
  if (magG < MPU_ACCEL_MAG_MIN_G || magG > MPU_ACCEL_MAG_MAX_G) return false;

  pitch = atan2((float)AcX, sqrt((float)AcY * AcY + (float)AcZ * AcZ)) * 180.0 / PI;
  roll  = atan2((float)AcY, sqrt((float)AcX * AcX + (float)AcZ * AcZ)) * 180.0 / PI;
  return true;
}

// Reports whether the body is level, using the MPU6050 as ground
// truth rather than just trusting the leg kinematics. Pitch is
// front-to-back tilt -- exactly what a front/rear height mismatch
// (like the one setBodyHeight() was fixed to avoid) would show up as.
#define LEVEL_TOLERANCE_DEG 3.0

void checkLevel() {
  float pitch, roll;
  if (!readMPU6050(pitch, roll)) { // bad read -- see readMPU6050()'s comment
    Serial.println(F("IMU read failed -- try again."));
    return;
  }
  Serial.print(F("Pitch:")); Serial.print(pitch, 1);
  Serial.print(F("  Roll:")); Serial.print(roll, 1);
  if (fabs(pitch) <= LEVEL_TOLERANCE_DEG && fabs(roll) <= LEVEL_TOLERANCE_DEG) {
    Serial.println(F("  -> Level"));
  } else {
    Serial.println(F("  -> NOT level"));
  }
}

// ============================================================
// VL53L0X SETUP — sensor 2 booted first to avoid address clash
// ============================================================
void setupVL53L0X() {
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(100);

  // Sensor 2 first — reassign before sensor 1 appears on the bus
  digitalWrite(XSHUT_2, HIGH);
  delay(100);
  tof2.setBus(&Wire);
  tof2.setTimeout(500);
  if (tof2.init()) {
    tof2.setAddress(TOF2_ADDR);
    // "Long range" tuning: lowering the signal rate limit and
    // lengthening both VCSEL periods trades noise immunity for reach,
    // out to VL53L0X's realistic ~2m ceiling.
    tof2.setSignalRateLimit(0.1);
    tof2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tof2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tof2.setMeasurementTimingBudget(50000);
    tof2.startContinuous(100);
    tof2Active = true;
    Serial.println(F("Sensor 2 ready (0x52)."));
  } else {
    Serial.println(F("Sensor 2 not found — skipping."));
  }

  // Sensor 1 — safe to boot now, no address conflict
  digitalWrite(XSHUT_1, HIGH);
  delay(100);
  tof1.setBus(&Wire);
  tof1.setTimeout(500);
  if (tof1.init()) {
    tof1.setSignalRateLimit(0.1);
    tof1.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tof1.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tof1.setMeasurementTimingBudget(50000);
    tof1.startContinuous(100);
    tof1Active = true;
    Serial.println(F("Sensor 1 ready (0x29)."));
  } else {
    Serial.println(F("Sensor 1 not found — skipping."));
  }
}

// ============================================================
// POLL TOF SENSORS
// readRangeContinuousMillimeters() blocks internally until a new
// sample is ready, which would stall the servo/command loop while
// waiting. RESULT_INTERRUPT_STATUS is the same register the
// library's own blocking wait polls, so checking it ourselves first
// lets us only call readRangeContinuousMillimeters() once a sample
// is actually ready, keeping loop() responsive.
// ============================================================
bool tofDataReady(VL53L0X &s) {
  return (s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

void pollTofSensors() {
  if (tof1Active && tofDataReady(tof1)) {
    tof1_mm = tof1.readRangeContinuousMillimeters();
    tof1_ok = !tof1.timeoutOccurred() && tof1_mm > 0 && tof1_mm <= TOF_MAX_MM;
  }
  if (tof2Active && tofDataReady(tof2)) {
    tof2_mm = tof2.readRangeContinuousMillimeters();
    tof2_ok = !tof2.timeoutOccurred() && tof2_mm > 0 && tof2_mm <= TOF_MAX_MM;
  }
  pushTofFilterSample();
}

// ============================================================
// PRINT SENSOR VALUES
// ============================================================
void printSensors() {
  float pitch, roll;
  bool imuOk = readMPU6050(pitch, roll); // bad read -- see readMPU6050()'s comment

  if (imuOk) {
    Serial.print(F("Pitch:")); Serial.print(pitch, 1);
    Serial.print(F("  Roll:")); Serial.print(roll, 1);
  } else {
    Serial.print(F("Pitch:---  Roll:---"));
  }

  if (tof1Active) {
    if (tof1_ok) {
      Serial.print(F("  ToF1:")); Serial.print(tof1_mm); Serial.print(F("mm"));
    } else {
      Serial.print(F("  ToF1:---(raw=")); Serial.print(tof1_mm); Serial.print(F(")"));
    }
  } else {
    Serial.print(F("  ToF1:N/A"));
  }

  if (tof2Active) {
    if (tof2_ok) {
      Serial.print(F("  ToF2:")); Serial.print(tof2_mm); Serial.print(F("mm"));
    } else {
      Serial.print(F("  ToF2:---(raw=")); Serial.print(tof2_mm); Serial.print(F(")"));
    }
  } else {
    Serial.print(F("  ToF2:N/A"));
  }

  Serial.println();
}

// ============================================================
// NON-BLOCKING SERIAL COMMAND READER
// Works with any Serial Monitor line ending setting
// ============================================================
String cmdBuffer = "";

String readCommand() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      String cmd = cmdBuffer;
      cmdBuffer = "";
      cmd.trim();
      cmd.toLowerCase();
      if (cmd.length() > 0) return cmd;
    } else {
      cmdBuffer += c;
    }
  }
  return "";
}

// ============================================================
// COMMAND HANDLER
// ============================================================
void handleCommand(String input) {
  if (input == "start") {
    // Low/splayed stance that keeps the balance polygon satisfied
    // even with one front leg lifted (unlike the fully-extended
    // default), found by manual testing. knee_fl/knee_fr are left
    // at their calibrated straight position -- only the rear knees
    // needed bending for this stance.
    setHip(RL, 0);
    setHip(RR, 0);
    setKnee(RL, 70);
    setKnee(RR, 70);
    setHip(FL, 70);
    setHip(FR, 70);
    Serial.println(F("Start stance applied."));

  } else if (input == "angles") {
    // Prints every leg's CURRENT commanded hip/knee at once -- avoids
    // having to scroll back through a long jog session and reconstruct
    // "the final settled values" by hand (a real source of confusion
    // before this existed: a truncated log once left FL's actual final
    // angles misread as its very first jog value instead).
    const char* legNames[NUM_HIPS] = { "FL", "FR", "RL", "RR" };
    Serial.println(F("Current joint angles (hip/knee):"));
    for (int i = 0; i < NUM_HIPS; i++) {
      Serial.print(F("  ")); Serial.print(legNames[i]);
      Serial.print(F(": hip=")); Serial.print(hipPos[i]);
      Serial.print(F(" knee=")); Serial.println(kneePos[i]);
    }
    Serial.println(F("As #define lines:"));
    for (int i = 0; i < NUM_HIPS; i++) {
      Serial.print(F("#define PRECLIMB_HIP_"));  Serial.print(legNames[i]); Serial.print(F("   ")); Serial.println(hipPos[i]);
      Serial.print(F("#define PRECLIMB_KNEE_")); Serial.print(legNames[i]); Serial.print(F("  ")); Serial.println(kneePos[i]);
    }

  } else if (input == "rear_legs_shared_start") {
    commandClimbPose(REAR_LEGS_SHARED_START);
    Serial.println(F("Commanding REAR_LEGS_SHARED_START -- RL and RR moving to the same shared pose together. Wait for 'Climb pose reached.' (all four legs settled) before sending knee_rl 270 to start the lurch."));

  } else if (input == "new_stable_lift") {
    commandNewStableLift();
    Serial.println(F("Commanding NEW_STABLE_LIFT (FR/RL/RR only) -- NOT monitored for tilt, watch closely."));

  } else if (input == "sensors") {
    printSensors();

  } else if (input == "level") {
    checkLevel();

  } else if (input == "balance on") {
    balanceEnabled = true;
    Serial.println(F("Self-balancing enabled."));

  } else if (input == "balance off") {
    balanceEnabled = false;
    for (int i = 0; i < NUM_HIPS; i++) legHeightCorrection[i] = 0;
    setBodyHeight(lastCommandedHeight); // return to the uncorrected, uniform height
    Serial.println(F("Self-balancing disabled, corrections reset."));

  } else if (input == "help") {
    Serial.println();
    Serial.println(F("Commands: start | hip_fl/fr/rl/rr <angle> | knee_fl/fr/rl/rr <angle> | angles | stand <percent> | stand_sweep | lift_fl/fr/rl/rr | second_fr | raise_rear | raise_rear_stop | rear_prep | rear_wheel_lift_rl/rr | rear_wheel_lower | rear_wheel_stop | rear_legs_shared_start | new_stable_lift | lower | drive <speed -255..255> <duration_ms> | drive_to <speed> <target_mm> <timeout_ms> | drive_stop | turn_test <speed -255..255> <duration_ms> | level | balance on/off | sensors | help"));
    Serial.println();

  } else if (input == "stand_sweep") {
    if (startStepScan()) {
      Serial.println(F("Scanning 0->100%, will report ToF1 only on a large jump..."));
    } else {
      Serial.println(F("Cannot start scan (already scanning, or a stand move is already in progress)."));
    }

  }
  /* "stand" command removed by request -- kept here, commented out,
     in case it's needed again.
  else if (input == "stand") {
    if (startStandMove(1.0)) {
      Serial.println(F("Standing up..."));
    } else if (standProgress >= 1.0) {
      Serial.println(F("Already standing."));
    } else {
      Serial.println(F("Already moving."));
    }
  }
  */
  else if (input.startsWith("stand ")) {
    float pct = constrain(input.substring(6).toFloat(), 0.0, 100.0);
    if (startStandMove(pct / 100.0)) {
      Serial.print(F("Moving to ")); Serial.print(pct); Serial.println(F("% stand..."));
    } else {
      Serial.println(F("Already moving, or already at that percent."));
    }

  } else if (input == "lift_fl" || input == "lift_fr" || input == "lift_rl" || input == "lift_rr") {
    int legIdx = (input == "lift_fl") ? FL : (input == "lift_fr") ? FR : (input == "lift_rl") ? RL : RR;
    if (startLift(legIdx)) {
      Serial.println(F("Raising to a stable stance before lift..."));
    } else {
      Serial.println(F("Cannot start lift (already mid-sequence)."));
    }

  } else if (input == "lower") {
    if (startLower()) {
      Serial.println(F("Lowering leg..."));
    } else {
      Serial.println(F("No leg currently lifted."));
    }

  } else if (input == "second_fr") {
    // See startSecondLegOntoStep() -- untested stance, no verified
    // pose for "one front leg already on the step." Watch closely.
    if (startSecondLegOntoStep(FR)) {
      Serial.println(F("Placing FR onto the step next to the held leg -- UNTESTED stance, watch closely."));
    } else {
      Serial.println(F("Cannot start second-leg placement (first leg isn't down-and-holding, is already FR, or a safety abort is active -- send 'lower' first if so)."));
    }

  } else if (input == "raise_rear") {
    // See RAISE REAR's block comment above -- UNTESTED, watch closely.
    if (startRaiseRear()) {
      Serial.println(F("Raising rear toward level -- UNTESTED, watch extremely closely."));
    } else {
      Serial.println(F("Cannot start raise_rear (front leg(s) not down-and-holding, or already in progress)."));
    }

  } else if (input == "raise_rear_stop") {
    stopRaiseRear();
    Serial.println(F("Raise rear stopped."));

  } else if (input == "rear_prep") {
    if (startRearPrep()) {
      Serial.println(F("Rear prep: front knees, then front hips, then rear hips/knees..."));
    } else {
      Serial.println(F("Cannot start rear prep (already in progress, or front legs not down-and-holding)."));
    }

  } else if (input == "rear_wheel_lift_rl" || input == "rear_wheel_lift_rr") {
    // See REAR WHEEL LIFT's block comment above -- do NOT use lift_rl/
    // lift_rr for this, they reset the whole robot's pose and would
    // undo raise_rear. UNTESTED, no verified rear-leg pose exists for
    // this stance -- watch extremely closely.
    int legIdx = (input == "rear_wheel_lift_rl") ? RL : RR;
    if (startRearWheelLift(legIdx)) {
      Serial.println(F("Shifting weight before lifting the rear wheel -- UNTESTED, watch extremely closely."));
    } else {
      Serial.println(F("Cannot start rear wheel lift (raise_rear not idle, another lift already in progress, or the stability margin from this stance is too low)."));
    }

  } else if (input == "rear_wheel_lower") {
    if (startRearWheelLower()) {
      Serial.println(F("Lowering rear wheel..."));
    } else {
      Serial.println(F("No rear wheel currently lifted."));
    }

  } else if (input == "rear_wheel_stop") {
    abortRearWheelLift();
    Serial.println(F("Rear wheel lift stopped -- legs frozen where they are, stance NOT restored (send 'rear_wheel_lower' first if the leg is still up)."));

  } else if (input.startsWith("drive ")) {
    String rest = input.substring(6);
    int    sep  = rest.indexOf(' ');
    if (sep > 0) {
      int speed = rest.substring(0, sep).toInt();
      unsigned long durationMs = (unsigned long)rest.substring(sep + 1).toInt();
      startDrive(speed, durationMs);
      Serial.print(F("Driving at ")); Serial.print(speed);
      Serial.print(F(" for ")); Serial.print(durationMs); Serial.println(F("ms."));
    } else {
      Serial.println(F("Usage: drive <speed -255..255> <duration_ms>"));
    }

  } else if (input.startsWith("drive_to ")) {
    String rest = input.substring(9);
    int    sep1 = rest.indexOf(' ');
    int    sep2 = (sep1 > 0) ? rest.indexOf(' ', sep1 + 1) : -1;
    if (sep1 > 0 && sep2 > 0) {
      int speed = rest.substring(0, sep1).toInt();
      float targetMM = rest.substring(sep1 + 1, sep2).toFloat();
      unsigned long timeoutMs = (unsigned long)rest.substring(sep2 + 1).toInt();
      startDriveToTof(speed, targetMM, timeoutMs);
      Serial.print(F("Driving at ")); Serial.print(speed);
      Serial.print(F(" until ToF1 ")); Serial.print(speed > 0 ? "<= " : ">= "); Serial.print(targetMM, 0);
      Serial.print(F("mm, timeout ")); Serial.print(timeoutMs); Serial.println(F("ms."));
    } else {
      Serial.println(F("Usage: drive_to <speed -255..255> <target_mm> <timeout_ms>"));
    }

  } else if (input == "drive_stop") {
    stopWheels();
    squareState = SQUARE_IDLE; // also cancels an in-progress square-up
    turnTestActive = false; // also cancels an in-progress turn test
    if (raiseRearState != RAISE_REAR_IDLE) stopRaiseRear(); // also cancels an in-progress raise_rear
    Serial.println(F("Wheels stopped."));

  } else if (input.startsWith("turn_test ")) {
    String rest = input.substring(10);
    int    sep  = rest.indexOf(' ');
    if (sep > 0) {
      int speed = rest.substring(0, sep).toInt();
      unsigned long durationMs = (unsigned long)rest.substring(sep + 1).toInt();
      if (startTurnTest(speed, durationMs)) {
        Serial.println(F("Turn test running -- mark position/heading before AND after, then measure by hand."));
      } else {
        Serial.println(F("Cannot start turn test (a drive, square-up, or another turn test is already active)."));
      }
    } else {
      Serial.println(F("Usage: turn_test <speed -255..255> <duration_ms>"));
    }

  } else {
    int space = input.indexOf(' ');
    if (space > 0) {
      String name  = input.substring(0, space);
      int    angle = input.substring(space + 1).toInt();
      bool   found = false;
      for (int i = 0; i < NUM_HIPS; i++) {
        if (name == HIP_NAMES[i]) {
          setHip(i, angle);
          Serial.print(HIP_NAMES[i]); Serial.print(F(" -> ")); Serial.println(hipPos[i]);
          found = true;
          break;
        }
      }
      for (int i = 0; i < NUM_HIPS && !found; i++) {
        if (name == KNEE_NAMES[i]) {
          if (kneeInstalled[i]) {
            setKnee(i, angle);
            Serial.print(KNEE_NAMES[i]); Serial.print(F(" -> ")); Serial.println(kneePos[i]);
          } else {
            Serial.print(KNEE_NAMES[i]); Serial.println(F(" not installed yet."));
          }
          found = true;
        }
      }
      if (!found) Serial.println(F("Unknown command. Type 'help'."));
    } else {
      Serial.println(F("Unknown command. Type 'help'."));
    }
  }
}

// ============================================================
// WHEEL DRIVE
// All four wheels driven together (same direction/speed) via one
// L298N channel per wheel -- see the WHEEL_*_IN1/IN2/EN pin defines
// above. No wheel encoders exist anywhere in this codebase, so this
// is open-loop: a commanded speed for a commanded DURATION, timed by
// millis() (non-blocking, same pattern as the rest of this file), not
// distance. Whatever real distance a given speed/duration covers has
// to be found empirically on hardware (time a few runs at a fixed
// speed over a measured distance) before trusting driveForMs() to hit
// a specific standoff -- there's no closed-loop correction here.
// ============================================================
const int WHEEL_IN1_PINS[NUM_HIPS] = { WHEEL_FL_IN1, WHEEL_FR_IN1, WHEEL_RL_IN1, WHEEL_RR_IN1 };
const int WHEEL_IN2_PINS[NUM_HIPS] = { WHEEL_FL_IN2, WHEEL_FR_IN2, WHEEL_RL_IN2, WHEEL_RR_IN2 };
const int WHEEL_EN_PINS[NUM_HIPS]  = { WHEEL_FL_EN,  WHEEL_FR_EN,  WHEEL_RL_EN,  WHEEL_RR_EN  };

// Confirmed on hardware (WheelDriveTest sketch): the front and rear
// axles are mounted as mirror images of each other, so the same
// electrical signal spins FL/FR backward while RL/RR go forward.
// Reversing FL/FR here makes positive speed mean the same real-world
// direction on all four -- same fix as HIP_MIRROR[] for the hips.
const bool WHEEL_REVERSED[NUM_HIPS] = { true, true, false, false }; // FL, FR, RL, RR

// Sets one wheel's signed speed: positive = forward, negative =
// reverse, 0 = ACTIVE BRAKE (both IN pins low, EN driven high) --
// requested directly after a real hardware failure: a stationary
// wheel used to mean both IN pins low with EN also at 0%, which
// disables the H-bridge output entirely and lets the wheel coast
// freely. Since the chassis is one rigid frame, a wheel left at that
// old "0 speed" during a partial drive (e.g. second_fr's right-side-
// only reverse, FL+RL nominally left alone) wasn't actually anchored
// at all -- it just rolled along, dragged by whichever wheels WERE
// being driven. Confirmed on hardware: this let the pre-lift reverse
// pull FL straight off the step even though FL was never itself
// commanded to move. Both IN pins low with EN driven shorts the motor
// across the bridge's low-side switches instead, actively resisting
// rotation rather than floating.
void setOneWheel(int i, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed == 0) {
    digitalWrite(WHEEL_IN1_PINS[i], LOW);
    digitalWrite(WHEEL_IN2_PINS[i], LOW);
    analogWrite(WHEEL_EN_PINS[i], 255);
    return;
  }
  bool forward = speed > 0;
  bool reverse = speed < 0;
  if (WHEEL_REVERSED[i]) { bool t = forward; forward = reverse; reverse = t; }
  digitalWrite(WHEEL_IN1_PINS[i], forward);
  digitalWrite(WHEEL_IN2_PINS[i], reverse);
  analogWrite(WHEEL_EN_PINS[i], abs(speed));
}

// Sets all four wheels to the same signed speed -- see setOneWheel().
void setWheelSpeeds(int speed) {
  for (int i = 0; i < NUM_HIPS; i++) setOneWheel(i, speed);
}

// Independent left/right speeds for in-place turning -- FL+RL share
// one side, FR+RR the other (matches HIP_OFFSET_Y[]: FL/RL carry the
// same sign, FR/RR the other). Used by the square-up routine below.
void setWheelSpeedsLR(int leftSpeed, int rightSpeed) {
  setOneWheel(FL, leftSpeed);
  setOneWheel(RL, leftSpeed);
  setOneWheel(FR, rightSpeed);
  setOneWheel(RR, rightSpeed);
}

void stopWheels() {
  setWheelSpeeds(0);
  driveActive = false;
  driveTofTargetMM = -1;
  driveTofStartMM = -1;
}

// Starts driving at speed (-255..255) for durationMs, then auto-stops
// -- see updateDrive(), called from loop(). Cuts short and restarts
// the timer if a drive is already active (last command wins, same as
// re-issuing any other move in this file).
void startDrive(int speed, unsigned long durationMs) {
  if (squareState != SQUARE_IDLE) {
    Serial.println(F("Cannot drive: square-up is in progress."));
    return;
  }
  setWheelSpeeds(speed);
  driveActive = true;
  driveStopAtMs = millis() + durationMs;
  driveTofTargetMM = -1;
}

// Drives toward/away from whatever ToF1 sees, stopping as soon as its
// LIVE reading crosses targetMM, instead of guessing a speed-to-
// distance mapping with no encoders to correct it. speed>0 (forward)
// assumes ToF1's reading is DECREASING (closing in on the step);
// speed<0 (reverse) assumes it's INCREASING (backing away) -- matches
// ToF1's existing forward-facing mount used by the step scan. Only
// meaningful while ToF1 has a real line of sight to what you're
// measuring against (the step's face) -- this does NOT help driving
// forward once a leg is already resting on the step, a different
// problem with no ToF line of sight to use.
// timeoutMs is a hard safety fallback (same mechanism as the plain
// timed drive) in case the reading is invalid or never reaches the
// target -- always stops by then regardless of what ToF1 says.
//
// DRIVE_TOF_AWAY_JUMP_MM (see updateDrive()) catches a DIFFERENT
// failure than tof1_ok going false: confirmed on hardware that
// approaching a step can make the reading jump FARTHER away mid-drive
// while still reporting "valid" (tof1_ok stays true) -- the beam
// likely lands on something behind/past the step's near edge instead
// of the face itself once close enough. That reading never satisfies
// the stop condition (which needs it to keep DECREASING), so the
// wheels kept driving blind and hit the step for real -- confirmed:
// re-measured distance was 318mm right before the drive, but read
// 367mm (further away) once it stopped, well past the 320mm target,
// with the robot already against the step. A reading during a forward
// approach should only ever get smaller, never jump meaningfully
// larger -- treating a large jump the same as an invalid reading
// (stop immediately) catches this the same way the tof1_ok check
// already catches the sensor going fully blind.
#define DRIVE_TOF_AWAY_JUMP_MM 30.0
void startDriveToTof(int speed, float targetMM, unsigned long timeoutMs) {
  if (squareState != SQUARE_IDLE) {
    Serial.println(F("Cannot drive: square-up is in progress."));
    return;
  }
  setWheelSpeeds(speed);
  driveActive = true;
  driveStopAtMs = millis() + timeoutMs;
  driveTofTargetMM = targetMM;
  driveTofApproaching = (speed > 0);
  driveTofStartMM = tof1_ok ? (float)tof1_mm : -1;
}

void updateDrive() {
  if (!driveActive) return;
  if (driveTofTargetMM >= 0) {
    if (!tof1_ok) {
      // Confirmed on hardware: driving right up close to the step is
      // exactly where ToF1's reading is most likely to go invalid
      // (near/past its minimum reliable range) -- this used to fall
      // straight through to the 8-second hard timeout below with
      // nothing checking distance in the meantime, so the wheels kept
      // pushing into the step instead of stopping. Losing the ToF
      // signal on a targeted drive is itself a reason to stop -- safer
      // to stop a little early on a noise blip than to keep driving
      // blind toward an obstacle.
      Serial.println(F("Drive stopped: ToF1 reading lost mid-drive."));
      stopWheels();
      return;
    }
    // See DRIVE_TOF_AWAY_JUMP_MM's comment -- a "valid" reading that
    // jumps meaningfully farther away than where the drive started is
    // just as untrustworthy as tof1_ok going false, and confirmed on
    // hardware to otherwise drive straight through into the step
    // without ever satisfying the stop condition below.
    if (driveTofStartMM >= 0) {
      float awayFromStart = driveTofApproaching ? ((float)tof1_mm - driveTofStartMM) : (driveTofStartMM - (float)tof1_mm);
      if (awayFromStart > DRIVE_TOF_AWAY_JUMP_MM) {
        Serial.print(F("Drive stopped: ToF1 reading jumped "));
        Serial.print(awayFromStart, 0);
        Serial.println(F("mm further away mid-drive (likely seeing past the target) -- stopping as a precaution."));
        stopWheels();
        return;
      }
    }
    if (driveTofApproaching  && tof1_mm <= driveTofTargetMM) { stopWheels(); return; }
    if (!driveTofApproaching && tof1_mm >= driveTofTargetMM) { stopWheels(); return; }
  }
  if ((long)(millis() - driveStopAtMs) >= 0) stopWheels(); // safety fallback -- always wins eventually regardless of ToF state
}

// ============================================================
// RAISE REAR: with FL (and FR) already resting on the step, extend
// the rear legs to push the chassis up toward the step's height,
// driving the wheels forward a little after each increment so the
// rear doesn't get left behind as the body rises out from under it --
// otherwise the chassis would just pitch further nose-up without the
// rear ever catching up.
//
// RL/RR move hip AND knee together, interpolating from wherever they
// started toward HIP_START/KNEE_START -- straight down, full leg
// extension. An earlier version aimed at a computed PARTIAL target
// height (FL's actual foot depth + the measured step height, via
// computeRearJointsForHeight()) instead of full extension, on the
// theory that "front hip and rear hip end up at the same absolute
// height" was more precise than always maxing out. Reverted by
// request after hardware showed the opposite problem: RL/RR reached
// that computed target ("rear legs reached their target height") while
// the chassis was STILL several degrees off level, meaning the
// estimate undershot -- always driving to the genuine physical
// maximum (straight down) removes that ceiling entirely, so the rear
// can rise as far as it actually can rather than stopping short at a
// possibly-wrong estimate.
// NOT knee alone (an earlier version before that): confirmed by
// request that extending only the knee doesn't actually finish
// "standing straight down" -- with the hip left parked near
// PRECLIMB_HIP_RL/RR (close to HIP_MIN) the whole leg was extending
// along whatever direction that low hip angle already pointed, not
// truly vertical. Moving both joints toward HIP_START/KNEE_START
// together is what "stood all the way up, not allowed past straight
// down" actually means geometrically.
//
// Stopping condition is still LEVEL, not a fixed number of degrees:
// the front is already up at step height and the rear is still on the
// ground, so pitch starts off-level and should move toward 0 as the
// rear rises to match -- once |pitch| is within LEVEL_TOLERANCE_DEG,
// the whole chassis is roughly at step height and this is done. This
// IMU check stays the authoritative stop condition (the height target
// above only changes what the motion aims at, not what decides when to
// stop) -- checking the MAGNITUDE (not a specific signed direction)
// means this doesn't depend on knowing whether "front high" reads as
// positive or negative pitch in this code, it just watches for pitch
// closing in on zero either way.
//
// The front legs move too, using the SAME progress fraction as RL/RR
// (not a separate slow/capped schedule -- an earlier version did that,
// "lower the front legs but not too much"). Requested directly: hip
// swings toward HIP_MAX (as far forward as the joint goes) while knee
// folds toward KNEE_MIN (up and tucked underneath the chassis as far
// as it goes), the same fold-toward-the-extreme idea already confirmed
// working for SECOND_FR_SAFE_KNEE/LIFT_SAFE_KNEE_FL -- the direction is
// the point, not a specific partial angle. UNTESTED at this new
// behaviour: FL/FR are already resting on the step, so a large hip/
// knee swing here risks dragging or lifting a wheel off the step
// surface if it moves too fast relative to the rear's own rise. Watch
// extremely closely, ready to stop (drive_stop / power off) if a front
// wheel starts to slip.
//
// The forward drive is now a real measured distance, not a blind
// guess: by request, the total forward travel for this whole maneuver
// is RAISE_REAR_TOTAL_FORWARD_MM (260mm -- the front wheels sit
// ~26cm ahead of the chassis once placed on the step, so driving
// roughly that far brings the majority of the chassis over the step
// edge). WHEEL_MM_PER_MS_AT_120 is the real measured speed from
// WheelCalibration/WheelCalibration.ino (FL lifted, other three wheels
// loaded and driving, at RAISE_REAR_DRIVE_SPEED): observed ~1.5 wheel
// rotations in 4000ms, i.e. 1.5 * 282.7mm circumference / 4000ms. The
// total is split evenly across RAISE_REAR_STEPS increments (still the
// same interleaved step-then-drive-then-settle structure already
// confirmed to work on hardware) rather than repeating a flat blind
// guess on every step regardless of how much distance that actually
// added up to.
//
// UNTESTED ON HARDWARE (the height-target and real-distance changes,
// on top of the maneuver already being new territory) -- this
// deliberately runs the chassis through a large, sustained pitch
// change while driving, with both ends of the robot at different
// heights the whole time. Watch extremely closely and be ready to
// catch/support the robot.
// ============================================================
#define RAISE_REAR_STEPS               25   // number of increments from wherever RL/RR started to HIP_START/KNEE_START -- fine and incremental, same idea as LIFT_DESCEND_STEPS
#define RAISE_REAR_DRIVE_SPEED         120
#define WHEEL_MM_PER_MS_AT_120         0.106 // measured via WheelCalibration.ino: ~1.5 rotations of FL's wheel (90mm dia, 282.7mm circumference) in 4000ms while driving loaded at speed 120
#define RAISE_REAR_TOTAL_FORWARD_MM    260.0 // measured: front wheels sit ~26cm ahead of the chassis once placed on the step -- by the time raise_rear finishes, most of the chassis should be over the step edge
#define RAISE_REAR_DRIVE_MS            98   // RAISE_REAR_TOTAL_FORWARD_MM / WHEEL_MM_PER_MS_AT_120 / RAISE_REAR_STEPS = 260 / 0.106 / 25 ~= 98ms -- replaces the old blind flat 150ms-per-step guess
#define RAISE_REAR_SETTLE_MS           400  // dwell after the drive pulse before trusting the IMU
#define RAISE_REAR_ROLL_ABORT_DEG      12.0 // roll isn't the axis being intentionally changed here -- tighter than the general LIFT_TILT_ABORT_DEG, any real roll means something is going wrong sideways
#define RAISE_REAR_MAX_STEPS           80   // hard fallback in case level is never reached (e.g. wheel slip, bad height estimate) -- deliberately well past RAISE_REAR_STEPS, covers extra settle-and-check cycles after RL/RR already reached their target height

// RaiseRearState/raiseRearState are declared near the top of the file
// (right after driveActive) -- handleCommand()'s drive_stop branch
// needs them and comes before this section. See that comment.
int   raiseRearStepCount = 0;
int   raiseRearRLHipStart = 0, raiseRearRLKneeStart = 0;
int   raiseRearRRHipStart = 0, raiseRearRRKneeStart = 0;
int   raiseRearFrontKneeStartFL = 0, raiseRearFrontKneeStartFR = 0;
int   raiseRearFrontHipStartFL = 0, raiseRearFrontHipStartFR = 0;
unsigned long raiseRearSettleStartMs = 0;

// RL/RR always target straight-down, full extension -- see the block
// comment above for why (an earlier computed-partial-height version
// undershot on hardware).
int raiseRearHipTargetRL = 0, raiseRearKneeTargetRL = 0;
int raiseRearHipTargetRR = 0, raiseRearKneeTargetRR = 0;

bool startRaiseRear() {
  if (liftState != LIFT_HOLDING) return false; // expects FL (and FR, via second_fr) already down and holding on the step
  if (raiseRearState != RAISE_REAR_IDLE) return false;
  raiseRearStepCount = 0;
  raiseRearRLHipStart = hipPos[RL];   raiseRearRLKneeStart = kneePos[RL];
  raiseRearRRHipStart = hipPos[RR];   raiseRearRRKneeStart = kneePos[RR];
  // Only captured ONCE per climb (see raiseRearFrontKneeCaptured's
  // comment) -- re-invoking raise_rear after it stops short must not
  // re-anchor this to the already-advanced current position.
  if (!raiseRearFrontKneeCaptured) {
    raiseRearFrontKneeStartFL = kneePos[FL];
    raiseRearFrontKneeStartFR = kneePos[FR];
    raiseRearFrontHipStartFL  = hipPos[FL];
    raiseRearFrontHipStartFR  = hipPos[FR];
    raiseRearFrontKneeCaptured = true;
  }

  raiseRearHipTargetRL = HIP_START[RL]; raiseRearKneeTargetRL = KNEE_START[RL];
  raiseRearHipTargetRR = HIP_START[RR]; raiseRearKneeTargetRR = KNEE_START[RR];
  Serial.println(F("Raise rear target: RL/RR straight down (HIP_START/KNEE_START, full extension)."));

  moveSpeedScale = LIFT_MOVE_SPEED_SCALE; // careful, slow motion -- matches the rest of the climb sequence
  raiseRearState = RAISE_REAR_STEPPING;
  return true;
}

void stopRaiseRear() {
  raiseRearState = RAISE_REAR_IDLE;
  moveSpeedScale = 1.0;
  stopWheels();
}

void updateRaiseRear() {
  if (raiseRearState == RAISE_REAR_IDLE) return;

  if (raiseRearState == RAISE_REAR_STEPPING) {
    // Shared progress fraction so hip and knee both reach HIP_START/
    // KNEE_START at the SAME step, regardless of how different their
    // individual travel distances are -- clamped to 1.0 so continuing
    // to call this after reaching full stand just holds there, not
    // overshoots.
    float t = min(1.0, (float)(raiseRearStepCount + 1) / (float)RAISE_REAR_STEPS);
    int newHipRL  = raiseRearRLHipStart  + (int)round((raiseRearHipTargetRL  - raiseRearRLHipStart)  * t);
    int newKneeRL = raiseRearRLKneeStart + (int)round((raiseRearKneeTargetRL - raiseRearRLKneeStart) * t);
    int newHipRR  = raiseRearRRHipStart  + (int)round((raiseRearHipTargetRR  - raiseRearRRHipStart)  * t);
    int newKneeRR = raiseRearRRKneeStart + (int)round((raiseRearKneeTargetRR - raiseRearRRKneeStart) * t);
    setHip(RL, newHipRL);   setKnee(RL, newKneeRL);
    setHip(RR, newHipRR);   setKnee(RR, newKneeRR);

    // Front legs move too, using the SAME progress fraction t -- hip
    // toward HIP_MAX (forward), knee toward KNEE_MIN (tucked under the
    // chassis). See the block comment above for the reasoning and the
    // UNTESTED warning.
    int newHipFL  = raiseRearFrontHipStartFL  + (int)round((HIP_MAX[FL]  - raiseRearFrontHipStartFL)  * t);
    int newKneeFL = raiseRearFrontKneeStartFL + (int)round((KNEE_MIN[FL] - raiseRearFrontKneeStartFL) * t);
    int newHipFR  = raiseRearFrontHipStartFR  + (int)round((HIP_MAX[FR]  - raiseRearFrontHipStartFR)  * t);
    int newKneeFR = raiseRearFrontKneeStartFR + (int)round((KNEE_MIN[FR] - raiseRearFrontKneeStartFR) * t);
    setHip(FL, newHipFL);   setKnee(FL, newKneeFL);
    setHip(FR, newHipFR);   setKnee(FR, newKneeFR);

    raiseRearStepCount++;
    raiseRearState = RAISE_REAR_DRIVING;
    return;
  }

  if (raiseRearState == RAISE_REAR_DRIVING) {
    bool allDone = true;
    for (int i = 0; i < NUM_HIPS; i++) allDone = allDone && legMoveDone(i);
    if (!allDone) return;
    // Nudge forward to keep the rear from falling behind as the body
    // rises -- see the block comment above. Plain timed drive, not
    // ToF-targeted: at this range ToF1 is likely already too close to
    // the step (or looking past it) to be a reliable target, same
    // unreliability the approach-drive fixes were built around.
    startDrive(RAISE_REAR_DRIVE_SPEED, RAISE_REAR_DRIVE_MS);
    raiseRearSettleStartMs = millis();
    raiseRearState = RAISE_REAR_SETTLING;
    return;
  }

  if (raiseRearState == RAISE_REAR_SETTLING) {
    if (driveActive) return; // still driving forward
    if (millis() - raiseRearSettleStartMs < RAISE_REAR_SETTLE_MS) return; // let the chassis physically settle before trusting the IMU

    float pitch, roll;
    if (!readMPU6050(pitch, roll)) return; // bad read -- see readMPU6050()'s comment; retry next tick rather than acting on garbage

    Serial.print(F("Raise rear step ")); Serial.print(raiseRearStepCount);
    Serial.print(F(": pitch=")); Serial.print(pitch, 1);
    Serial.print(F(" roll=")); Serial.println(roll, 1);

    if (fabs(roll) > RAISE_REAR_ROLL_ABORT_DEG) {
      Serial.println(F("Raise rear ABORTED: roll exceeded safety threshold -- freezing where it is. Check the robot before continuing."));
      stopRaiseRear();
      return;
    }

    if (fabs(pitch) <= LEVEL_TOLERANCE_DEG) {
      Serial.println(F("Raise rear complete -- chassis level, rear should now be near step height."));
      stopRaiseRear();
      return;
    }

    if (raiseRearStepCount >= RAISE_REAR_MAX_STEPS) {
      Serial.println(F("Raise rear stopped: reached the step limit without leveling out -- check the rear legs' reach."));
      stopRaiseRear();
      return;
    }

    if (hipPos[RL] >= raiseRearHipTargetRL && kneePos[RL] >= raiseRearKneeTargetRL &&
        hipPos[RR] >= raiseRearHipTargetRR && kneePos[RR] >= raiseRearKneeTargetRR) {
      Serial.println(F("Raise rear stopped: rear legs reached their target height but chassis still not level -- may need to drive further forward, or the step height estimate is off."));
      stopRaiseRear();
      return;
    }

    raiseRearState = RAISE_REAR_STEPPING;
  }
}

// ============================================================
// REAR PREP: preparation stance for lifting a rear leg, requested
// directly with these exact hand-jogged angles. Staged in three
// steps, not one combined move -- front knees first, then front hips,
// then rear hips/knees together, checking each stage finished before
// starting the next (same incremental philosophy as every other new
// maneuver in this file).
// ============================================================
#define REAR_PREP_HIP_FL   90
#define REAR_PREP_KNEE_FL  0
#define REAR_PREP_HIP_FR   88
#define REAR_PREP_KNEE_FR  0
#define REAR_PREP_HIP_RL   40
#define REAR_PREP_KNEE_RL  30
#define REAR_PREP_HIP_RR   40
#define REAR_PREP_KNEE_RR  50

enum RearPrepState { REAR_PREP_IDLE, REAR_PREP_FRONT_KNEE, REAR_PREP_FRONT_HIP, REAR_PREP_REAR };
RearPrepState rearPrepState = REAR_PREP_IDLE;

bool startRearPrep() {
  if (rearPrepState != REAR_PREP_IDLE) return false;
  if (liftState != LIFT_HOLDING) return false; // expects both front legs already down and holding on the step
  moveSpeedScale = LIFT_MOVE_SPEED_SCALE; // careful, slow motion -- matches the rest of the climb sequence
  setKnee(FL, REAR_PREP_KNEE_FL);
  setKnee(FR, REAR_PREP_KNEE_FR);
  rearPrepState = REAR_PREP_FRONT_KNEE;
  return true;
}

void updateRearPrep() {
  if (rearPrepState == REAR_PREP_IDLE) return;

  if (rearPrepState == REAR_PREP_FRONT_KNEE) {
    if (!legMoveDone(FL) || !legMoveDone(FR)) return; // still moving the front knees
    setHip(FL, REAR_PREP_HIP_FL);
    setHip(FR, REAR_PREP_HIP_FR);
    rearPrepState = REAR_PREP_FRONT_HIP;
    return;
  }

  if (rearPrepState == REAR_PREP_FRONT_HIP) {
    if (!legMoveDone(FL) || !legMoveDone(FR)) return; // still moving the front hips
    setHip(RL, REAR_PREP_HIP_RL);
    setHip(RR, REAR_PREP_HIP_RR);
    setKnee(RL, REAR_PREP_KNEE_RL);
    setKnee(RR, REAR_PREP_KNEE_RR);
    rearPrepState = REAR_PREP_REAR;
    return;
  }

  if (rearPrepState == REAR_PREP_REAR) {
    if (!legMoveDone(RL) || !legMoveDone(RR)) return; // still moving the rear hips/knees
    Serial.println(F("Rear prep complete."));
    moveSpeedScale = 1.0;
    rearPrepState = REAR_PREP_IDLE;
  }
}

// ============================================================
// REAR WHEEL LIFT: once raise_rear finishes (chassis level, front on
// the step, rear extended to match), lift ONE rear wheel (RL or RR)
// clear of the ground as the next step toward getting it onto the
// step too.
//
// Deliberately NOT built on startLiftSequence()/updateLiftSequence()
// (lift_fl/fr/rl/rr, step_fl/fr/rl/rr) -- that machinery always begins
// with startStandMove() (LIFT_RAISING), which drives every joint
// through the CROUCH_LOW<->HIP_START/KNEE_START stand-progress
// interpolation regardless of the robot's actual current pose, and for
// FL specifically calls createStablePlatform() (snaps all four legs to
// the hardcoded PRECLIMB_* angles). Both would immediately undo
// raise_rear's work: FL/FR would leave the step and RL/RR would drop
// out of the height-matched stance raise_rear just achieved. Confirmed
// by reading through that whole sequence, not assumed -- this is why
// lift_rl/lift_rr are NOT safe to send after raise_rear, even though
// they exist and target the right legs.
//
// This sequence instead treats WHATEVER POSE THE ROBOT IS CURRENTLY IN
// as the baseline and only touches the leg being lifted plus a weight
// shift on the other three -- reusing the same generic
// stabilityMargin()/findBestStabilityShift()/setFoot() helpers the
// original sequence's own non-FL branch already uses, just without any
// of the FL-specific resets.
//
// Answers both things asked for directly: "how high we can lift the
// chassis" is whatever margin findBestStabilityShift() finds for the
// remaining 3-leg support triangle from THIS stance (rejected below
// MIN_STABILITY_MARGIN_MM, the same safety floor the front-leg
// sequence uses) -- it's a stability search, not a fixed number, the
// same as it already is for any other leg. "How high to lift it" is
// REAR_WHEEL_LIFT_MM, the vertical clearance the foot rises once
// tucked.
//
// This is a PLAIN LIFT-AND-HOLD only -- clear of the ground, not yet a
// full reach onto the step. Getting it actually onto the step is a
// deliberate follow-up once this simpler piece is confirmed on
// hardware, the same incremental-trust approach as every other new
// maneuver in this file (raise_rear itself was built the same way).
//
// UNTESTED ON HARDWARE, more so than most here: no rear-leg pose data
// exists for this stance at all (unlike FL's extensively hand-tuned
// PRECLIMB/LIFT_SAFE_KNEE/LIFT_LIFTED_HIP constants -- those took many
// rounds of real testing to arrive at). This uses solveLegIK()'s
// general solve directly for RL/RR, which the rest of this file has
// repeatedly found unreliable at LARGE angles -- but the starting pose
// here (near vertical, close to HIP_START/KNEE_START, the small-angle
// regime the IK derivation actually assumes) is a different, more
// favorable situation than the earlier documented failures (which
// happened from PRECLIMB_HIP_RL/RR's far-off-vertical, near-HIP_MIN
// starting angles) -- a reasoned starting point, not a verified one.
// Watch extremely closely and be ready to catch/support the robot.
// ============================================================
#define REAR_WHEEL_LIFT_MM       30.0 // vertical clearance once tucked -- matches LEG_LIFT_MM's own reasoning, conservative, not a step reach yet
#define REAR_LIFT_SETTLE_MS      3000 // same dwell as LIFT_SETTLE_DWELL_MS -- let the shift physically settle before trusting the IMU
#define REAR_LIFT_TILT_LIMIT_DEG 8.0  // same as LIFT_PRELIFT_TILT_LIMIT_DEG -- commit gate before actually lifting

enum RearLiftState { REAR_LIFT_IDLE, REAR_LIFT_SHIFTING, REAR_LIFT_SETTLING, REAR_LIFT_RAISING, REAR_LIFT_HOLDING, REAR_LIFT_LOWERING, REAR_LIFT_UNSHIFTING };
RearLiftState rearLiftState = REAR_LIFT_IDLE;
int   rearLiftLegIdx = -1;
int   rearLiftStanceIdx[3];
float rearLiftStanceX[3], rearLiftStanceY[3]; // stance-leg foot positions before the shift, to restore on lower
float rearLiftOrigX = 0, rearLiftOrigY = 0;   // the lifted leg's own foot position before the shift, to restore on lower
unsigned long rearLiftSettleStartMs = 0;

bool startRearWheelLift(int legToLift) {
  if (legToLift != RL && legToLift != RR) return false;
  if (rearLiftState != REAR_LIFT_IDLE) return false;
  if (liftState != LIFT_IDLE) return false;       // don't fight the other (FL/FR) lift machinery if it's somehow active
  if (raiseRearState != RAISE_REAR_IDLE) return false; // let raise_rear finish first

  rearLiftLegIdx = legToLift;
  int n = 0;
  for (int i = 0; i < NUM_HIPS; i++) {
    if (i == legToLift) continue;
    rearLiftStanceIdx[n] = i;
    n++;
  }
  legForwardKinematics(legToLift, rearLiftOrigX, rearLiftOrigY);
  for (int k = 0; k < 3; k++) legForwardKinematics(rearLiftStanceIdx[k], rearLiftStanceX[k], rearLiftStanceY[k]);

  float bx[3], by[3];
  for (int k = 0; k < 3; k++) footBodyPosition(rearLiftStanceIdx[k], bx[k], by[k]);
  float bestShift, bestMargin;
  findBestStabilityShift(bx, by, rearLiftStanceX, rearLiftStanceY, bestShift, bestMargin);
  if (bestMargin < MIN_STABILITY_MARGIN_MM) {
    Serial.print(F("Rear wheel lift rejected: best achievable stability margin is "));
    Serial.print(bestMargin, 0);
    Serial.print(F("mm, below the ")); Serial.print(MIN_STABILITY_MARGIN_MM, 0);
    Serial.println(F("mm safety floor -- not attempting from this stance."));
    return false;
  }

  moveSpeedScale = LIFT_MOVE_SPEED_SCALE;
  for (int k = 0; k < 3; k++) {
    int i = rearLiftStanceIdx[k];
    if (!setFoot(i, rearLiftStanceX[k] - bestShift, rearLiftStanceY[k])) {
      Serial.println(F("Rear wheel lift aborted: weight-shift target unreachable."));
      moveSpeedScale = 1.0;
      return false;
    }
  }
  rearLiftState = REAR_LIFT_SHIFTING;
  return true;
}

// Freezes every leg this sequence touched exactly where it currently
// is (same freezeLeg() pattern checkLiftTiltSafety() uses) before
// resetting state -- an immediate stop, not "let whatever move is in
// flight finish".
void abortRearWheelLift() {
  if (rearLiftLegIdx != -1) {
    freezeLeg(rearLiftLegIdx);
    for (int k = 0; k < 3; k++) freezeLeg(rearLiftStanceIdx[k]);
  }
  rearLiftState = REAR_LIFT_IDLE;
  rearLiftLegIdx = -1;
  moveSpeedScale = 1.0;
}

bool startRearWheelLower() {
  if (rearLiftState != REAR_LIFT_HOLDING) return false;
  if (!setFoot(rearLiftLegIdx, rearLiftOrigX, rearLiftOrigY)) return false;
  rearLiftState = REAR_LIFT_LOWERING;
  return true;
}

void updateRearWheelLift() {
  if (rearLiftState == REAR_LIFT_IDLE) return;

  if (rearLiftState == REAR_LIFT_SHIFTING) {
    for (int k = 0; k < 3; k++) if (!legMoveDone(rearLiftStanceIdx[k])) return;
    rearLiftSettleStartMs = millis();
    rearLiftState = REAR_LIFT_SETTLING;

  } else if (rearLiftState == REAR_LIFT_SETTLING) {
    if (millis() - rearLiftSettleStartMs < REAR_LIFT_SETTLE_MS) return;

    float ax, ay, bx2, by2, cx, cy;
    footBodyPosition(rearLiftStanceIdx[0], ax, ay);
    footBodyPosition(rearLiftStanceIdx[1], bx2, by2);
    footBodyPosition(rearLiftStanceIdx[2], cx, cy);
    float settledMargin = stabilityMargin(0, 0, ax, ay, bx2, by2, cx, cy);
    if (settledMargin < MIN_STABILITY_MARGIN_MM) {
      Serial.print(F("Rear wheel lift aborted: settled stability margin is "));
      Serial.print(settledMargin, 0);
      Serial.println(F("mm after shifting -- below the safety floor, not lifting."));
      abortRearWheelLift();
      return;
    }
    float pitch, roll;
    if (!readMPU6050(pitch, roll)) return; // bad read -- retry next tick rather than acting on garbage
    if (fabs(pitch) > REAR_LIFT_TILT_LIMIT_DEG || fabs(roll) > REAR_LIFT_TILT_LIMIT_DEG) {
      Serial.print(F("Rear wheel lift aborted: body tilt pitch="));
      Serial.print(pitch, 1); Serial.print(F(" roll=")); Serial.print(roll, 1);
      Serial.println(F(" already exceeds the pre-lift check -- not safe to lift from this stance."));
      abortRearWheelLift();
      return;
    }

    if (!setFoot(rearLiftLegIdx, rearLiftOrigX, rearLiftOrigY - REAR_WHEEL_LIFT_MM)) {
      Serial.println(F("Rear wheel lift aborted: lift target unreachable."));
      abortRearWheelLift();
      return;
    }
    rearLiftState = REAR_LIFT_RAISING;

  } else if (rearLiftState == REAR_LIFT_RAISING) {
    if (!legMoveDone(rearLiftLegIdx)) return;
    moveSpeedScale = 1.0;
    rearLiftState = REAR_LIFT_HOLDING;
    Serial.print(F("Rear wheel ")); Serial.print(rearLiftLegIdx == RL ? "RL" : "RR");
    Serial.println(F(" lifted and holding. Send 'rear_wheel_lower' to set it back down."));

  } else if (rearLiftState == REAR_LIFT_LOWERING) {
    if (!legMoveDone(rearLiftLegIdx)) return;
    for (int k = 0; k < 3; k++) {
      int i = rearLiftStanceIdx[k];
      setFoot(i, rearLiftStanceX[k], rearLiftStanceY[k]);
    }
    rearLiftState = REAR_LIFT_UNSHIFTING;

  } else if (rearLiftState == REAR_LIFT_UNSHIFTING) {
    for (int k = 0; k < 3; k++) if (!legMoveDone(rearLiftStanceIdx[k])) return;
    Serial.println(F("Rear wheel lowered, stance restored."));
    abortRearWheelLift();
  }
}

// ============================================================
// SQUARE-UP: turn in place until ToF1 and ToF2 agree, meaning the
// robot is perpendicular to whatever flat face is ahead (the step).
// ToF1 sits toward the left, ToF2 toward the right (both centre-
// mounted, forward-facing) -- if the robot is yawed relative to the
// step's face, one beam travels a shorter path to it than the other;
// square is where both read the same.
//
// Turn direction confirmed by hand on real hardware -- see
// applySquareTurn(): ToF1 lower than ToF2 means reverse the left
// wheels/forward the right wheels, opposite the other way. An earlier
// version guessed a direction once and self-corrected by comparing
// consecutive readings to see if the gap got better or worse, but
// that comparison is itself noise-prone at this small a signal --
// confirmed on hardware to look like it was moving the robot
// randomly. Direction is now recomputed fresh from the current
// reading every pulse instead.
//
// UNTESTED ON HARDWARE: turn speed/pulse/settle timing below are
// starting guesses, not measured. Watch the first run closely and be
// ready to send 'drive_stop' if it doesn't behave.
//
// TOLERANCE: the two sensors are mounted right next to each other, so
// the signal is small -- confirmed on hardware at 3.0 (~10mm sensor
// separation * sin(45deg) matches almost exactly). At more realistic
// small misalignments (10-20deg) the expected signal is only 2-3mm,
// so a 10mm tolerance (the original guess) would call the robot
// "square" at almost any angle and never correct anything. Lowered
// well below the smallest signal worth reacting to; if this turns out
// tighter than the sensors' actual noise floor (causing it to never
// settle/oscillate), loosen it back up based on what's actually seen.
// The two ToF sensors do NOT read the same distance even at true
// square -- confirmed by hand with the robot squared up by an
// independent method (tape measure to symmetric points, not the
// sensors themselves), then sampled 18 times in place: ToF1 averaged
// ~293.8mm, ToF2 ~315.9mm -> offset ~-22mm. (A single first sample,
// 297/312, gave -15mm -- noisy enough on its own to be off by 7mm;
// see TOF_FILTER_N below for why this needed averaging at all.)
// Comparing raw tof1-tof2 against 0 was targeting the wrong angle by
// this whole offset, which is exactly why 'square' was overshooting.
// All comparisons below now target this offset instead of 0.
//
// Re-confirmed a THIRD time against the actual step (squared up with
// a box held flat against both hips, 25 samples): ToF1 ~246.9mm, ToF2
// ~267.8mm -> offset ~-21mm, matching the wall/corner calibration
// above within noise. This offset is a property of the two sensors
// themselves, not the target surface -- ruling out "wrong offset for
// this surface" as the cause of 'square' turning away from the step
// even when already looking straight at it. The real cause was the
// fixed-size turn pulse overcorrecting marginal errors -- see
// SQUARE_TURN_MS_PER_MM below.
#define SQUARE_DIFF_OFFSET_MM -22.0
// Individual sensor noise measured from that same 18-sample burst:
// ToF1 std dev ~2.9mm, ToF2 ~2.3mm. Since diff = tof1-tof2 combines
// both, its noise is larger still (~4mm) -- bigger than the 3mm
// tolerance below, using single instantaneous readings. Averaging
// TOF_FILTER_N samples first (see tof1Filtered()/tof2Filtered())
// shrinks that noise by roughly sqrt(N) before comparing.
#define SQUARE_TOLERANCE_MM  3.0
#define SQUARE_TURN_SPEED    150
// SQUARE_TURN_PULSE_MS is now a CEILING, not a fixed size every pulse
// -- see applySquareTurn()/SQUARE_TURN_MS_PER_MM below. Was raised
// 150->400 as a fixed value first (confirmed on hardware the original
// pulse was too short to produce a visible/meaningful turn per step,
// also confounded by a wire that had fallen out of one motor driver
// at the time). Kept as the cap for genuinely large errors.
#define SQUARE_TURN_PULSE_MS 400
// Confirmed on hardware: a fixed-size pulse for EVERY correction,
// however small the error, massively overcorrects a marginal/noise-
// driven trigger (a reading just past SQUARE_TOLERANCE_MM by only a
// couple mm) -- this was turning the robot AWAY from true square even
// when it was already looking directly at the step, because the same
// full 400ms/150-speed pulse fired regardless of whether the error was
// 4mm or 40mm. Pulse duration now scales with the error itself.
#define SQUARE_TURN_MS_PER_MM   15
#define SQUARE_TURN_PULSE_MIN_MS 60 // floor -- shorter than this may not move the robot at all
#define SQUARE_SETTLE_MS     300
#define SQUARE_MAX_ATTEMPTS  20

// Takes adjustedDiff = (tof1 - tof2) - SQUARE_DIFF_OFFSET_MM, i.e.
// already corrected for the two sensors' real offset at true square --
// so adjustedDiff==0 means square, not raw tof1==tof2. Confirmed by
// hand which way this should turn: adjustedDiff<0 -> reverse the left
// wheels, forward the right wheels. Opposite when adjustedDiff>0.
// Computed fresh from the CURRENT reading every pulse, not guessed
// once and corrected from a noisy better/worse comparison -- that
// self-correcting version was confirmed on hardware to look like it
// was moving the robot randomly, since comparing two noisy consecutive
// readings to infer "did it get better" is itself noise-prone when
// the signal is this small (see SQUARE_TOLERANCE_MM's comment).
//
// Also sets squareCurrentPulseMs proportional to the error size
// (SQUARE_TURN_MS_PER_MM per mm, clamped to
// [SQUARE_TURN_PULSE_MIN_MS, SQUARE_TURN_PULSE_MS]) -- see this
// function's header comment above for why a fixed pulse regardless of
// error size was the actual bug behind overshooting past true square.
void applySquareTurn(float adjustedDiff) {
  int sign = (adjustedDiff < 0) ? -1 : 1;
  setWheelSpeedsLR(SQUARE_TURN_SPEED * sign, -SQUARE_TURN_SPEED * sign);
  unsigned long pulseMs = (unsigned long)(fabs(adjustedDiff) * SQUARE_TURN_MS_PER_MM);
  squareCurrentPulseMs = constrain(pulseMs, SQUARE_TURN_PULSE_MIN_MS, SQUARE_TURN_PULSE_MS);
}

// ============================================================
// TURN TEST: fires exactly ONE differential-drive pulse, then stops --
// no ToF involved at all. Exists to answer a question the square-up
// data alone couldn't: is a turn pulse actually rotating the robot in
// place, or also translating it (sliding forward/back/sideways)? Mark
// the floor before and after (tape at the wheelbase corners, or
// against a wall) and measure by hand -- if there's real sideways/
// forward drift on top of the rotation, that explains square-up not
// converging cleanly (both ToF readings climbing together instead of
// just their difference changing), and no amount of ToF-side tuning
// fixes a chassis that isn't doing a clean pivot turn.
// ============================================================
bool startTurnTest(int speed, unsigned long durationMs) {
  return startTurnTestLR(speed, -speed, durationMs);
}

// Same as startTurnTest() but with independent left/right speeds, so a
// pulse can drive one side harder than the other instead of a
// symmetric pivot -- see second_fr's LIFT_FR_TURN1/TURN2 pulses.
bool startTurnTestLR(int leftSpeed, int rightSpeed, unsigned long durationMs) {
  if (squareState != SQUARE_IDLE || driveActive || turnTestActive) return false;
  setWheelSpeedsLR(leftSpeed, rightSpeed);
  turnTestActive = true;
  turnTestStopAtMs = millis() + durationMs;
  return true;
}

void updateTurnTest() {
  if (!turnTestActive) return;
  if ((long)(millis() - turnTestStopAtMs) >= 0) {
    setWheelSpeedsLR(0, 0);
    turnTestActive = false;
    Serial.println(F("Turn test complete -- measure the actual rotation/drift now."));
  }
}

bool startSquareUp() {
  if (squareState != SQUARE_IDLE || driveActive) return false;
  if (!tof1_ok || !tof2_ok) {
    Serial.println(F("Cannot square: ToF1/ToF2 reading invalid."));
    return false;
  }
  float adjustedDiff = tof1Filtered() - tof2Filtered() - SQUARE_DIFF_OFFSET_MM;
  Serial.print(F("Square check: ToF1=")); Serial.print(tof1_mm);
  Serial.print(F(" ToF2=")); Serial.print(tof2_mm);
  Serial.print(F(" (filtered ")); Serial.print(tof1Filtered(), 0);
  Serial.print(F("/")); Serial.print(tof2Filtered(), 0);
  Serial.print(F(") adjustedDiff=")); Serial.println(adjustedDiff, 0);
  if (fabs(adjustedDiff) <= SQUARE_TOLERANCE_MM) {
    Serial.println(F("Already square (within tolerance)."));
    return true;
  }
  squareAttempts = 0;
  applySquareTurn(adjustedDiff);
  squareStateStartMs = millis();
  squareState = SQUARE_TURN_PULSE;
  return true;
}

void updateSquareUp() {
  if (squareState == SQUARE_IDLE) return;

  if (squareState == SQUARE_TURN_PULSE) {
    if (millis() - squareStateStartMs < squareCurrentPulseMs) return;
    setWheelSpeedsLR(0, 0);
    squareStateStartMs = millis();
    squareState = SQUARE_SETTLE;
    return;
  }

  if (squareState == SQUARE_SETTLE) {
    if (millis() - squareStateStartMs < SQUARE_SETTLE_MS) return;
    squareState = SQUARE_VERIFY;
    return;
  }

  // SQUARE_VERIFY
  if (!tof1_ok || !tof2_ok) {
    Serial.println(F("Square aborted: ToF reading lost mid-maneuver."));
    squareState = SQUARE_IDLE;
    return;
  }
  float adjustedDiff = tof1Filtered() - tof2Filtered() - SQUARE_DIFF_OFFSET_MM;
  Serial.print(F("Square check: ToF1=")); Serial.print(tof1_mm);
  Serial.print(F(" ToF2=")); Serial.print(tof2_mm);
  Serial.print(F(" (filtered ")); Serial.print(tof1Filtered(), 0);
  Serial.print(F("/")); Serial.print(tof2Filtered(), 0);
  Serial.print(F(") adjustedDiff=")); Serial.println(adjustedDiff, 0);

  if (fabs(adjustedDiff) <= SQUARE_TOLERANCE_MM) {
    Serial.println(F("Square: aligned."));
    squareState = SQUARE_IDLE;
    return;
  }

  squareAttempts++;
  if (squareAttempts >= SQUARE_MAX_ATTEMPTS) {
    Serial.println(F("Square aborted: too many attempts, not converging."));
    squareState = SQUARE_IDLE;
    return;
  }

  applySquareTurn(adjustedDiff);
  squareStateStartMs = millis();
  squareState = SQUARE_TURN_PULSE;
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(57600); // dropped from 115200 -- was dropping/stalling at the higher rate
  delay(2000);
  Serial.println(FIRMWARE_BUILD);
  Serial.println(F("Booting..."));
  Wire.begin();
  // Without this, a glitched I2C transaction (e.g. electrical noise
  // from the servos coupling into SDA/SCL) can hang Wire.* calls
  // forever, freezing the whole sketch -- which looks exactly like
  // "Serial just stops printing" since loop() never gets back around
  // to it. This lets a stuck bus time out and auto-recover instead.
  Wire.setWireTimeout(25000, true); // 25ms timeout, reset bus on timeout

  for (int i = 0; i < NUM_HIPS; i++) {
    hipServos[i].attach(HIP_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    // Pre-set hipPos so the first move is a zero-length ease (instant
    // snap to home), not a sweep from the uninitialized default of 0.
    hipPos[i] = HIP_START[i];
    setHip(i, HIP_START[i]);

    if (!kneeInstalled[i]) continue;
    kneeServos[i].attach(KNEE_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    kneePos[i] = KNEE_START[i];
    setKnee(i, KNEE_START[i]);
  }
  Serial.println(F("Servos OK"));

  for (int i = 0; i < NUM_HIPS; i++) {
    pinMode(WHEEL_IN1_PINS[i], OUTPUT);
    pinMode(WHEEL_IN2_PINS[i], OUTPUT);
    pinMode(WHEEL_EN_PINS[i], OUTPUT);
  }
  stopWheels();
  Serial.println(F("Wheel drive OK"));

  setupVL53L0X();
  setupMPU6050();

  // Lower from the full-extension home pose into CROUCH_LOW, the
  // hand-confirmed lowest level stance -- queued here (after the setup
  // delays above) rather than right after the home-pose snap, so the
  // eased ramp isn't skipped over by time that elapses during
  // setupVL53L0X()/setupMPU6050()'s delay() calls before loop() gets a
  // chance to start animating it. Send "stand" to raise it gradually
  // to full standing height from here.
  enterCrouchLow();
  Serial.println(F("Startup height (confirmed low crouch) -> stand up with 'stand'."));

  Serial.println();
  Serial.println(F("Ready. Type 'help' for commands."));
  Serial.println();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // Step any in-progress eased servo moves forward
  updateServoMotion();

  // Step any in-progress lift/lower sequence forward
  updateLiftSequence();

  // Watch any in-progress climb-pose move for tilt, and settle it
  updateClimbMoveTracking();

  // Step any in-progress stand sequence forward
  updateStand();

  // Step any in-progress stand sweep forward
  updateStepScan();

  // Step any in-progress auto-step (sweep-detected -> auto placement) forward
  updateAutoStep();

  // Nudge toward level if self-balancing is enabled
  updateBalance();

  // Pick up new ToF data as soon as it's ready
  pollTofSensors();

  // Auto-stop a timed drive once its duration elapses
  updateDrive();

  // Step any in-progress square-up (turn until ToF1/ToF2 agree) forward
  updateSquareUp();

  // Auto-stop a single turn-test pulse once its duration elapses
  updateTurnTest();

  // Step any in-progress rear-raise (extend RL/RR toward level) forward
  updateRaiseRear();

  // Step any in-progress rear-prep (front knees -> front hips -> rear hips/knees) forward
  updateRearPrep();

  // Step any in-progress rear-wheel lift (RL/RR clear of the ground) forward
  updateRearWheelLift();

  // Step any in-progress new-stable-lift move (FR/RL/RR, no tilt check) forward
  updateNewStableLift();

  // Non-blocking command reader — works with any line ending
  String cmd = readCommand();
  if (cmd.length() > 0) handleCommand(cmd);
}
