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
#define TOF1_FORWARD_OFFSET_MM   -60.0 // measured: 60mm BEHIND the front hip pivots (corrected from an earlier "~3mm forward" estimate)
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
    Serial.println("Stand target reached.");
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
  Serial.print("ToF1 cleared lip at "); Serial.print((int)round(standProgress * 100)); Serial.print("%: baseline=");
  if (scanBaselineToF1Ok) { Serial.print(scanBaselineToF1); Serial.print("mm"); } else { Serial.print("---"); }
  Serial.print(" -> now=");
  if (tof1_ok) { Serial.print(tof1_mm); Serial.print("mm"); } else { Serial.print("---"); }
  Serial.print("  (possible step, cumulative delta >= "); Serial.print(STEP_CHANGE_THRESHOLD_MM); Serial.print("mm)");
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
    Serial.print("  -> estimated step: height~"); Serial.print(stepHeightMM, 0);
    Serial.print("mm at ~"); Serial.print(stepForwardMM, 0);
    Serial.println("mm forward of the hip. UNVALIDATED estimate -- sanity-check before trusting step_scan_*.");

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
      Serial.print("  -> REJECTED: ");
      Serial.print(stepForwardMM, 0);
      Serial.print("mm exceeds this leg's max possible reach ("); Serial.print(LEG_THIGH_MM + LEG_CALF_MM, 0);
      Serial.println("mm) -- not usable, not stored. Distance estimate is unreliable for this scan, not just this leg's workspace.");
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
        Serial.println("Auto-attempting step placement...");
        if (startPlaceOnStep(AUTO_STEP_LEG, lastDetectedStepForwardMM, lastDetectedStepHeightMM)) {
          autoStepState = AUTO_PLACING;
        } else {
          Serial.println("Auto step placement could not start -- a lift/step sequence is already in progress (liftState != LIFT_IDLE). If the last attempt ended in a hold or a safety abort, send 'lower' first -- manually jogging hip_xx/knee_xx does NOT reset this.");
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
    Serial.print(curPercent); Serial.print("%: ToF1=");
    if (tof1_ok) {
      Serial.print(tof1_mm);
      float shift = footXAtStandProgress(TOF1_HEIGHT_REF_LEG, 0.0) - footXAtStandProgress(TOF1_HEIGHT_REF_LEG, standProgress);
      Serial.print("mm (self-motion compensated: "); Serial.print((float)tof1_mm + shift, 0); Serial.println("mm)");
    } else {
      Serial.println("---");
    }
    nextScanReportPercent += SCAN_REPORT_STEP_PERCENT;
  }

  if (standProgress >= 1.0) {
    scanState = SCAN_IDLE;
    if (!scanStepReported) {
      Serial.println("Scan complete. No step/lip crossing found in this range.");
    } else {
      Serial.println("Scan complete.");
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
  readMPU6050(pitch, roll);
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
#define LIFT_DESCEND_STEPS 12
#define LIFT_CONTACT_TILT_DELTA_DEG 2.0

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
#define PRECLIMB_HIP_FR   92
#define PRECLIMB_KNEE_FR  108

// ============================================================
// SAFE-KNEE LIFT (FL lift only)
// Requested directly: before the hip starts lifting the leg, first
// move the knee to a safe, verified position on its own and let that
// settle -- confirmed by hand (CLIMB_PREP_TALL/CLIMB_LIFT_TALL) that
// knee=270 is a safe fold to hold the leg at before the hip does any
// large motion, rather than moving hip and knee together into unknown
// combined territory. Only the hip moves during the lift-off itself;
// the knee is left alone here and only changes later (during the
// reach) if a step-place is actually in progress.
//
// LIFT_LIFTED_HIP_FL=150 matches CLIMB_LIFT_TALL exactly -- the one
// combination hardware-confirmed (real IMU: Level) to pair a genuinely
// lifted-looking hip angle with this same safe knee, regardless of
// which prep pose the leg started from.
//
// Declared here (not next to where it's used, further down) because
// startLower() -- also further down, but textually BEFORE
// updateLiftSequence() -- needs these as plain #defines, and macros
// (unlike functions) aren't auto-prototyped by Arduino: they must
// textually precede their first use in the file.
// ============================================================
#define LIFT_SAFE_KNEE_FL   270
#define LIFT_LIFTED_HIP_FL  150

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

enum LiftState { LIFT_IDLE, LIFT_RAISING, LIFT_SHIFTING, LIFT_SETTLING, LIFT_REMEASURE_DOWN, LIFT_REMEASURE_UP, LIFT_KNEE_SAFE, LIFT_TUCK, LIFT_CLEAR, LIFT_DESCEND, LIFT_REACH, LIFT_HOLDING, LIFT_RISE, LIFT_UNTUCK, LIFT_LOWERING };
LiftState liftState = LIFT_IDLE;
unsigned long liftSettleStartMs = 0;
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

// Returns to idle from anywhere in the sequence (abort or success) --
// centralizing this so moveSpeedScale can never be left slow after
// the sequence ends, forgotten in one abort path but not another.
void abortLiftSequence() {
  liftState = LIFT_IDLE;
  liftLegIdx = -1;
  liftIsSecondLeg = false;
  moveSpeedScale = 1.0;
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
  liftLegIdx = legToLift;
  int n = 0;
  for (int i = 0; i < NUM_HIPS; i++) {
    if (i == legToLift) continue;
    liftStanceIdx[n] = i;
    n++;
  }

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
  setKnee(liftLegIdx, (legToLift == FR) ? LIFT_SAFE_KNEE_FR : LIFT_SAFE_KNEE_FL);
  liftState = LIFT_KNEE_SAFE;
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
// genuine falls seen so far reached 35-41 degrees. 15 clears that
// nuisance case with room to spare while staying well below the range
// that's actually been an unrecoverable topple.
#define LIFT_TILT_ABORT_DEG 15.0
#define LIFT_TILT_CHECK_MS  50  // how often to poll the IMU while a sequence is active

// Tighter than LIFT_TILT_ABORT_DEG on purpose: this is the "are we
// actually stable enough to COMMIT to lifting a leg off the ground"
// check, not the "is it actively falling over" check -- want to catch
// a stance that's already leaning before removing one of its four
// points of contact, not just once it's clearly too late.
#define LIFT_PRELIFT_TILT_LIMIT_DEG 5.0

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
  readMPU6050(pitch, roll);
  if (fabs(pitch) < LIFT_TILT_ABORT_DEG && fabs(roll) < LIFT_TILT_ABORT_DEG) return false;

  Serial.print("LIFT SAFETY ABORT: body tilt pitch=");
  Serial.print(pitch, 1);
  Serial.print(" roll=");
  Serial.print(roll, 1);
  Serial.println(" exceeded the safety threshold -- freezing all legs where they are.");

  freezeLeg(liftLegIdx);
  for (int k = 0; k < 3; k++) freezeLeg(liftStanceIdx[k]);
  liftTiltAborted = true;
  liftState = LIFT_HOLDING; // existing 'lower' recovery path takes over from here
  return true;
}

// ============================================================
// VERIFIED CLIMB TIERS (FL lift only)
// Three complete, hand-verified pose sets spanning the low/mid/tall
// range tested directly on hardware, each confirmed level by the real
// IMU (not the analytic model -- that's confirmed unreliable at these
// large angles, especially for RL/RR, see LEG_CALF_MM's comment).
// Each tier has a PREP pose (FL still down at its rest position, other
// three legs in their tested position) and a LIFT pose (FL raised/
// extended, others unchanged or minimally adjusted, matching exactly
// what was verified). Selected entirely by name (low/mid/tall), not by
// any computed height -- there isn't yet a trustworthy way to map a
// detected step height onto one of these three tiers automatically,
// so for now the operator picks the tier that matches the step in
// front of the robot.
//
// Commanded as raw hip/knee angles via commandClimbPose(), bypassing
// solveLegIK()/setFoot() entirely for all four legs, the same
// reasoning as the earlier PRECLIMB_* stance: the model cannot be
// trusted to reproduce or verify poses at this angle range.
//
// (ClimbPose itself is declared near the top of the file, right after
// the #includes -- Arduino auto-generates a forward declaration for
// commandClimbPose() right after the #includes too, and that
// declaration needs ClimbPose to already be a known type at that
// point, not just here where it's actually used.)
// ============================================================
const ClimbPose CLIMB_PREP_LOW  = {  95, 110,  95, 120,   0,  20,  10,  20 }; // Pitch -0.3 Roll 2.9 -> Level
const ClimbPose CLIMB_LIFT_LOW  = { 200, 110,  92, 120,   0,  20,   0,  20 }; // Pitch  0.9 Roll 2.1 -> Level
const ClimbPose CLIMB_PREP_MID  = {  92, 100,  92, 108,   0,  50,   0,  55 }; // Pitch  1.9 Roll 2.5 -> Level
const ClimbPose CLIMB_LIFT_MID  = { 200, 100,  88, 108,   0,  50,   0,  48 }; // Pitch  2.5 Roll 1.9 -> Level
const ClimbPose CLIMB_PREP_TALL = { 150, 270,  60, 150,   0,  80,   0,  80 }; // Pitch  2.0 Roll 0.8 -> Level (FL knee in a "safe spot", not yet extended)
const ClimbPose CLIMB_LIFT_TALL = { 150, 145,  60, 150,   0,  80,   0,  80 }; // Pitch  2.8 Roll 1.1 -> Level (FL knee swings to fully extended)

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

// Same reasoning as checkLiftTiltSafety(), reused here since a climb
// pose move is just as capable of tipping the robot -- freezes all
// four legs (not three, since none of them are "the lifted leg" in
// this scheme) if tilt exceeds LIFT_TILT_ABORT_DEG.
bool checkClimbTiltSafety() {
  if (millis() - lastClimbTiltCheckMs < LIFT_TILT_CHECK_MS) return false;
  lastClimbTiltCheckMs = millis();

  float pitch, roll;
  readMPU6050(pitch, roll);
  if (fabs(pitch) < LIFT_TILT_ABORT_DEG && fabs(roll) < LIFT_TILT_ABORT_DEG) return false;

  Serial.print("CLIMB SAFETY ABORT: body tilt pitch=");
  Serial.print(pitch, 1);
  Serial.print(" roll=");
  Serial.print(roll, 1);
  Serial.println(" exceeded the safety threshold -- freezing all legs where they are.");

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
  Serial.println("Climb pose reached.");
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
  if (liftState != LIFT_IDLE && liftState != LIFT_HOLDING &&
      liftState != LIFT_RAISING && liftState != LIFT_SHIFTING &&
      liftState != LIFT_SETTLING && liftState != LIFT_REMEASURE_DOWN &&
      liftState != LIFT_REMEASURE_UP) {
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
      Serial.print("Lift aborted: best achievable stability margin is ");
      Serial.print(bestMargin, 0);
      Serial.print("mm, below the ");
      Serial.print(MIN_STABILITY_MARGIN_MM, 0);
      Serial.println("mm safety floor -- not attempting a shift from this stance.");
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
        Serial.println("Lift aborted: weight-shift target unreachable.");
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
        Serial.print("Lift aborted: settled stability margin is ");
        Serial.print(settledMargin, 0);
        Serial.println("mm after shifting -- below the safety floor, not proceeding to lift.");
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
      readMPU6050(pitch, roll);
      if (fabs(pitch) > LIFT_PRELIFT_TILT_LIMIT_DEG || fabs(roll) > LIFT_PRELIFT_TILT_LIMIT_DEG) {
        Serial.print("Lift aborted: body tilt pitch=");
        Serial.print(pitch, 1);
        Serial.print(" roll=");
        Serial.print(roll, 1);
        Serial.println(" already exceeds the pre-lift stability check -- not safe to lift from this stance.");
        abortLiftSequence();
        return;
      }
    }
    if (liftIsStepPlace) {
      // Re-measure the step's forward distance with a fresh, LIVE
      // ToF1 reading before committing to the reach -- see
      // REMEASURE_LOWER_DEG above for why a sink is needed first.
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
    if (tof1_ok) {
      float freshForwardMM = (float)tof1_mm + TOF1_FORWARD_OFFSET_MM;
      Serial.print("Re-measured step distance: "); Serial.print(freshForwardMM, 0);
      Serial.print("mm forward (scan estimate was "); Serial.print(liftStepForwardMM, 0);
      Serial.println("mm).");
      liftStepForwardMM = freshForwardMM;
    } else {
      Serial.println("Re-measure: ToF1 reading invalid, keeping the scan-derived estimate.");
    }
    // Back to the verified stance before continuing.
    createStablePlatform();
    liftState = LIFT_REMEASURE_UP;

  } else if (liftState == LIFT_REMEASURE_UP) {
    {
      bool allDone = true;
      for (int i = 0; i < NUM_HIPS; i++) allDone = allDone && legMoveDone(i);
      if (!allDone) return;
    }
    setKnee(liftLegIdx, LIFT_SAFE_KNEE_FL);
    liftState = LIFT_KNEE_SAFE;

  } else if (liftState == LIFT_KNEE_SAFE) {
    if (!legMoveDone(liftLegIdx)) return; // knee still settling into its safe position
    // Step 3 of 3, part B: NOW lift the hip, with the knee already
    // safely folded and holding still -- see LIFT_LIFTED_HIP_FL above.
    setHip(liftLegIdx, (liftLegIdx == FR) ? LIFT_LIFTED_HIP_FR : LIFT_LIFTED_HIP_FL);
    liftState = LIFT_TUCK;

  } else if (liftState == LIFT_TUCK) {
    if (!legMoveDone(liftLegIdx)) return; // hip still lifting
    if (liftIsStepPlace) {
      // Move forward to the step's x while staying at the elevated
      // clear height -- NOT yet the step's own target y -- so the
      // foot is already past the leading edge before it ever
      // descends to tread height.
      //
      // forceBranch pins FL to ONE elbow branch for both this move and
      // LIFT_CLEAR below -- confirmed on hardware that without forcing,
      // the reach can flip branch between the two calls (visually:
      // "swaps which way the elbow is held" and places the foot back
      // on the floor instead of the step), since legMoveDone() only
      // checks the move finished, not which branch it finished in.
      //
      // Branch was first tried as 1 (matching the safe-knee step's
      // knee=270, "above KNEE_START folds forward"), but confirmed on
      // hardware to still land wrong: for this far-forward, shallow-y
      // reach, branch 1 (theta2 = +kneeMag) swings the calf the SAME
      // rotational direction as the hip's forward lean -- a hyper-
      // extended curl that folds back up near the hip instead of
      // reaching down onto the step. Branch 0 (theta2 = -kneeMag)
      // folds the calf the OPPOSITE way from the hip's lean -- thigh
      // forward, shin bent back down to the foot, the natural
      // "reaching forward onto a step" shape -- so that's forced here
      // instead. The safe-knee step itself (knee=270, branch 1) is a
      // different target (tucked near the hip) and is left alone.
      int forceBranch = (liftLegIdx == FL || liftLegIdx == FR) ? 0 : -1;
      if (!setFoot(liftLegIdx, liftStepForwardMM, computeClearY(), forceBranch)) {
        Serial.println("Step placement aborted: clear-traverse target unreachable -- check step distance against this leg's workspace.");
        abortLiftSequence();
        return;
      }
      liftState = LIFT_CLEAR;
    } else {
      Serial.println("Leg lifted (tucked).");
      liftState = LIFT_HOLDING;
    }

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
    readMPU6050(liftDescendBasePitch, liftDescendBaseRoll);
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
      readMPU6050(pitch, roll);
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
      Serial.println("Step placement aborted: descent target unreachable -- check step height against this leg's workspace.");
      liftState = LIFT_HOLDING; // still elevated and clear of the step; leave it there, not mid-fault
      return;
    }

  } else if (liftState == LIFT_REACH) {
    if (!legMoveDone(liftLegIdx)) return;
    Serial.println(liftDescendStoppedEarly
      ? "Foot placed on step (stopped early: contact detected via tilt before reaching the full nominal descent)."
      : "Foot placed on step.");
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
    Serial.println("Leg lowered, stance restored.");
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
      Serial.println("Auto step placement ABORTED -- tilt safety triggered mid-sequence, foot NOT reliably placed. Send 'lower' to retract.");
    } else {
      Serial.println("Auto step placement complete -- foot on step. Send 'lower' when ready to retract.");
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
  Serial.println("MPU6050 ready.");
}

void readMPU6050(float &pitch, float &roll) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t AcX = Wire.read() << 8 | Wire.read();
  int16_t AcY = Wire.read() << 8 | Wire.read();
  int16_t AcZ = Wire.read() << 8 | Wire.read();

  pitch = atan2((float)AcX, sqrt((float)AcY * AcY + (float)AcZ * AcZ)) * 180.0 / PI;
  roll  = atan2((float)AcY, sqrt((float)AcX * AcX + (float)AcZ * AcZ)) * 180.0 / PI;
}

// Reports whether the body is level, using the MPU6050 as ground
// truth rather than just trusting the leg kinematics. Pitch is
// front-to-back tilt -- exactly what a front/rear height mismatch
// (like the one setBodyHeight() was fixed to avoid) would show up as.
#define LEVEL_TOLERANCE_DEG 3.0

void checkLevel() {
  float pitch, roll;
  readMPU6050(pitch, roll);
  Serial.print("Pitch:"); Serial.print(pitch, 1);
  Serial.print("  Roll:"); Serial.print(roll, 1);
  if (fabs(pitch) <= LEVEL_TOLERANCE_DEG && fabs(roll) <= LEVEL_TOLERANCE_DEG) {
    Serial.println("  -> Level");
  } else {
    Serial.println("  -> NOT level");
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
    Serial.println("Sensor 2 ready (0x52).");
  } else {
    Serial.println("Sensor 2 not found — skipping.");
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
    Serial.println("Sensor 1 ready (0x29).");
  } else {
    Serial.println("Sensor 1 not found — skipping.");
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
}

// ============================================================
// PRINT SENSOR VALUES
// ============================================================
void printSensors() {
  float pitch, roll;
  readMPU6050(pitch, roll);

  Serial.print("Pitch:"); Serial.print(pitch, 1);
  Serial.print("  Roll:"); Serial.print(roll, 1);

  if (tof1Active) {
    if (tof1_ok) {
      Serial.print("  ToF1:"); Serial.print(tof1_mm); Serial.print("mm");
    } else {
      Serial.print("  ToF1:---(raw="); Serial.print(tof1_mm); Serial.print(")");
    }
  } else {
    Serial.print("  ToF1:N/A");
  }

  if (tof2Active) {
    if (tof2_ok) {
      Serial.print("  ToF2:"); Serial.print(tof2_mm); Serial.print("mm");
    } else {
      Serial.print("  ToF2:---(raw="); Serial.print(tof2_mm); Serial.print(")");
    }
  } else {
    Serial.print("  ToF2:N/A");
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
    Serial.println("Start stance applied.");

  } else if (input == "angles") {
    // Prints every leg's CURRENT commanded hip/knee at once -- avoids
    // having to scroll back through a long jog session and reconstruct
    // "the final settled values" by hand (a real source of confusion
    // before this existed: a truncated log once left FL's actual final
    // angles misread as its very first jog value instead).
    const char* legNames[NUM_HIPS] = { "FL", "FR", "RL", "RR" };
    Serial.println("Current joint angles (hip/knee):");
    for (int i = 0; i < NUM_HIPS; i++) {
      Serial.print("  "); Serial.print(legNames[i]);
      Serial.print(": hip="); Serial.print(hipPos[i]);
      Serial.print(" knee="); Serial.println(kneePos[i]);
    }
    Serial.println("As #define lines:");
    for (int i = 0; i < NUM_HIPS; i++) {
      Serial.print("#define PRECLIMB_HIP_");  Serial.print(legNames[i]); Serial.print("   "); Serial.println(hipPos[i]);
      Serial.print("#define PRECLIMB_KNEE_"); Serial.print(legNames[i]); Serial.print("  "); Serial.println(kneePos[i]);
    }

  } else if (input == "climb_low_prep") {
    commandClimbPose(CLIMB_PREP_LOW);
    Serial.println("Commanding CLIMB_PREP_LOW.");

  } else if (input == "climb_low_lift") {
    commandClimbPose(CLIMB_LIFT_LOW);
    Serial.println("Commanding CLIMB_LIFT_LOW.");

  } else if (input == "climb_mid_prep") {
    commandClimbPose(CLIMB_PREP_MID);
    Serial.println("Commanding CLIMB_PREP_MID.");

  } else if (input == "climb_mid_lift") {
    commandClimbPose(CLIMB_LIFT_MID);
    Serial.println("Commanding CLIMB_LIFT_MID.");

  } else if (input == "climb_tall_prep") {
    commandClimbPose(CLIMB_PREP_TALL);
    Serial.println("Commanding CLIMB_PREP_TALL.");

  } else if (input == "climb_tall_lift") {
    commandClimbPose(CLIMB_LIFT_TALL);
    Serial.println("Commanding CLIMB_LIFT_TALL.");

  } else if (input == "sensors") {
    printSensors();

  } else if (input == "level") {
    checkLevel();

  } else if (input == "balance on") {
    balanceEnabled = true;
    Serial.println("Self-balancing enabled.");

  } else if (input == "balance off") {
    balanceEnabled = false;
    for (int i = 0; i < NUM_HIPS; i++) legHeightCorrection[i] = 0;
    setBodyHeight(lastCommandedHeight); // return to the uncorrected, uniform height
    Serial.println("Self-balancing disabled, corrections reset.");

  } else if (input == "help") {
    Serial.println();
    Serial.println("Commands: start | all <angle> | hip_fl/fr/rl/rr <angle> | knee_fl/fr/rl/rr <angle> | foot_fl/fr/rl/rr <x_mm> <y_mm> | angles | stand | stand <percent> | stand_sweep | lift_fl/fr/rl/rr | step_fl/fr/rl/rr <forward_mm> <step_height_mm> | step_scan_fl/fr/rl/rr | second_fr | climb_low/mid/tall_prep | climb_low/mid/tall_lift | lower | level | balance on/off | sensors | help");
    Serial.println();

  } else if (input == "stand_sweep") {
    if (startStepScan()) {
      Serial.println("Scanning 0->100%, will report ToF1 only on a large jump...");
    } else {
      Serial.println("Cannot start scan (already scanning, or a stand move is already in progress).");
    }

  } else if (input == "stand") {
    if (startStandMove(1.0)) {
      Serial.println("Standing up...");
    } else if (standProgress >= 1.0) {
      Serial.println("Already standing.");
    } else {
      Serial.println("Already moving.");
    }

  } else if (input.startsWith("stand ")) {
    float pct = constrain(input.substring(6).toFloat(), 0.0, 100.0);
    if (startStandMove(pct / 100.0)) {
      Serial.print("Moving to "); Serial.print(pct); Serial.println("% stand...");
    } else {
      Serial.println("Already moving, or already at that percent.");
    }

  } else if (input == "lift_fl" || input == "lift_fr" || input == "lift_rl" || input == "lift_rr") {
    int legIdx = (input == "lift_fl") ? FL : (input == "lift_fr") ? FR : (input == "lift_rl") ? RL : RR;
    if (startLift(legIdx)) {
      Serial.println("Raising to a stable stance before lift...");
    } else {
      Serial.println("Cannot start lift (already mid-sequence).");
    }

  } else if (input.startsWith("step_fl ") || input.startsWith("step_fr ") ||
             input.startsWith("step_rl ") || input.startsWith("step_rr ")) {
    int legIdx = input.startsWith("step_fl ") ? FL :
                 input.startsWith("step_fr ") ? FR :
                 input.startsWith("step_rl ") ? RL : RR;
    String rest = input.substring(8);
    int    sep  = rest.indexOf(' ');
    if (sep > 0) {
      float forwardMM = rest.substring(0, sep).toFloat();
      float heightMM  = rest.substring(sep + 1).toFloat();
      if (startPlaceOnStep(legIdx, forwardMM, heightMM)) {
        Serial.println("Raising to a stable stance before step placement...");
      } else {
        Serial.println("Cannot start step placement (already mid-sequence).");
      }
    } else {
      Serial.println("Usage: step_fl/fr/rl/rr <forward_mm> <step_height_mm>");
    }

  } else if (input == "step_scan_fl" || input == "step_scan_fr" ||
             input == "step_scan_rl" || input == "step_scan_rr") {
    if (!lastDetectedStepValid) {
      Serial.println("No step estimate yet -- run stand_sweep first and watch for an estimated-step line.");
    } else {
      int legIdx = (input == "step_scan_fl") ? FL : (input == "step_scan_fr") ? FR :
                   (input == "step_scan_rl") ? RL : RR;
      Serial.print("Using last scan estimate: height~"); Serial.print(lastDetectedStepHeightMM, 0);
      Serial.print("mm at ~"); Serial.print(lastDetectedStepForwardMM, 0); Serial.println("mm forward.");
      if (startPlaceOnStep(legIdx, lastDetectedStepForwardMM, lastDetectedStepHeightMM)) {
        Serial.println("Raising to a stable stance before step placement...");
      } else {
        Serial.println("Cannot start step placement (already mid-sequence).");
      }
    }

  } else if (input == "lower") {
    if (startLower()) {
      Serial.println("Lowering leg...");
    } else {
      Serial.println("No leg currently lifted.");
    }

  } else if (input == "second_fr") {
    // See startSecondLegOntoStep() -- untested stance, no verified
    // pose for "one front leg already on the step." Watch closely.
    if (startSecondLegOntoStep(FR)) {
      Serial.println("Placing FR onto the step next to the held leg -- UNTESTED stance, watch closely.");
    } else {
      Serial.println("Cannot start second-leg placement (first leg isn't down-and-holding, is already FR, or a safety abort is active -- send 'lower' first if so).");
    }

  } else if (input.startsWith("all ")) {
    int angle = input.substring(4).toInt();
    allHips(angle);
    Serial.print("All hips -> "); Serial.println(angle);

  } else if (input.startsWith("foot_fl ") || input.startsWith("foot_fr ") ||
             input.startsWith("foot_rl ") || input.startsWith("foot_rr ")) {
    int legIdx = input.startsWith("foot_fl ") ? FL :
                 input.startsWith("foot_fr ") ? FR :
                 input.startsWith("foot_rl ") ? RL : RR;
    const char *legName = (legIdx == FL) ? "foot_fl" :
                          (legIdx == FR) ? "foot_fr" :
                          (legIdx == RL) ? "foot_rl" : "foot_rr";
    String rest = input.substring(8);
    int    sep  = rest.indexOf(' ');
    if (sep > 0) {
      float x = rest.substring(0, sep).toFloat();
      float y = rest.substring(sep + 1).toFloat();
      if (setFoot(legIdx, x, y)) {
        Serial.print(legName); Serial.print(" -> hip="); Serial.print(hipPos[legIdx]);
        Serial.print(" knee="); Serial.println(kneePos[legIdx]);
      } else {
        Serial.print(legName); Serial.println(" target unreachable.");
      }
    } else {
      Serial.print("Usage: "); Serial.print(legName); Serial.println(" <x_mm> <y_mm>");
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
          Serial.print(HIP_NAMES[i]); Serial.print(" -> "); Serial.println(hipPos[i]);
          found = true;
          break;
        }
      }
      for (int i = 0; i < NUM_HIPS && !found; i++) {
        if (name == KNEE_NAMES[i]) {
          if (kneeInstalled[i]) {
            setKnee(i, angle);
            Serial.print(KNEE_NAMES[i]); Serial.print(" -> "); Serial.println(kneePos[i]);
          } else {
            Serial.print(KNEE_NAMES[i]); Serial.println(" not installed yet.");
          }
          found = true;
        }
      }
      if (!found) Serial.println("Unknown command. Type 'help'.");
    } else {
      Serial.println("Unknown command. Type 'help'.");
    }
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(57600); // dropped from 115200 -- was dropping/stalling at the higher rate
  delay(2000);
  Serial.println(FIRMWARE_BUILD);
  Serial.println("Booting...");
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
  Serial.println("Servos OK");

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
  Serial.println("Startup height (confirmed low crouch) -> stand up with 'stand'.");

  Serial.println();
  Serial.println("Ready. Type 'help' for commands.");
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

  // Non-blocking command reader — works with any line ending
  String cmd = readCommand();
  if (cmd.length() > 0) handleCommand(cmd);
}
