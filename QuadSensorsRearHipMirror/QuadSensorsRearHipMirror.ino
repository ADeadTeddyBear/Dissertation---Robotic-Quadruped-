// ============================================================
// QuadSensorsRearHipMirror.ino
// Fork of QuadSensors.ino that adds a mirrored fallback branch for
// the rear legs' height IK -- see computeRearJointsForHeightMirrored()
// below. Kept as a separate sketch (rather than edited in place) since
// the mirrored branch is UNTESTED ON HARDWARE; QuadSensors.ino is left
// exactly as merged so it stays the known-good build.
//
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
// going lower risks the leg colliding with/damaging the robot. RL/RR
// confirmed clash-free all the way down to 0.
const int  HIP_MIN[NUM_HIPS]    = {   6,   2,   0,   0 };
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
// RL trim unchanged: 60 straight -> trim +30.
const int  HIP_TRIM[NUM_HIPS]   = { 14, 37, 30, 10 };

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
// rear stance.
const int  KNEE_MIN[NUM_HIPS]      = {   0,   0,  20,  20 };
const int  KNEE_MAX[NUM_HIPS]      = { 270, 270, 270, 270 };
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

#define FIRMWARE_BUILD "QuadSensorsRearHipMirror build 2026-07-25-i (VL53L0X)"

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
  angle = constrain(angle, KNEE_MIN[i], KNEE_MAX[i]);
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
const float LEG_CALF_MM  = 150.0; // hip-to-knee measured 165mm, knee-to-ground (INCLUDING the wheel) measured 150mm -- corrected from an earlier 105mm that didn't include the wheel

// Solves 2-link planar IK for leg i. (x, y) is the desired foot
// position relative to that leg's hip pivot, in mm (x forward+, y
// down+). Returns false if the target is out of reach; otherwise
// fills hipAngleOut/kneeAngleOut with servo angles. These are not yet
// clamped to leg i's HIP_MIN/MAX or KNEE_MIN/MAX -- setFoot() still
// routes them through setHip()/setKnee(), which enforce those.
bool solveLegIK(int i, float x, float y, float &hipAngleOut, float &kneeAngleOut) {
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
  float bestHip = 0, bestKnee = 0, bestAngleChange = -1;
  for (int branch = 0; branch < 2; branch++) {
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
bool setFoot(int i, float x, float y) {
  float hipAngle, kneeAngle;
  if (!solveLegIK(i, x, y, hipAngle, kneeAngle)) return false;
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

// Same as computeFrontJointsForHeight(), for leg i (RL/RR), using the
// confirmed hip-/knee- fold direction.
bool computeRearJointsForHeightPrimary(int i, float heightMM, int &hipOut, int &kneeOut) {
  float bRear = rearAmountForHeight(heightMM);
  int hip  = (int)round(HIP_START[i] - bRear);
  int knee = (int)round(KNEE_START[i] - bRear);
  if (hip < HIP_MIN[i] || hip > HIP_MAX[i] || knee < KNEE_MIN[i] || knee > KNEE_MAX[i]) return false;
  hipOut = hip;
  kneeOut = knee;
  return true;
}

// Mirror image of computeRearJointsForHeightPrimary(): folds the rear
// leg hip+/knee+ instead of hip-/knee-. rearAmountForHeight()'s height
// formula comes from cos(2*amount) via the double-angle identity, and
// cosine is even, so the SAME amount reaches the SAME height regardless
// of which way the leg folds -- only the hip/knee sign (and the foot's
// horizontal drift direction) differs.
//
// This exists because the primary branch runs the rear hip toward 0
// as heightMM drops (crouching), and below about 195mm it needs to go
// negative, past HIP_MIN=0 -- e.g. heightMM=100 needs hip=~-18 deg.
// Folding the other way instead needs hip=~+78 deg and knee=~+183 deg
// at that same height, comfortably inside HIP_MIN/MAX and KNEE_MIN/MAX
// (and nowhere near the 220-degree hip ceiling that must never be
// exceeded).
//
// UNTESTED ON HARDWARE: this drifts the rear foot forward as the body
// crouches (the same direction as the front feet), the opposite of the
// primary branch's backward drift, and the "does the fold clear the
// chassis" check (see the SAFETY note above solveLegIK()) has only
// been confirmed for FL, not RL/RR. Jog slowly and watch for chassis
// contact before trusting this near the bottom of the rear legs'
// travel.
bool computeRearJointsForHeightMirrored(int i, float heightMM, int &hipOut, int &kneeOut) {
  float bRear = rearAmountForHeight(heightMM);
  int hip  = (int)round(HIP_START[i] + bRear);
  int knee = (int)round(KNEE_START[i] + bRear);
  if (hip < HIP_MIN[i] || hip > HIP_MAX[i] || knee < KNEE_MIN[i] || knee > KNEE_MAX[i]) return false;
  hipOut = hip;
  kneeOut = knee;
  return true;
}

// Tries the confirmed hip-/knee- fold first; only falls back to the
// untested mirrored fold if the primary direction can't reach
// heightMM without a joint passing its real limit. Keeps today's
// behavior (and today's tested foot path) unchanged for every height
// the primary branch already reaches.
bool computeRearJointsForHeight(int i, float heightMM, int &hipOut, int &kneeOut) {
  if (computeRearJointsForHeightPrimary(i, heightMM, hipOut, kneeOut)) return true;
  return computeRearJointsForHeightMirrored(i, heightMM, hipOut, kneeOut);
}

// Sets the body to heightMM, level front-to-back, while preserving
// the confirmed joint-bend directions (front hip+/knee-, rear
// hip-/knee-, falling back to the untested rear hip+/knee+ mirror only
// when the confirmed direction can't reach heightMM -- see
// computeRearJointsForHeight()) -- replaces the earlier uniform
// setFoot(i, 0, height)
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
float lastCommandedHeight = 315; // LEG_THIGH_MM + LEG_CALF_MM -- full extension, matches whatever those are currently set to

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
          Serial.println("Auto step placement could not start (unreachable weight-shift at this stance).");
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
const float BODY_HALF_LENGTH_MM = 157.5; // half of 315mm front-to-rear hip spacing
const float BODY_HALF_WIDTH_MM  = 102.5; // half of 205mm left-to-right hip spacing
const float HIP_OFFSET_X[NUM_HIPS] = {  BODY_HALF_LENGTH_MM,  BODY_HALF_LENGTH_MM, -BODY_HALF_LENGTH_MM, -BODY_HALF_LENGTH_MM }; // FL,FR,RL,RR
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
// (before any shift). A brute-force sweep rather than a closed-form
// solve -- this only runs once per lift/step-placement start, not in
// the control loop, so the cost is negligible, and it doesn't depend
// on the worst-case-edge staying the same one throughout the search
// the way a more clever approach might assume.
void findBestStabilityShift(float bx[3], float by[3], float &bestShiftOut, float &bestMarginOut) {
  bestShiftOut = 0;
  bestMarginOut = -1.0;
  for (float shift = -STABILITY_SHIFT_SEARCH_RANGE_MM; shift <= STABILITY_SHIFT_SEARCH_RANGE_MM; shift += STABILITY_SHIFT_SEARCH_STEP_MM) {
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

// The whole lift/step-placement sequence moves at this fraction of
// normal servo speed (see moveSpeedScale) -- confirmed manually that a
// slow, careful, incremental approach is what actually gets a foot
// onto a step without tipping; this is the automated equivalent of
// that same care, not just the geometry alone. Applies to every phase
// (raise, shift, tuck, clear, reach, and the retract path back down),
// not just the reach itself, since the shift and raise are just as
// capable of upsetting balance if done abruptly.
#define LIFT_MOVE_SPEED_SCALE 0.3

enum LiftState { LIFT_IDLE, LIFT_RAISING, LIFT_SHIFTING, LIFT_TUCK, LIFT_CLEAR, LIFT_REACH, LIFT_HOLDING, LIFT_RISE, LIFT_RETRACT, LIFT_UNTUCK, LIFT_LOWERING };
LiftState liftState = LIFT_IDLE;
int liftLegIdx = -1;
int liftStanceIdx[3];
float liftStanceX[3], liftStanceY[3]; // stance-leg foot positions before the shift, to restore on lower
float liftOrigX, liftOrigY;           // the lifted leg's own foot position before the shift, to restore on lower
bool  liftIsStepPlace = false;
float liftStepForwardMM = 0, liftStepHeightMM = 0;

// Returns to idle from anywhere in the sequence (abort or success) --
// centralizing this so moveSpeedScale can never be left slow after
// the sequence ends, forgotten in one abort path but not another.
void abortLiftSequence() {
  liftState = LIFT_IDLE;
  liftLegIdx = -1;
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
bool startLower() {
  if (liftState != LIFT_HOLDING) return false;
  if (liftIsStepPlace) {
    // Rise straight up (vertical-only) off the step to the same clear
    // height used on the way in, BEFORE moving back horizontally --
    // mirrors the outbound CLEAR-then-descend split so the foot never
    // drags back across the step's front face at tread height.
    setFoot(liftLegIdx, liftStepForwardMM, computeClearY());
    liftState = LIFT_RISE;
  } else {
    setFoot(liftLegIdx, liftOrigX, liftOrigY);
    liftState = LIFT_UNTUCK;
  }
  return true;
}

// Steps the lift/reach/lower sequence forward -- call every loop() pass.
void updateLiftSequence() {
  if (liftState == LIFT_RAISING) {
    if (standMoveInProgress) return; // still rising to LIFT_STAND_TARGET_PROGRESS

    // NOW capture foot positions -- after the raise, not before it --
    // since standing taller moves every foot's position relative to
    // its hip (see applyStandProgress()/heightAtStandProgress()).
    legForwardKinematics(liftLegIdx, liftOrigX, liftOrigY);
    for (int k = 0; k < 3; k++) {
      legForwardKinematics(liftStanceIdx[k], liftStanceX[k], liftStanceY[k]);
    }

    float bx[3], by[3];
    for (int k = 0; k < 3; k++) footBodyPosition(liftStanceIdx[k], bx[k], by[k]);

    float bestShift, bestMargin;
    findBestStabilityShift(bx, by, bestShift, bestMargin);
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
    for (int k = 0; k < 3; k++) {
      int i = liftStanceIdx[k];
      if (!setFoot(i, liftStanceX[k] - bestShift, liftStanceY[k])) {
        Serial.println("Lift aborted: weight-shift target unreachable.");
        abortLiftSequence();
        return;
      }
    }
    liftState = LIFT_SHIFTING;

  } else if (liftState == LIFT_SHIFTING) {
    for (int k = 0; k < 3; k++) if (!legMoveDone(liftStanceIdx[k])) return;
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
    // Tuck: bring the foot toward directly under the hip (x=0) while
    // raising it clear of the ground -- keeps the leg as close to the
    // body as possible while airborne, rather than swinging out first.
    if (!setFoot(liftLegIdx, 0, liftOrigY - LEG_LIFT_MM)) {
      Serial.println("Lift aborted: tuck-raise target unreachable.");
      abortLiftSequence();
      return;
    }
    liftState = LIFT_TUCK;

  } else if (liftState == LIFT_TUCK) {
    if (!legMoveDone(liftLegIdx)) return;
    if (liftIsStepPlace) {
      // Move forward to the step's x while staying at the elevated
      // clear height -- NOT yet the step's own target y -- so the
      // foot is already past the leading edge before it ever
      // descends to tread height.
      if (!setFoot(liftLegIdx, liftStepForwardMM, computeClearY())) {
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
    float targetY = lastCommandedHeight - liftStepHeightMM;
    if (!setFoot(liftLegIdx, liftStepForwardMM, targetY)) {
      Serial.println("Step placement aborted: descent target unreachable -- check step height against this leg's workspace.");
      liftState = LIFT_HOLDING; // still elevated and clear of the step; leave it there, not mid-fault
      return;
    }
    liftState = LIFT_REACH;

  } else if (liftState == LIFT_REACH) {
    if (!legMoveDone(liftLegIdx)) return;
    Serial.println("Foot placed on step.");
    liftState = LIFT_HOLDING;

  } else if (liftState == LIFT_RISE) {
    if (!legMoveDone(liftLegIdx)) return;
    // Now purely a horizontal move at the fixed clear height -- back
    // past the step's edge before ever descending toward the ground.
    setFoot(liftLegIdx, 0, computeClearY());
    liftState = LIFT_RETRACT;

  } else if (liftState == LIFT_RETRACT) {
    if (!legMoveDone(liftLegIdx)) return;
    setFoot(liftLegIdx, liftOrigX, liftOrigY);
    liftState = LIFT_UNTUCK;

  } else if (liftState == LIFT_UNTUCK) {
    if (!legMoveDone(liftLegIdx)) return;
    for (int k = 0; k < 3; k++) setFoot(liftStanceIdx[k], liftStanceX[k], liftStanceY[k]);
    liftState = LIFT_LOWERING;

  } else if (liftState == LIFT_LOWERING) {
    bool allDone = legMoveDone(liftLegIdx);
    for (int k = 0; k < 3; k++) allDone = allDone && legMoveDone(liftStanceIdx[k]);
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
    Serial.println("Auto step placement complete -- foot on step. Send 'lower' when ready to retract.");
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
    Serial.println("Commands: start | all <angle> | hip_fl/fr/rl/rr <angle> | knee_fl/fr/rl/rr <angle> | foot_fl/fr/rl/rr <x_mm> <y_mm> | stand | stand <percent> | stand_sweep | lift_fl/fr/rl/rr | step_fl/fr/rl/rr <forward_mm> <step_height_mm> | step_scan_fl/fr/rl/rr | lower | level | balance on/off | sensors | help");
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
