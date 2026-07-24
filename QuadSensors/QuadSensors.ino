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
// 170 = leg straight up; capped there (rather than the servo's full 270)
// so it can't swing past vertical and clash with the top of the robot.
const int  HIP_MAX[NUM_HIPS]    = { 170, 170, 170, 170 };
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
// FL re-calibrated again: 42 looked straight -> trim +12.
// RR re-calibrated after fixing an under-voltage issue that was
// causing unreliable servo behavior on the rear knees: 40 straight
// -> trim +10.
// FR calibrated: 67 looked straight down -> trim +37.
// RL trim unchanged: 60 straight -> trim +30.
const int  HIP_TRIM[NUM_HIPS]   = { 12, 37, 30, 10 };

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
const int  KNEE_MIN[NUM_HIPS]      = {   0,   0,   0,   0 };
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

#define FIRMWARE_BUILD "QuadSensors build 2026-07-23-d (VL53L0X)"

// ============================================================
// TIMING
// ============================================================
#define SENSOR_INTERVAL_MS 200
unsigned long lastSensorPrint = 0;

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
  hipMoveDurationMs[i] = max((unsigned long)(fabs(angle - hipPos[i]) / HIP_MOVE_DEG_PER_SEC * 1000.0), MOVE_MIN_MS);
}

void startKneeMove(int i, int angle) {
  kneeMoveFrom[i] = kneePos[i];
  kneeMoveTo[i]   = angle;
  kneeMoveStartMs[i] = millis();
  kneeMoveDurationMs[i] = max((unsigned long)(fabs(angle - kneePos[i]) / KNEE_MOVE_DEG_PER_SEC * 1000.0), MOVE_MIN_MS);
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
// SAFETY: the "full combined range (thigh and knee both bent back to
// their limits at once) doesn't hit the chassis" check has only been
// physically confirmed for FL so far. FR (and RL/RR later) need
// their own confirmation before trusting foot_* near the back of
// their workspace -- mirrored legs aren't guaranteed identical
// clearance, the same reason FR needed its own hip_fr minimum (2)
// distinct from FL's (6). No combined-angle limit is enforced beyond
// each leg's own HIP_MIN/MAX and KNEE_MIN/MAX.
// ============================================================
const float LEG_THIGH_MM = 165.0;
const float LEG_CALF_MM  = 105.0;

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

  // Knee bends backward (toward the chassis) to reach a shortened
  // target, matching a normal walking gait where the foot lifts by
  // folding the knee back and up. If an assembled leg needs the
  // other branch instead, flip the sign here.
  float theta2 = -acos(cosKnee);

  float k1 = LEG_THIGH_MM + LEG_CALF_MM * cos(theta2);
  float k2 = LEG_CALF_MM * sin(theta2);
  float theta1 = atan2(x, y) - atan2(k2, k1);

  hipAngleOut  = degrees(theta1) + HIP_START[i];
  kneeAngleOut = degrees(theta2) + KNEE_START[i];
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

// Checks whether the body's center (approximating the center of
// mass) falls within the support triangle of legs a, b, c, given
// their current foot positions.
bool isStableOn(int a, int b, int c) {
  float ax, ay, bx, by, cx, cy;
  footBodyPosition(a, ax, ay);
  footBodyPosition(b, bx, by);
  footBodyPosition(c, cx, cy);
  return pointInTriangle(0, 0, ax, ay, bx, by, cx, cy);
}

// ============================================================
// LEG LIFT SEQUENCE (front legs only for now)
// With all four feet at their neutral stance, the geometry above
// works out to a real finding: lifting one front leg leaves the
// body's center sitting exactly on the boundary of the remaining
// 3-leg support triangle -- zero margin, not just "a bit tight". A
// weight shift before lifting isn't optional here.
//
// Since these legs can only move fore-aft (no side-to-side/
// ab-adduction capability), the shift below is a centroid-alignment
// heuristic: move the other three feet's local x so the body's
// center moves toward the centroid of their (new) foot positions.
// This is a first-pass approach, not an optimal-margin solve, and
// the isStableOn() check afterward is what actually gates the lift --
// if the shift didn't achieve real margin, the lift aborts with a
// message rather than proceeding on the heuristic alone.
//
// UNTESTED ON HARDWARE. Watch closely and be ready to catch/support
// the robot the first several times this runs.
// ============================================================
#define LEG_LIFT_MM 30.0 // conservative -- thighs should not fully lift yet

enum LiftState { LIFT_IDLE, LIFT_SHIFTING, LIFT_LIFTING, LIFT_UP, LIFT_LOWERING };
LiftState liftState = LIFT_IDLE;
int liftLegIdx = -1;
int liftStanceIdx[3];
float liftStanceX[3], liftStanceY[3]; // stance-leg foot positions before the shift, to restore on lower

bool legMoveDone(int i) {
  unsigned long now = millis();
  bool hipDone  = (now - hipMoveStartMs[i]) >= hipMoveDurationMs[i];
  bool kneeDone = !kneeInstalled[i] || ((now - kneeMoveStartMs[i]) >= kneeMoveDurationMs[i]);
  return hipDone && kneeDone;
}

// Starts lifting leg legToLift (FL or FR only). Returns false without
// doing anything if a lift is already in progress or the leg isn't a
// front leg. NOTE: if a stance leg's shift turns out to be
// unreachable partway through, the legs before it in the loop will
// already have been commanded -- this doesn't roll those back.
bool startLift(int legToLift) {
  if (legToLift != FL && legToLift != FR) return false;
  if (liftState != LIFT_IDLE) return false;

  int n = 0;
  for (int i = 0; i < NUM_HIPS; i++) {
    if (i == legToLift) continue;
    liftStanceIdx[n] = i;
    legForwardKinematics(i, liftStanceX[n], liftStanceY[n]);
    n++;
  }

  float bx[3], by[3];
  for (int k = 0; k < 3; k++) footBodyPosition(liftStanceIdx[k], bx[k], by[k]);
  float centroidX = (bx[0] + bx[1] + bx[2]) / 3.0;

  // A planted foot is fixed on the ground -- commanding it "forward"
  // in body-local terms moves the body backward relative to it, and
  // vice versa, so shift each stance leg the opposite way to move
  // the body toward the centroid.
  for (int k = 0; k < 3; k++) {
    int i = liftStanceIdx[k];
    if (!setFoot(i, liftStanceX[k] - centroidX, liftStanceY[k])) return false;
  }

  liftLegIdx = legToLift;
  liftState = LIFT_SHIFTING;
  return true;
}

// Starts lowering the currently-lifted leg and restoring the shifted
// stance legs back to their pre-lift positions.
bool startLower() {
  if (liftState != LIFT_UP) return false;
  float lx, ly;
  legForwardKinematics(liftLegIdx, lx, ly);
  setFoot(liftLegIdx, lx, ly + LEG_LIFT_MM);
  for (int k = 0; k < 3; k++) setFoot(liftStanceIdx[k], liftStanceX[k], liftStanceY[k]);
  liftState = LIFT_LOWERING;
  return true;
}

// Steps the lift/lower sequence forward -- call every loop() pass.
void updateLiftSequence() {
  if (liftState == LIFT_SHIFTING) {
    for (int k = 0; k < 3; k++) if (!legMoveDone(liftStanceIdx[k])) return;
    if (!isStableOn(liftStanceIdx[0], liftStanceIdx[1], liftStanceIdx[2])) {
      Serial.println("Lift aborted: center still not inside the support triangle after shifting.");
      liftState = LIFT_IDLE;
      liftLegIdx = -1;
      return;
    }
    float lx, ly;
    legForwardKinematics(liftLegIdx, lx, ly);
    setFoot(liftLegIdx, lx, ly - LEG_LIFT_MM); // y is down+, so subtract to raise the foot
    liftState = LIFT_LIFTING;

  } else if (liftState == LIFT_LIFTING) {
    if (!legMoveDone(liftLegIdx)) return;
    Serial.println("Leg lifted.");
    liftState = LIFT_UP;

  } else if (liftState == LIFT_LOWERING) {
    bool allDone = legMoveDone(liftLegIdx);
    for (int k = 0; k < 3; k++) allDone = allDone && legMoveDone(liftStanceIdx[k]);
    if (!allDone) return;
    Serial.println("Leg lowered, stance restored.");
    liftState = LIFT_IDLE;
    liftLegIdx = -1;
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
    allHips(90);
    Serial.println("All hips -> 90");

  } else if (input == "sensors") {
    printSensors();

  } else if (input == "help") {
    Serial.println();
    Serial.println("Commands: start | all <angle> | hip_fl/fr/rl/rr <angle> | knee_fl/fr/rl/rr <angle> | foot_fl/fr <x_mm> <y_mm> | lift_fl | lift_fr | lower | sensors | help");
    Serial.println();

  } else if (input == "lift_fl" || input == "lift_fr") {
    int legIdx = (input == "lift_fl") ? FL : FR;
    if (startLift(legIdx)) {
      Serial.println("Shifting weight before lift...");
    } else {
      Serial.println("Cannot start lift (already mid-sequence, or unreachable shift).");
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

  } else if (input.startsWith("foot_fl ") || input.startsWith("foot_fr ")) {
    int legIdx = input.startsWith("foot_fl ") ? FL : FR;
    const char *legName = (legIdx == FL) ? "foot_fl" : "foot_fr";
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

  // Pick up new ToF data as soon as it's ready, independent of the print timer
  pollTofSensors();

  // Auto sensor print every 200ms
  if (millis() - lastSensorPrint >= SENSOR_INTERVAL_MS) {
    printSensors();
    lastSensorPrint = millis();
  }

  // Non-blocking command reader — works with any line ending
  String cmd = readCommand();
  if (cmd.length() > 0) handleCommand(cmd);
}
