// ============================================================
// Tier1Testing.ino -- standalone sketch, Test 1 only: step height
// estimation accuracy.
//
// Independent of the other two validation tests (servo repeatability,
// IMU calibration) -- this sketch only touches the legs and the two
// ToF sensors, nothing else.
//
// All pin numbers, trims, joint limits, and IK formulas below are
// copied directly from QuadSensors.ino (not re-guessed), so results
// here reflect the real robot's real geometry:
//   - Hip/knee pins, HIP_TRIM, HIP_MIN/MAX, KNEE_MIN/MAX, HIP_START/
//     KNEE_START, HIP_MIRROR/KNEE_MIRROR: QuadSensors.ino's SERVO
//     CONFIG / KNEE CONFIG sections.
//   - LEG_THIGH_MM/LEG_CALF_MM, frontAmountForHeight()/
//     rearAmountForHeight()/computeFrontJointsForHeight()/
//     computeRearJointsForHeight(): QuadSensors.ino's LEG INVERSE
//     KINEMATICS section -- the same "accurate" height-command math
//     that used to back the "height <mm>" command (setBodyHeight()).
//   - setupVL53L0X()'s XSHUT sequencing (sensor 2 booted first) and
//     tuning: QuadSensors.ino's VL53L0X SETUP section.
//
// IMPORTANT -- what changed from the original test spec, and why:
//   There is NO downward-facing ToF sensor on this robot. Both ToF1
//   (0x29) and ToF2 (0x52, via XSHUT_2) are mounted level and
//   forward-facing, side by side (ToF1 left-of-centre, ToF2 right-
//   of-centre) -- confirmed by QuadSensors.ino's own square-up
//   comments. So this test cannot use a fixed
//   "H_step = h_s - d_step*sin(45deg)" formula; that sensor doesn't
//   exist. Both sensors are still booted (XSHUT sequencing needs both
//   to avoid an I2C address clash), but only ToF1 is actually read.
//
//   Instead, this reimplements the REAL method QuadSensors.ino uses
//   for step height (see its "ToF1 -> step geometry" comment): sweep
//   commanded body height up via IK (computeFrontJointsForHeight()/
//   computeRearJointsForHeight(), the same math as the old accurate
//   "height <mm>" command) while ToF1's level, forward-aimed beam
//   stays fixed on the step's front face; the moment the sensor's
//   rising HEIGHT clears the step's top edge, ToF1's reading jumps
//   (reused threshold: 150mm, same as QuadSensors.ino's
//   STEP_CHANGE_THRESHOLD_MM). Estimated step height = the body
//   height at that crossover + TOF1_HEIGHT_ABOVE_HIP_MM (same
//   constant QuadSensors.ino uses) + a calibration offset.
//
// Servos are written directly (writeMicroseconds), not through
// QuadSensors.ino's eased-motion system -- movements snap instead of
// easing; explicit settle delays are used instead.
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

// ------------------------------------------------------------
// SERVO CONFIG -- copied from QuadSensors.ino
// ------------------------------------------------------------
enum { FL = 0, FR, RL, RR, NUM_HIPS };

const int HIP_PINS[NUM_HIPS]    = { 6, 5, 8, 7 };      // FL, FR, RL, RR
const int KNEE_PINS[NUM_HIPS]   = { 30, 31, 32, 33 };  // FL, FR, RL, RR
const int HIP_TRIM[NUM_HIPS]    = { 14, 37, 20, 10 };
const int HIP_MIN[NUM_HIPS]     = { 6, 2, 4, 0 };
const int HIP_MAX[NUM_HIPS]     = { 220, 220, 200, 200 };
const int HIP_START[NUM_HIPS]   = { 30, 30, 30, 30 };
const bool HIP_MIRROR[NUM_HIPS] = { false, true, true, false };

const int KNEE_MIN[NUM_HIPS]     = { 0, 0, 20, 20 };
const int KNEE_MAX[NUM_HIPS]     = { 270, 270, 270, 270 };
const int KNEE_START[NUM_HIPS]   = { 125, 130, 135, 135 };
const bool KNEE_MIRROR[NUM_HIPS] = { false, true, false, true };

#define SERVO_PULSE_MIN_US 500
#define SERVO_PULSE_MAX_US 2500

Servo hipServos[NUM_HIPS];
Servo kneeServos[NUM_HIPS];
int   hipPos[NUM_HIPS];
int   kneePos[NUM_HIPS];

void writeHip(int i, int angle) {
  angle = constrain(angle, HIP_MIN[i], HIP_MAX[i]);
  hipPos[i] = angle;
  int trimmed = constrain(angle + HIP_TRIM[i], HIP_MIN[i], HIP_MAX[i]);
  int physical = HIP_MIRROR[i] ? (270 - trimmed) : trimmed;
  int pulse = map(physical, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  hipServos[i].writeMicroseconds(pulse);
}

void writeKnee(int i, int angle) {
  angle = constrain(angle, KNEE_MIN[i], KNEE_MAX[i]);
  kneePos[i] = angle;
  int physical = KNEE_MIRROR[i] ? (270 - angle) : angle;
  int pulse = map(physical, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  kneeServos[i].writeMicroseconds(pulse);
}

// ------------------------------------------------------------
// LEG INVERSE KINEMATICS -- copied from QuadSensors.ino
// ------------------------------------------------------------
const float LEG_THIGH_MM = 165.0;
const float LEG_CALF_MM  = 195.0;

float frontAmountForHeight(float heightMM) {
  float c = (heightMM - LEG_CALF_MM) / LEG_THIGH_MM;
  c = constrain(c, -1.0, 1.0);
  return degrees(acos(c));
}

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

bool computeFrontJointsForHeight(int i, float heightMM, int &hipOut, int &kneeOut) {
  float aFront = frontAmountForHeight(heightMM);
  int hip  = (int)round(HIP_START[i] + aFront);
  int knee = (int)round(KNEE_START[i] - aFront);
  if (hip < HIP_MIN[i] || hip > HIP_MAX[i] || knee < KNEE_MIN[i] || knee > KNEE_MAX[i]) return false;
  hipOut = hip;
  kneeOut = knee;
  return true;
}

bool computeRearJointsForHeight(int i, float heightMM, int &hipOut, int &kneeOut) {
  float bRear = rearAmountForHeight(heightMM);
  int hip  = (int)round(HIP_START[i] - bRear);
  int knee = (int)round(KNEE_START[i] - bRear);
  if (hip < HIP_MIN[i] || hip > HIP_MAX[i] || knee < KNEE_MIN[i] || knee > KNEE_MAX[i]) return false;
  hipOut = hip;
  kneeOut = knee;
  return true;
}

// Same role as QuadSensors.ino's setBodyHeight() -- the "accurate"
// IK-based height command -- but writes directly (no easing) since
// this is a standalone test sketch.
bool commandBodyHeight(float heightMM) {
  int hipFL, kneeFL, hipFR, kneeFR, hipRL, kneeRL, hipRR, kneeRR;
  bool ok = computeFrontJointsForHeight(FL, heightMM, hipFL, kneeFL) &&
            computeFrontJointsForHeight(FR, heightMM, hipFR, kneeFR) &&
            computeRearJointsForHeight(RL, heightMM, hipRL, kneeRL) &&
            computeRearJointsForHeight(RR, heightMM, hipRR, kneeRR);
  if (!ok) return false;
  writeHip(FL, hipFL);   writeKnee(FL, kneeFL);
  writeHip(FR, hipFR);   writeKnee(FR, kneeFR);
  writeHip(RL, hipRL);   writeKnee(RL, kneeRL);
  writeHip(RR, hipRR);   writeKnee(RR, kneeRR);
  return true;
}

// ------------------------------------------------------------
// VL53L0X -- copied from QuadSensors.ino (sensor 2 booted first via
// XSHUT to avoid an address clash, both tuned the same "long range"
// way). Only tof1 is actually read by this test; tof2 is still booted
// because the XSHUT sequencing needs both sensors present to assign
// addresses correctly.
// ------------------------------------------------------------
#define XSHUT_1   A0
#define XSHUT_2   A1
#define TOF2_ADDR 0x52
#define TOF_MAX_MM 2000

VL53L0X tof1, tof2;
bool tof1Active = false, tof2Active = false;
uint16_t tof1_mm = 0;
bool tof1_ok = false;

void setupVL53L0X() {
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  delay(100);

  digitalWrite(XSHUT_2, HIGH);
  delay(100);
  tof2.setBus(&Wire);
  tof2.setTimeout(500);
  if (tof2.init()) {
    tof2.setAddress(TOF2_ADDR);
    tof2.setSignalRateLimit(0.1);
    tof2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
    tof2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    tof2.setMeasurementTimingBudget(50000);
    tof2.startContinuous(100);
    tof2Active = true;
    Serial.println(F("Sensor 2 ready (0x52)."));
  } else {
    Serial.println(F("Sensor 2 not found -- skipping."));
  }

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
    Serial.println(F("Sensor 1 not found -- skipping."));
  }
}

bool tofDataReady(VL53L0X &s) {
  return (s.readReg(VL53L0X::RESULT_INTERRUPT_STATUS) & 0x07) != 0;
}

// Blocking single-shot read (this sketch doesn't run a loop() poll
// cycle) -- waits up to the sensor's own timeout for a fresh sample.
void readTof1() {
  unsigned long start = millis();
  while (!tofDataReady(tof1) && millis() - start < 600) { /* wait */ }
  tof1_mm = tof1.readRangeContinuousMillimeters();
  tof1_ok = !tof1.timeoutOccurred() && tof1_mm > 0 && tof1_mm <= TOF_MAX_MM;
}

// ------------------------------------------------------------
// SERIAL HELPERS
// ------------------------------------------------------------
void waitForAnyKey() {
  while (Serial.available()) Serial.read();
  while (!Serial.available()) { /* wait */ }
  while (Serial.available()) Serial.read();
}

float promptForFloat(const __FlashStringHelper *prompt) {
  Serial.println(prompt);
  String line = "";
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') {
        if (line.length() > 0) break;
      } else {
        line += c;
      }
    }
  }
  return line.toFloat();
}

void printMeanStd(const char *label, float *vals, int n) {
  if (n == 0) {
    Serial.print(label);
    Serial.println(F(",NO_VALID_READINGS"));
    return;
  }
  float sum = 0;
  for (int i = 0; i < n; i++) sum += vals[i];
  float mean = sum / n;
  float sq = 0;
  for (int i = 0; i < n; i++) { float d = vals[i] - mean; sq += d * d; }
  float stddev = sqrt(sq / n);
  Serial.print(label);
  Serial.print(F(",mean=")); Serial.print(mean, 3);
  Serial.print(F(",stddev=")); Serial.println(stddev, 3);
}

// ============================================================
// TEST 1 -- step height estimation accuracy
//
// TEST1_HEIGHT_MIN_MM/MAX_MM/STEP_MM/SETTLE_MS below are new sweep
// parameters chosen for this test (not copied from QuadSensors.ino,
// which paces its own sweep by stand-progress fraction, not raw mm)
// -- adjust if the sweep is too coarse/slow on real hardware.
// TOF1_HEIGHT_ABOVE_HIP_MM and the 150mm jump threshold ARE copied
// from QuadSensors.ino directly.
// ============================================================
#define TEST1_HEIGHT_MIN_MM   150.0
#define TEST1_HEIGHT_MAX_MM   340.0
#define TEST1_HEIGHT_STEP_MM  5.0
#define TEST1_SETTLE_MS       300
#define TEST1_JUMP_THRESHOLD_MM 150 // same as QuadSensors.ino's STEP_CHANGE_THRESHOLD_MM
#define TOF1_HEIGHT_ABOVE_HIP_MM 5.0 // same constant QuadSensors.ino uses
#define TEST1_DELTA_CAL_MM    0.0   // calibration offset, tune once real vs. estimated bias is known
#define TEST1_BLOCKS          3
#define TEST1_SWEEPS_PER_BLOCK 5

// Returns the estimated step height in mm, or NAN if no crossover was found.
float sweepForStepHeight() {
  bool haveBaseline = false;
  uint16_t baseline = 0;

  for (float h = TEST1_HEIGHT_MIN_MM; h <= TEST1_HEIGHT_MAX_MM; h += TEST1_HEIGHT_STEP_MM) {
    if (!commandBodyHeight(h)) continue; // unreachable at this height for at least one leg -- skip
    delay(TEST1_SETTLE_MS);
    readTof1();
    if (!tof1_ok) continue;

    Serial.print(F("TEST1_RAW,")); Serial.print(h, 1); Serial.print(F(",")); Serial.println(tof1_mm);

    if (!haveBaseline) {
      baseline = tof1_mm;
      haveBaseline = true;
      continue;
    }
    if ((int)tof1_mm - (int)baseline > TEST1_JUMP_THRESHOLD_MM) {
      return h + TOF1_HEIGHT_ABOVE_HIP_MM + TEST1_DELTA_CAL_MM;
    }
  }
  return NAN;
}

void runTest1() {
  Serial.println(F("=== TEST 1: step height estimation accuracy ==="));
  for (int block = 0; block < TEST1_BLOCKS; block++) {
    Serial.print(F("Block ")); Serial.print(block + 1); Serial.print(F(" of ")); Serial.println(TEST1_BLOCKS);
    float actualHeight = promptForFloat(F("Set up the obstacle, then enter its actual height in mm:"));

    float estimates[TEST1_SWEEPS_PER_BLOCK];
    int validCount = 0;

    for (int sweep = 0; sweep < TEST1_SWEEPS_PER_BLOCK; sweep++) {
      float est = sweepForStepHeight();
      Serial.print(F("TEST1,")); Serial.print(block + 1); Serial.print(F(","));
      Serial.print(actualHeight, 1); Serial.print(F(","));
      Serial.print(sweep + 1); Serial.print(F(","));
      if (isnan(est)) {
        Serial.println(F("NO_JUMP_DETECTED"));
      } else {
        Serial.println(est, 1);
        estimates[validCount] = est;
        validCount++;
      }
      commandBodyHeight(TEST1_HEIGHT_MIN_MM); // reset low before the next sweep
      delay(TEST1_SETTLE_MS);
    }

    char label[32];
    snprintf(label, sizeof(label), "TEST1_SUMMARY,block%d", block + 1);
    printMeanStd(label, estimates, validCount);

    if (block < TEST1_BLOCKS - 1) {
      Serial.println(F("Send any character to continue to the next block..."));
      waitForAnyKey();
    }
  }
}

// ============================================================
// SETUP / LOOP
// ============================================================
void setup() {
  Serial.begin(115200); // NOTE: QuadSensors.ino runs at 57600 because 115200 was confirmed to drop/stall on this exact hardware -- watch for garbled output and drop to 57600 if it happens here too
  while (!Serial) { /* wait for native USB, harmless no-op on the Mega 2560's hardware UART */ }

  Wire.begin();
  Wire.setWireTimeout(25000, true); // same as QuadSensors.ino -- protects against I2C bus hangs from servo/motor electrical noise

  for (int i = 0; i < NUM_HIPS; i++) {
    hipServos[i].attach(HIP_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    kneeServos[i].attach(KNEE_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  }
  // Boot to the known-safe reference pose (leg straight down) rather
  // than snapping straight into the sweep -- avoids slamming into a
  // mechanical limit before anything has been verified.
  for (int i = 0; i < NUM_HIPS; i++) {
    writeHip(i, HIP_START[i]);
    writeKnee(i, KNEE_START[i]);
  }
  delay(1000);

  setupVL53L0X();

  Serial.println(F("Tier 1 testing ready."));
  Serial.println(F("Send any character to begin Test 1 (step height estimation)..."));
  waitForAnyKey();
  runTest1();

  Serial.println(F("Test 1 complete."));
}

void loop() {
  // Nothing to do -- everything runs once, in setup().
}
