// ============================================================
// Tier1Testing.ino -- standalone sketch, Test 1 only: step height
// estimation accuracy.
//
// Independent of the other two validation tests (servo repeatability,
// IMU calibration) -- this sketch only touches the legs and the two
// ToF sensors, nothing else.
//
// All pin numbers, trims, joint limits, and the sweep/detection
// algorithm below are copied directly from
// QuadSensorsRearHipMirror.ino (not re-guessed or re-derived):
//   - Hip/knee pins, HIP_TRIM, HIP_MIN/MAX, KNEE_MIN/MAX, HIP_START/
//     KNEE_START, HIP_MIRROR/KNEE_MIRROR: its SERVO CONFIG / KNEE
//     CONFIG sections.
//   - CROUCH_LOW_HIP/KNEE, heightAtStandProgress(),
//     footXAtStandProgress(), applyStandProgress(),
//     FINE_STEP_FRACTION/FINE_STEP_INTERVAL_MS/STEP_CHANGE_THRESHOLD_MM,
//     TOF1_HEIGHT_ABOVE_HIP_MM/TOF1_FORWARD_OFFSET_MM/
//     TOF1_HEIGHT_REF_LEG, and the scan-with-self-motion-compensation
//     crossing detection: its STAND SEQUENCE and STAND SWEEP sections
//     -- this IS the "stand_sweep" command, ported as-is, not a new
//     height-sweep invented for this test.
//   - setupVL53L0X()'s XSHUT sequencing (sensor 2 booted first) and
//     tuning: its VL53L0X SETUP section.
//
// IMPORTANT -- what changed from the original test spec, and why:
//   There is NO downward-facing ToF sensor on this robot. Both ToF1
//   (0x29) and ToF2 (0x52, via XSHUT_2) are mounted level and
//   forward-facing, side by side (ToF1 left-of-centre, ToF2 right-
//   of-centre) -- confirmed by the source file's own square-up
//   comments. So this test cannot use a fixed
//   "H_step = h_s - d_step*sin(45deg)" formula; that sensor doesn't
//   exist. Both sensors are still booted (XSHUT sequencing needs both
//   to avoid an I2C address clash), but only ToF1 is actually read.
//
//   An earlier version of this sketch used setBodyHeight()'s per-height
//   IK formulas (computeFrontJointsForHeight()/
//   computeRearJointsForHeight()) to sweep in raw mm steps. Replaced
//   by request with the REAL stand_sweep mechanism instead -- the
//   source file's own comments note the IK-height-target command
//   ("height <mm>") was actually retired in favour of this
//   angle-interpolation approach because the IK formulas "could reject
//   or misbehave" -- stand_sweep is the one QuadSensorsRearHipMirror.ino
//   still actually uses for step-height estimation today.
//
// Servos are written directly (writeMicroseconds), not through the
// source file's eased-motion system -- movements snap instead of
// easing; explicit settle delays are used instead. There is no
// auto-step-placement here (that's the climb sequence, out of scope
// for this measurement-only test) -- this stops at reporting the
// estimated height.
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <VL53L0X.h>

// ------------------------------------------------------------
// SERVO CONFIG -- copied from QuadSensorsRearHipMirror.ino
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
// LEG GEOMETRY -- copied from QuadSensorsRearHipMirror.ino
// ------------------------------------------------------------
const float LEG_THIGH_MM = 165.0;
const float LEG_CALF_MM  = 195.0;

// ------------------------------------------------------------
// VL53L0X -- copied from QuadSensorsRearHipMirror.ino (sensor 2
// booted first via XSHUT to avoid an address clash, both tuned the
// same "long range" way). Only tof1 is actually read by this test;
// tof2 is still booted because the XSHUT sequencing needs both
// sensors present to assign addresses correctly.
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

// ============================================================
// STAND SEQUENCE (confirmed-low crouch <-> full standing) -- copied
// from QuadSensorsRearHipMirror.ino. standProgress (0 = CROUCH_LOW,
// 1 = full standing at HIP_START/KNEE_START) blends every joint
// through the same single fraction, via plain per-joint linear
// interpolation between confirmed endpoints. Written directly here
// (no easing/gradual standStep() pacing) since this sketch snaps
// servos, matching the rest of this file's simplified style.
// ============================================================
const int CROUCH_LOW_HIP[NUM_HIPS]  = { 140, 140, 140, 140 }; // FL, FR, RL, RR -- confirmed by hand
const int CROUCH_LOW_KNEE[NUM_HIPS] = {  30,  20, 240, 250 }; // FL, FR, RL, RR -- confirmed by hand

#define TOF1_HEIGHT_ABOVE_HIP_MM 5.0    // measured: "a few mm" above the hip-pivot line
#define TOF1_FORWARD_OFFSET_MM   -58.0  // measured with calipers: 58mm BEHIND the front hip pivots
#define TOF1_HEIGHT_REF_LEG      FL     // any leg works (all move identically during the sweep); front leg chosen since ToF1 sits at the front

float standProgress = 0.0; // 0 = CROUCH_LOW stance, 1 = full standing

// Real hip-to-ground height (mm) leg i would have at a given
// standProgress, using the exact CROUCH_LOW<->HIP_START/KNEE_START
// angle interpolation applyStandProgress() itself commands.
float heightAtStandProgress(int i, float progress) {
  float hip  = CROUCH_LOW_HIP[i]  + (HIP_START[i]  - CROUCH_LOW_HIP[i])  * progress;
  float knee = CROUCH_LOW_KNEE[i] + (KNEE_START[i] - CROUCH_LOW_KNEE[i]) * progress;
  float theta1 = radians(hip - HIP_START[i]);
  float theta2 = radians(knee - KNEE_START[i]);
  return LEG_THIGH_MM * cos(theta1) + LEG_CALF_MM * cos(theta1 + theta2);
}

// Same interpolation, but the x (forward) component -- leg i's foot
// position relative to its own hip, fore-aft, at a given standProgress.
// Used to compensate the raw ToF1 reading for the chassis's own
// forward/backward shift as the sweep changes body height.
float footXAtStandProgress(int i, float progress) {
  float hip  = CROUCH_LOW_HIP[i]  + (HIP_START[i]  - CROUCH_LOW_HIP[i])  * progress;
  float knee = CROUCH_LOW_KNEE[i] + (KNEE_START[i] - CROUCH_LOW_KNEE[i]) * progress;
  float theta1 = radians(hip - HIP_START[i]);
  float theta2 = radians(knee - KNEE_START[i]);
  return LEG_THIGH_MM * sin(theta1) + LEG_CALF_MM * sin(theta1 + theta2);
}

void applyStandProgress(float progress) {
  progress = constrain(progress, 0.0, 1.0);
  standProgress = progress;
  for (int i = 0; i < NUM_HIPS; i++) {
    int hip  = (int)round(CROUCH_LOW_HIP[i]  + (HIP_START[i]  - CROUCH_LOW_HIP[i])  * progress);
    int knee = (int)round(CROUCH_LOW_KNEE[i] + (KNEE_START[i] - CROUCH_LOW_KNEE[i]) * progress);
    writeHip(i, hip);
    writeKnee(i, knee);
  }
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
// This is stand_sweep, ported as-is from QuadSensorsRearHipMirror.ino:
// step standProgress up in FINE_STEP_FRACTION (1%) increments, no
// pause between steps beyond FINE_STEP_INTERVAL_MS (matches ToF1's
// own ~100ms continuous-ranging cycle), comparing each self-motion-
// compensated reading against the scan's 0%-baseline. A crossing
// (cumulative drift >= STEP_CHANGE_THRESHOLD_MM, or the target
// disappearing entirely) means ToF1's rising height has cleared the
// step's top edge -- estimated height = heightAtStandProgress() at
// that point + TOF1_HEIGHT_ABOVE_HIP_MM, exactly as the source file
// computes it. TEST1_DELTA_CAL_MM is an extra calibration offset on
// top of that, not part of the original algorithm -- defaults to 0.
// ============================================================
#define FINE_STEP_FRACTION 0.01   // 1% per step -- same as QuadSensorsRearHipMirror.ino
#define FINE_STEP_INTERVAL_MS 100 // matches the ToF's own ~100ms continuous-ranging cycle
#define STEP_CHANGE_THRESHOLD_MM 150 // same as QuadSensorsRearHipMirror.ino
// Derived from ONE hardware calibration point: a 195mm step (physically
// confirmed accurate to 2mm at the actual stopping pose) printed a raw
// estimate of 237.017mm before this offset -- 197 - 237.017 ~= -40.0.
// Crossing detection itself is confirmed correct (chassis height at the
// stop matched the real step within 2mm); the error is entirely in
// heightAtStandProgress()'s %-to-mm conversion at that stand-percentage.
// This offset is ONLY validated at ~195mm so far -- re-check it against
// a different known height before trusting it broadly; if the gap isn't
// constant across heights, a flat offset won't be the real fix.
#define TEST1_DELTA_CAL_MM -40.0
#define TEST1_BLOCKS 3
#define TEST1_SWEEPS_PER_BLOCK 5

// Returns the estimated step height in mm, or NAN if no crossing was
// found across the full 0-100% sweep. Mirrors updateStepScan()'s
// per-step logic, just run to completion in one blocking call instead
// of being paced across loop() ticks.
float sweepForStepHeight() {
  applyStandProgress(0.0);
  delay(500); // let the crouch settle before taking the baseline reading

  readTof1();
  uint16_t baseline = tof1_mm;
  bool baselineOk = tof1_ok;

  while (standProgress < 1.0) {
    applyStandProgress(standProgress + FINE_STEP_FRACTION);
    delay(FINE_STEP_INTERVAL_MS);
    readTof1();

    float chassisShiftMM = footXAtStandProgress(TOF1_HEIGHT_REF_LEG, 0.0)
                          - footXAtStandProgress(TOF1_HEIGHT_REF_LEG, standProgress);
    float compensatedToF1 = (float)tof1_mm + chassisShiftMM;

    Serial.print(F("TEST1_RAW,")); Serial.print((int)round(standProgress * 100));
    Serial.print(F(",")); Serial.print(tof1_ok ? (long)tof1_mm : -1);
    Serial.print(F(",")); Serial.println(compensatedToF1, 1);

    bool crossed = false;
    if (baselineOk) {
      if (!tof1_ok) {
        crossed = true; // target disappeared entirely -- cleared it with nothing behind
      } else if (compensatedToF1 >= (float)baseline + STEP_CHANGE_THRESHOLD_MM) {
        crossed = true;
      }
    }
    if (crossed) {
      return heightAtStandProgress(TOF1_HEIGHT_REF_LEG, standProgress) + TOF1_HEIGHT_ABOVE_HIP_MM + TEST1_DELTA_CAL_MM;
    }
  }
  return NAN;
}

void runTest1() {
  Serial.println(F("=== TEST 1: step height estimation accuracy (stand_sweep) ==="));
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
  Serial.begin(115200); // NOTE: QuadSensorsRearHipMirror.ino runs at 57600 because 115200 was confirmed to drop/stall on this exact hardware -- watch for garbled output and drop to 57600 if it happens here too
  while (!Serial) { /* wait for native USB, harmless no-op on the Mega 2560's hardware UART */ }

  Wire.begin();
  Wire.setWireTimeout(25000, true); // same as the source file -- protects against I2C bus hangs from servo/motor electrical noise

  for (int i = 0; i < NUM_HIPS; i++) {
    hipServos[i].attach(HIP_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    kneeServos[i].attach(KNEE_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  }
  // Boot straight to CROUCH_LOW (standProgress 0), same as
  // QuadSensorsRearHipMirror.ino's enterCrouchLow() -- the
  // hand-verified safe floor, not a theoretical one.
  applyStandProgress(0.0);
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
