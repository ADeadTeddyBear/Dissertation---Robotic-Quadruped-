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
#define HIP_RL_PIN   7
#define HIP_RR_PIN   8

// ============================================================
// KNEE SERVO PINS
// Only FL is wired so far; FR/RL/RR follow the same pattern once
// their legs are physically added.
// ============================================================
#define KNEE_FL_PIN  30

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
// FL is mechanically limited to 6 as its furthest inward point -- going
// lower risks the leg colliding with/damaging the robot.
const int  HIP_MIN[NUM_HIPS]    = {   6,   0,   0,   0 };
// 170 = leg straight up; capped there (rather than the servo's full 270)
// so it can't swing past vertical and clash with the top of the robot.
const int  HIP_MAX[NUM_HIPS]    = { 170, 170, 170, 170 };
// 30 = leg straight down (the home pose for IK); 0-30 lets the leg swing
// inward a bit from there, 30-170 covers the rest of its outward/upward travel.
const int  HIP_START[NUM_HIPS]  = { 30, 30, 30, 30 };
// Right side servos are mounted opposite — mirror their angle so
// sending 30 to FL and FR both means "straight down"
const bool HIP_MIRROR[NUM_HIPS] = { false, true, false, true }; // FL, FR, RL, RR
// Per-servo calibration: added before mirroring so commanding the same
// logical angle (e.g. 30) points every leg straight down, regardless of
// how each servo horn happens to be seated. Fill in from the by-eye
// calibration: trim = (angle that looked straight down) - 30.
// FL calibrated: 44 looked straight down -> trim +14.
// FR calibrated: 67 looked straight down -> trim +37.
// RL calibrated: 43 looked straight down -> trim +13.
// RR calibrated: 63 looked straight down -> trim +33.
const int  HIP_TRIM[NUM_HIPS]   = { 14, 37, 13, 33 };

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
// KNEE CONFIG — FL only for now
// Same servo model as the hips (500-2500us, 270 degrees), so the
// same pulse mapping applies. Full 0-270 range confirmed safe by
// testing; 140 is straight down (the home pose).
// ============================================================
const int KNEE_FL_MIN   = 0;
const int KNEE_FL_MAX   = 270;
const int KNEE_FL_START = 140; // confirmed straight down

Servo kneeFL;
int   kneeFLPos;

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
void setHip(int i, int angle) {
  angle = constrain(angle, HIP_MIN[i], HIP_MAX[i]);
  hipPos[i] = angle;
  int trimmed = constrain(angle + HIP_TRIM[i], HIP_MIN[i], HIP_MAX[i]);
  int physical = HIP_MIRROR[i] ? (270 - trimmed) : trimmed;
  int pulse = map(physical, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  hipServos[i].writeMicroseconds(pulse);
}

void setKneeFL(int angle) {
  angle = constrain(angle, KNEE_FL_MIN, KNEE_FL_MAX);
  kneeFLPos = angle;
  int pulse = map(angle, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  kneeFL.writeMicroseconds(pulse);
}

void allHips(int angle) {
  for (int i = 0; i < NUM_HIPS; i++) setHip(i, angle);
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
    Serial.println("Commands: start | all <angle> | hip_fl/fr/rl/rr <angle> | knee_fl <angle> | sensors | help");
    Serial.println();

  } else if (input.startsWith("all ")) {
    int angle = input.substring(4).toInt();
    allHips(angle);
    Serial.print("All hips -> "); Serial.println(angle);

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
      if (!found && name == "knee_fl") {
        setKneeFL(angle);
        Serial.print("knee_fl -> "); Serial.println(kneeFLPos);
        found = true;
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
  Serial.begin(115200);
  delay(2000);
  Serial.println(FIRMWARE_BUILD);
  Serial.println("Booting...");
  Wire.begin();

  for (int i = 0; i < NUM_HIPS; i++) {
    hipServos[i].attach(HIP_PINS[i], SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
    setHip(i, HIP_START[i]);
  }
  kneeFL.attach(KNEE_FL_PIN, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  setKneeFL(KNEE_FL_START);
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
