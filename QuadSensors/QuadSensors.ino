// ============================================================
// QuadSensors.ino
// Hip servo control + MPU6050 + dual VL53L1X (TOF400C)
//
// Libraries required (install via Library Manager):
//   VL53L1X  by Pololu
//   (MPU6050 driven via raw Wire — no extra library needed)
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <VL53L1X.h>

// ============================================================
// HIP SERVO PINS
// ============================================================
#define HIP_FL_PIN   6
#define HIP_FR_PIN   5
#define HIP_RL_PIN   7
#define HIP_RR_PIN   8

// ============================================================
// VL53L1X XSHUT PINS
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
const int  HIP_MIN[NUM_HIPS]    = {   0,   0,   0,   0 };
const int  HIP_MAX[NUM_HIPS]    = { 270, 270, 270, 270 };
const int  HIP_START[NUM_HIPS]  = { 135, 135, 135, 135 };
// Right side servos are mounted opposite — mirror their angle so
// sending 135 to FL and FR both means "centred"
const bool HIP_MIRROR[NUM_HIPS] = { false, true, false, true }; // FL, FR, RL, RR

const char* HIP_NAMES[NUM_HIPS] = { "hip_fl", "hip_fr", "hip_rl", "hip_rr" };

Servo hipServos[NUM_HIPS];
int   hipPos[NUM_HIPS];

// ============================================================
// SENSOR OBJECTS
// ============================================================
VL53L1X tof1, tof2;
bool tof1Active = false;
bool tof2Active = false;

uint16_t tof1_mm = 0, tof2_mm = 0;
uint8_t  tof1_status = VL53L1X::None, tof2_status = VL53L1X::None;
float    tof1_sig = 0, tof2_sig = 0;   // peak signal rate, MCPS
float    tof1_amb = 0, tof2_amb = 0;   // ambient rate, MCPS

#define FIRMWARE_BUILD "QuadSensors build 2026-07-23-c (dataReady poll + signal/ambient debug)"

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
  int physical = HIP_MIRROR[i] ? (270 - angle) : angle;
  hipServos[i].write(physical);
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
// VL53L1X SETUP — sensor 2 booted first to avoid address clash
// ============================================================
void setupVL53L1X() {
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
    // Long mode's VCSEL period is unambiguous only from ~40cm out — closer
    // targets fold into a shorter apparent range and can even count down as
    // the real target recedes. Short mode is unambiguous over its full
    // 0-1300mm span, which covers this sensor's mounted use case.
    tof2.setDistanceMode(VL53L1X::Short);
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
    tof1.setDistanceMode(VL53L1X::Short);
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
// VL53L1X::read(false) skips the dataReady() wait itself and
// unconditionally clears the ranging interrupt, so calling it on a
// fixed timer that isn't synced to the sensor's own measurement
// cadence can catch it mid-update and hand back a torn register
// read — which shows up as bogus, usually near-field, distance
// values that get worse the weaker (longer-range) the real signal
// is. Gating on dataReady() ourselves, every loop() pass, avoids
// that race; printSensors() just reports the latest cached result.
// ============================================================
void pollTofSensors() {
  if (tof1Active && tof1.dataReady()) {
    tof1_mm     = tof1.read(false);
    tof1_status = tof1.ranging_data.range_status;
    tof1_sig    = tof1.ranging_data.peak_signal_count_rate_MCPS;
    tof1_amb    = tof1.ranging_data.ambient_count_rate_MCPS;
  }
  if (tof2Active && tof2.dataReady()) {
    tof2_mm     = tof2.read(false);
    tof2_status = tof2.ranging_data.range_status;
    tof2_sig    = tof2.ranging_data.peak_signal_count_rate_MCPS;
    tof2_amb    = tof2.ranging_data.ambient_count_rate_MCPS;
  }
}

// ============================================================
// PRINT SENSOR VALUES
// A VL53L1X reading is only trustworthy when range_status == RangeValid —
// checking it rejects wrapped/low-confidence reads (see WrapTargetFail in
// setupVL53L1X()) instead of reporting a bogus, decreasing number.
// ============================================================
void printSensors() {
  float pitch, roll;
  readMPU6050(pitch, roll);

  Serial.print("Pitch:"); Serial.print(pitch, 1);
  Serial.print("  Roll:"); Serial.print(roll, 1);

  if (tof1Active) {
    if (tof1_mm == 0 || tof1_mm == 65535 || tof1_status != VL53L1X::RangeValid) {
      Serial.print("  ToF1:---(st"); Serial.print(tof1_status); Serial.print(")");
    } else {
      Serial.print("  ToF1:"); Serial.print(tof1_mm); Serial.print("mm");
    }
    Serial.print(" [sig="); Serial.print(tof1_sig, 2);
    Serial.print(" amb="); Serial.print(tof1_amb, 2); Serial.print("]");
  } else {
    Serial.print("  ToF1:N/A");
  }

  if (tof2Active) {
    if (tof2_mm == 0 || tof2_mm == 65535 || tof2_status != VL53L1X::RangeValid) {
      Serial.print("  ToF2:---(st"); Serial.print(tof2_status); Serial.print(")");
    } else {
      Serial.print("  ToF2:"); Serial.print(tof2_mm); Serial.print("mm");
    }
    Serial.print(" [sig="); Serial.print(tof2_sig, 2);
    Serial.print(" amb="); Serial.print(tof2_amb, 2); Serial.print("]");
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
    Serial.println("Commands: start | all <angle> | hip_fl/fr/rl/rr <angle> | sensors | help");
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
    hipServos[i].attach(HIP_PINS[i]);
    hipPos[i] = HIP_START[i];
    hipServos[i].write(hipPos[i]);
  }
  Serial.println("Servos OK");

  setupVL53L1X();
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
