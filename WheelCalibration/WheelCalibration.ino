// ============================================================
// WHEEL CALIBRATION -- standalone, single-purpose sketch
//
// Purpose: measure real mm-per-ms at the exact PWM speed raise_rear
// (and other timed drives) use, by lifting FL clear of the ground so
// its wheel spins free and easy to watch/count (the other three
// wheels stay grounded and driven -- this IS the robot's real loaded
// driving speed, not a no-load bench spin), then driving for a known,
// fixed duration. Count how many full rotations FL's wheel makes in
// that window and report it back -- with the wheel's known ~90mm
// diameter (45mm radius), that gives real mm-per-ms at this PWM
// level, letting timed drives target an actual distance instead of a
// blind fixed-duration pulse. See QuadSensors.ino's RAISE REAR
// section for where that number will actually get used.
//
// SAFETY: this sketch only attaches FL's hip/knee servos and the
// wheel driver pins -- same as WheelDriveTest.ino, it does NOT attach
// or hold FR/RL/RR's servos, so they go limp (no holding torque) the
// moment this uploads. Only run this with the robot resting normally
// on all four feet on the ground, or otherwise safely supported --
// NOT mid-climb, NOT with any leg already lifted.
//
// Wiring: same FL hip/knee pins and wheel driver pins QuadSensors.ino
// uses. Nothing else needs to be connected -- this sketch does not
// touch FR/RL/RR, the IMU, or either ToF sensor.
// ============================================================

#include <Servo.h>

#define HIP_FL_PIN         6
#define KNEE_FL_PIN        30
#define SERVO_PULSE_MIN_US 500   // matches QuadSensors.ino -- 270-degree servo, 500-2500us per datasheet
#define SERVO_PULSE_MAX_US 2500

// FL-specific calibration, copied from QuadSensors.ino so this
// produces the exact same physical angles -- FL needs no HIP_MIRROR/
// KNEE_MIRROR (both false for FL there), so the mapping is direct.
#define HIP_TRIM_FL   14
#define HIP_MIN_FL    6
#define HIP_MAX_FL    220

// Same lifted/tucked pose LIFT_KNEE_SAFE/LIFT_TUCK put FL into during
// a real climb sequence -- hand-verified there, reused here so this
// test's load/geometry matches the real thing as closely as possible.
#define LIFT_LIFTED_HIP_FL  150
#define LIFT_SAFE_KNEE_FL   270

// The "normal standing" reference to return FL to with 'lower'.
#define PRECLIMB_HIP_FL   92
#define PRECLIMB_KNEE_FL  100

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

enum Wheel { FL = 0, FR, RL, RR, NUM_WHEELS };
const char* WHEEL_NAMES[NUM_WHEELS] = { "FL", "FR", "RL", "RR" };
const int WHEEL_IN1_PINS[NUM_WHEELS] = { WHEEL_FL_IN1, WHEEL_FR_IN1, WHEEL_RL_IN1, WHEEL_RR_IN1 };
const int WHEEL_IN2_PINS[NUM_WHEELS] = { WHEEL_FL_IN2, WHEEL_FR_IN2, WHEEL_RL_IN2, WHEEL_RR_IN2 };
const int WHEEL_EN_PINS[NUM_WHEELS]  = { WHEEL_FL_EN,  WHEEL_FR_EN,  WHEEL_RL_EN,  WHEEL_RR_EN  };

// Same fix as QuadSensors.ino/WheelDriveTest.ino -- front/rear axles
// are mounted as mirror images of each other.
const bool WHEEL_REVERSED[NUM_WHEELS] = { true, true, false, false };

// Matches RAISE_REAR_DRIVE_SPEED in QuadSensors.ino -- the whole point
// is to measure real mm-per-ms AT this specific speed.
#define TEST_DRIVE_SPEED 120
#define DEFAULT_DRIVE_MS 5000

Servo hipFL, kneeFL;
int hipAngleFL = 0, kneeAngleFL = 0;

void goHip(int angle) {
  angle = constrain(angle, HIP_MIN_FL, HIP_MAX_FL);
  hipAngleFL = angle;
  int trimmed = constrain(angle + HIP_TRIM_FL, HIP_MIN_FL, HIP_MAX_FL);
  int pulse = map(trimmed, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  hipFL.writeMicroseconds(pulse);
}

void goKnee(int angle) {
  angle = constrain(angle, 0, 270);
  kneeAngleFL = angle;
  int pulse = map(angle, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  kneeFL.writeMicroseconds(pulse);
}

void setWheel(int w, int speed) {
  speed = constrain(speed, -255, 255);
  bool forward = speed > 0;
  bool reverse = speed < 0;
  if (WHEEL_REVERSED[w]) { bool t = forward; forward = reverse; reverse = t; }
  digitalWrite(WHEEL_IN1_PINS[w], forward);
  digitalWrite(WHEEL_IN2_PINS[w], reverse);
  analogWrite(WHEEL_EN_PINS[w], abs(speed));
}

void setAllWheels(int speed) {
  for (int w = 0; w < NUM_WHEELS; w++) setWheel(w, speed);
}

void stopAllWheels() {
  setAllWheels(0);
  Serial.println("Wheels stopped.");
}

void doLift() {
  goHip(LIFT_LIFTED_HIP_FL);
  goKnee(LIFT_SAFE_KNEE_FL);
  Serial.println("FL lifting clear of the ground -- give it a couple seconds to settle.");
  delay(2000); // simple, generous settle time -- no move-duration tracking in this sketch, unlike QuadSensors.ino
}

void doLower() {
  goHip(PRECLIMB_HIP_FL);
  goKnee(PRECLIMB_KNEE_FL);
  Serial.println("FL lowering back to a normal standing angle.");
  delay(2000);
}

void doDrive(unsigned long durationMs) {
  Serial.print("Driving all four wheels at "); Serial.print(TEST_DRIVE_SPEED);
  Serial.print(" for "); Serial.print(durationMs);
  Serial.println("ms -- watch FL's wheel and count full rotations now.");
  setAllWheels(TEST_DRIVE_SPEED);
  delay(durationMs);
  stopAllWheels();
  Serial.println("Drive complete -- tell me how many rotations FL's wheel made in that window.");
}

void setup() {
  Serial.begin(57600); // matches QuadSensors.ino -- higher rates dropped/stalled on this hardware
  hipFL.attach(HIP_FL_PIN, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  kneeFL.attach(KNEE_FL_PIN, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  for (int w = 0; w < NUM_WHEELS; w++) {
    pinMode(WHEEL_IN1_PINS[w], OUTPUT);
    pinMode(WHEEL_IN2_PINS[w], OUTPUT);
    pinMode(WHEEL_EN_PINS[w], OUTPUT);
  }
  stopAllWheels();
  goHip(PRECLIMB_HIP_FL);
  goKnee(PRECLIMB_KNEE_FL);

  Serial.println();
  Serial.println("Wheel calibration test.");
  Serial.println("SAFETY: only FL's servos and the wheels are attached here -- FR/RL/RR");
  Serial.println("have no holding torque in this sketch. Only run this with the robot");
  Serial.println("resting normally on all four feet, not mid-climb.");
  Serial.println("Commands:");
  Serial.println("  lift          -- lift FL clear of the ground");
  Serial.println("  lower         -- lower FL back to standing");
  Serial.println("  drive <ms>    -- drive all four wheels for <ms> milliseconds (default 5000)");
  Serial.println("  cal <ms>      -- lift, then drive for <ms> milliseconds -- the one-shot version");
  Serial.println("  stop          -- stop the wheels immediately");
  Serial.println();
}

void loop() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input == "lift") {
    doLift();
  } else if (input == "lower") {
    doLower();
  } else if (input == "stop") {
    stopAllWheels();
  } else if (input == "drive") {
    doDrive(DEFAULT_DRIVE_MS);
  } else if (input.startsWith("drive ")) {
    doDrive((unsigned long)input.substring(6).toInt());
  } else if (input == "cal") {
    doLift();
    doDrive(DEFAULT_DRIVE_MS);
  } else if (input.startsWith("cal ")) {
    doLift();
    doDrive((unsigned long)input.substring(4).toInt());
  } else {
    Serial.println("Unknown command. Use: lift, lower, drive <ms>, cal <ms>, or stop.");
  }
}
