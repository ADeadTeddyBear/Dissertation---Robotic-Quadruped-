// ============================================================
// WHEEL DRIVE TEST -- standalone, wheels-only sketch
//
// Purpose: troubleshoot the L298N wiring/wheel drive independently of
// the rest of the robot -- no servos, no IMU, no ToF sensors touched
// at all. Uses the exact same pins as QuadSensors.ino's wheel drive
// (WHEEL_*_IN1/IN2/EN), so anything confirmed working here is directly
// usable there with no changes.
//
// Unlike QuadSensors.ino's 'drive' command (which runs for a fixed
// duration then auto-stops), every drive here runs CONTINUOUSLY until
// you send another command -- easier to watch a wheel and adjust
// on the fly while troubleshooting. Always leaves with 'stop' when
// you're done with a test.
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

enum Wheel { FL = 0, FR, RL, RR, NUM_WHEELS };
const char* WHEEL_NAMES[NUM_WHEELS] = { "FL", "FR", "RL", "RR" };
const int WHEEL_IN1_PINS[NUM_WHEELS] = { WHEEL_FL_IN1, WHEEL_FR_IN1, WHEEL_RL_IN1, WHEEL_RR_IN1 };
const int WHEEL_IN2_PINS[NUM_WHEELS] = { WHEEL_FL_IN2, WHEEL_FR_IN2, WHEEL_RL_IN2, WHEEL_RR_IN2 };
const int WHEEL_EN_PINS[NUM_WHEELS]  = { WHEEL_FL_EN,  WHEEL_FR_EN,  WHEEL_RL_EN,  WHEEL_RR_EN  };

int currentSpeed[NUM_WHEELS] = { 0, 0, 0, 0 };

// Sets one wheel's speed: positive = forward, negative = reverse,
// 0 = stop (coasts -- both IN pins low, not an active brake).
void setWheel(int w, int speed) {
  speed = constrain(speed, -255, 255);
  currentSpeed[w] = speed;
  bool forward = speed > 0;
  bool reverse = speed < 0;
  digitalWrite(WHEEL_IN1_PINS[w], forward);
  digitalWrite(WHEEL_IN2_PINS[w], reverse);
  analogWrite(WHEEL_EN_PINS[w], abs(speed));
  Serial.print(WHEEL_NAMES[w]);
  Serial.print(" -> ");
  Serial.println(speed);
}

void stopAll() {
  for (int w = 0; w < NUM_WHEELS; w++) setWheel(w, 0);
  Serial.println("All wheels stopped.");
}

void setup() {
  Serial.begin(57600); // matches QuadSensors.ino -- higher rates dropped/stalled on this hardware
  for (int w = 0; w < NUM_WHEELS; w++) {
    pinMode(WHEEL_IN1_PINS[w], OUTPUT);
    pinMode(WHEEL_IN2_PINS[w], OUTPUT);
    pinMode(WHEEL_EN_PINS[w], OUTPUT);
  }
  stopAll();

  Serial.println();
  Serial.println("Wheel drive test.");
  Serial.println("Commands:");
  Serial.println("  all <speed>   -- all four wheels, e.g. 'all 150' or 'all -150'");
  Serial.println("  fl/fr/rl/rr <speed> -- one wheel only, others left as they are");
  Serial.println("  stop          -- stop every wheel");
  Serial.println("Speed is -255..255 (negative = reverse, 0 = stop). Runs continuously");
  Serial.println("until you send another command -- always end with 'stop'.");
  Serial.println();
}

void loop() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input == "stop") {
    stopAll();
    return;
  }

  int space = input.indexOf(' ');
  if (space <= 0) {
    Serial.println("Unknown command. Use: all/fl/fr/rl/rr <speed>, or stop.");
    return;
  }

  String name = input.substring(0, space);
  int speed = input.substring(space + 1).toInt();

  if (name == "all") {
    for (int w = 0; w < NUM_WHEELS; w++) setWheel(w, speed);
  } else if (name == "fl") {
    setWheel(FL, speed);
  } else if (name == "fr") {
    setWheel(FR, speed);
  } else if (name == "rl") {
    setWheel(RL, speed);
  } else if (name == "rr") {
    setWheel(RR, speed);
  } else {
    Serial.println("Unknown command. Use: all/fl/fr/rl/rr <speed>, or stop.");
  }
}
