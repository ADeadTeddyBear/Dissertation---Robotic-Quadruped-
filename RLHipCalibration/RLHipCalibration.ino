// ============================================================
// RL HIP SERVO CALIBRATION -- standalone, single-servo sketch
//
// Purpose: after replacing the RL hip servo, find its true
// "straight down" physical angle so HIP_TRIM[RL] in QuadSensors.ino
// (and QuadSensorsRearHipMirror.ino) can be recalibrated for the new
// servo -- the old trim (30, see HIP_TRIM[] in QuadSensors.ino) was
// calibrated for the OLD servo and has no reason to still be correct.
//
// Drives ONLY the RL hip servo, with NO trim applied -- every command
// here is the raw physical angle sent straight to the horn, so
// whatever value you find really is the number to put in HIP_TRIM's
// place (as the new commanded-straight-down reference), exactly the
// same way every other trim in this project was found (see e.g. git
// commit 806fb20: jog until the leg is visually/physically confirmed
// straight down, read back the commanded angle, that IS the new
// reference point).
//
// Uses the exact same 500-2500us / 0-270deg pulse mapping as
// QuadSensors.ino (SERVO_PULSE_MIN_US/MAX_US), so the number you find
// here is directly usable there with no conversion.
//
// Wiring: same RL hip pin as QuadSensors.ino (pin 8). Nothing else
// needs to be connected -- this sketch does not touch the other three
// hips, any knee, the IMU, or either ToF sensor.
// ============================================================

#include <Servo.h>

#define RL_HIP_PIN         8     // matches HIP_RL_PIN in QuadSensors.ino
#define SERVO_PULSE_MIN_US 500   // matches QuadSensors.ino -- 270-degree servo, 500-2500us per datasheet
#define SERVO_PULSE_MAX_US 2500

// Boots to a moderate middle angle, not an extreme (0 or 270) --
// avoids slamming the leg into a hard mechanical stop or the chassis
// before you've had a chance to see where it actually is.
#define START_ANGLE 90

Servo hipRL;
int currentAngle = START_ANGLE;

void goTo(int angle) {
  angle = constrain(angle, 0, 270);
  currentAngle = angle;
  int pulse = map(angle, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  hipRL.writeMicroseconds(pulse);
  Serial.print("RL hip -> ");
  Serial.print(angle);
  Serial.println(" deg (raw, no trim)");
}

void setup() {
  Serial.begin(57600); // matches QuadSensors.ino -- higher rates dropped/stalled on this hardware
  hipRL.attach(RL_HIP_PIN, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  goTo(START_ANGLE);

  Serial.println();
  Serial.println("RL hip servo calibration.");
  Serial.println("Commands:");
  Serial.println("  <number>   -- go straight to that raw angle (0-270), e.g. 90");
  Serial.println("  +          -- nudge +1 deg");
  Serial.println("  -          -- nudge -1 deg");
  Serial.println("  ++ / --    -- nudge +5 / -5 deg");
  Serial.println("Jog until the leg is visually/physically confirmed straight down,");
  Serial.println("then read the printed angle back -- that's the new HIP_TRIM[RL]");
  Serial.println("reference point (same method as every other trim in this project).");
  Serial.println();
}

void loop() {
  if (!Serial.available()) return;
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  if (input == "+") {
    goTo(currentAngle + 1);
  } else if (input == "-") {
    goTo(currentAngle - 1);
  } else if (input == "++") {
    goTo(currentAngle + 5);
  } else if (input == "--") {
    goTo(currentAngle - 5);
  } else {
    bool numeric = true;
    for (unsigned int i = 0; i < input.length(); i++) {
      char c = input.charAt(i);
      if (!isDigit(c) && c != '-') { numeric = false; break; }
    }
    if (numeric) {
      goTo(input.toInt());
    } else {
      Serial.println("Unknown command. Use a number (0-270), +, -, ++, or --.");
    }
  }
}
