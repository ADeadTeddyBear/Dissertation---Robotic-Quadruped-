// ============================================================
// RL HIP SERVO CALIBRATION -- standalone, single-servo sketch
//
// Purpose: after replacing the RL hip servo, find its true
// "straight down" LOGICAL angle (the same kind of number you'd type
// as "hip_rl <angle>" in QuadSensors.ino) so HIP_TRIM[RL] there can be
// recalibrated for the new servo -- the old trim (30, see HIP_TRIM[]
// in QuadSensors.ino) was calibrated for the OLD servo and has no
// reason to still be correct.
//
// Mirrors QuadSensors.ino's applyHipAngle() exactly: physical = mirror
// ? (270 - angle) : angle, then mapped 0-270deg -> 500-2500us. Starts
// with mirror ON, matching HIP_MIRROR[RL]=true in QuadSensors.ino
// today -- toggle it with 'm' if the replacement servo turns out to
// spin the opposite way to the old one (which is exactly what you
// reported seeing). Trim is 0 here on purpose: whatever LOGICAL angle
// you find straight-down at, minus HIP_START[RL] (30 in QuadSensors.ino),
// IS the new HIP_TRIM[RL] value, the same way every other trim in this
// project was found (jog to a known reference pose, read back the
// commanded angle -- see e.g. git commit 806fb20).
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
bool mirror = true; // matches HIP_MIRROR[RL] in QuadSensors.ino today

void goTo(int angle) {
  angle = constrain(angle, 0, 270);
  currentAngle = angle;
  int physical = mirror ? (270 - angle) : angle;
  int pulse = map(physical, 0, 270, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  hipRL.writeMicroseconds(pulse);
  Serial.print("RL hip -> ");
  Serial.print(angle);
  Serial.print(" deg (logical, mirror=");
  Serial.print(mirror ? "ON" : "OFF");
  Serial.println(")");
}

void setup() {
  Serial.begin(57600); // matches QuadSensors.ino -- higher rates dropped/stalled on this hardware
  hipRL.attach(RL_HIP_PIN, SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US);
  goTo(START_ANGLE);

  Serial.println();
  Serial.println("RL hip servo calibration.");
  Serial.println("Commands:");
  Serial.println("  <number>   -- go straight to that logical angle (0-270), e.g. 90");
  Serial.println("  +          -- nudge +1 deg");
  Serial.println("  -          -- nudge -1 deg");
  Serial.println("  ++ / --    -- nudge +5 / -5 deg");
  Serial.println("  m          -- toggle mirror (matches HIP_MIRROR[RL] in QuadSensors.ino)");
  Serial.println("Jog until, with increasing angle, the thigh swings FORWARD (same as");
  Serial.println("every other leg) and the leg is visually/physically straight down at");
  Serial.println("some angle -- that angle minus 30 (HIP_START[RL]) is the new");
  Serial.println("HIP_TRIM[RL]. Whichever mirror setting gets forward-on-increase is");
  Serial.println("also the new HIP_MIRROR[RL] value.");
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
  } else if (input == "m") {
    mirror = !mirror;
    goTo(currentAngle); // re-apply at the same logical angle under the new mirror setting
  } else {
    bool numeric = true;
    for (unsigned int i = 0; i < input.length(); i++) {
      char c = input.charAt(i);
      if (!isDigit(c) && c != '-') { numeric = false; break; }
    }
    if (numeric) {
      goTo(input.toInt());
    } else {
      Serial.println("Unknown command. Use a number (0-270), +, -, ++, --, or m.");
    }
  }
}
