/**
 * ====================================================================================================
 * MODULE: BIDIRECTIONAL DIFFERENTIAL MOTOR DRIVER (TB6612FNG / L298N)
 * TARGET HARDWARE: DUAL H-BRIDGE MOTOR DRIVER & ARDUINO NANO
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Controls the speed and direction of two differential DC gearmotors (N20 Micro Metal Gearmotors).
 * Accepts signed speed commands (-255 to +255):
 *   - Positive values (> 0)  : Rotate forward
 *   - Negative values (< 0)  : Rotate reverse (active braking / spin pivot)
 *   - Zero (0)               : Active electronic brake (both direction inputs pulled LOW)
 * 
 * HARDWARE H-BRIDGE LOGIC TABLE:
 * ----------------------------------------------------------------------------------------------------
 * Motor Channel | PWM Speed Pin | Direction Line 1 | Direction Line 2 | Resulting Motion Mode
 * ----------------------------------------------------------------------------------------------------
 * Left Motor    | D5 (PWMB)     | D2 (BIN1) = HIGH | D4 (BIN2) = LOW  | Rotate FORWARD
 * Left Motor    | D5 (PWMB)     | D2 (BIN1) = LOW  | D4 (BIN2) = HIGH | Rotate REVERSE
 * Left Motor    | D5 (PWMB)     | D2 (BIN1) = LOW  | D4 (BIN2) = LOW  | Active Electronic BRAKE
 * ----------------------------------------------------------------------------------------------------
 * Right Motor   | D6 (PWMA)     | D7 (AIN1) = HIGH | D8 (AIN2) = LOW  | Rotate FORWARD
 * Right Motor   | D6 (PWMA)     | D7 (AIN1) = LOW  | D8 (AIN2) = HIGH | Rotate REVERSE
 * Right Motor   | D6 (PWMA)     | D7 (AIN1) = LOW  | D8 (AIN2) = LOW  | Active Electronic BRAKE
 * ----------------------------------------------------------------------------------------------------
 * 
 * IMPORTANT HARDWARE REQUIREMENTS:
 * 1. TB6612FNG STBY (Standby) pin MUST be connected to 5V (HIGH).
 * 2. Motor Driver VM pin requires raw battery voltage (7.4V to 11.1V).
 * 3. Both PWM pins (D5 and D6) are driven by Timer 0 at identical carrier frequency (~976 Hz).
 */

#include <Arduino.h>

void motor(int LPWM, int RPWM) {
  // 1. CLAMP COMMAND SPEEDS TO VALID 8-BIT PWM INTERVAL [-255, +255]
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // 2. CONFIGURE LEFT MOTOR DIRECTION (PINS D2 & D4)
  if (LPWM > 0) {
    // Forward rotation
    digitalWrite(left_motor_forward, HIGH);
    digitalWrite(left_motor_backward, LOW);
  } else if (LPWM < 0) {
    // Reverse rotation
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, HIGH);
  } else {
    // Active electronic brake
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, LOW);
  }

  // 3. CONFIGURE RIGHT MOTOR DIRECTION (PINS D7 & D8)
  if (RPWM > 0) {
    // Forward rotation
    digitalWrite(right_motor_forward, HIGH);
    digitalWrite(right_motor_backward, LOW);
  } else if (RPWM < 0) {
    // Reverse rotation
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, HIGH);
  } else {
    // Active electronic brake
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, LOW);
  }

  // 4. APPLY PWM DUTY CYCLE VIA HARDWARE TIMER 0 (PINS D5 & D6)
  analogWrite(left_motor_speed, abs(LPWM));
  analogWrite(right_motor_speed, abs(RPWM));
}

/**
 * Diagnostic Tool: Spins motors forward, reverse, and in-place pivots with delays.
 * Use this during bench assembly to verify wiring direction and polarity before putting robot on track.
 */
void motor_test() {
  Serial.println(F("\n[MOTOR TEST] Testing Left & Right Forward (Speed = 150)..."));
  motor(150, 150);
  delay(1200);

  Serial.println(F("[MOTOR TEST] Testing Left & Right Reverse (Speed = -150)..."));
  motor(-150, -150);
  delay(1200);

  Serial.println(F("[MOTOR TEST] Testing In-Place Spin (Pivot Right)..."));
  motor(150, -150);
  delay(1000);

  Serial.println(F("[MOTOR TEST] Testing In-Place Spin (Pivot Left)..."));
  motor(-150, 150);
  delay(1000);

  Serial.println(F("[MOTOR TEST] Stopping Motors (Active Electronic Brake)."));
  motor(0, 0);
}
