/**
 * ====================================================================================================
 * MODULE: BIDIRECTIONAL DIFFERENTIAL MOTOR DRIVER (TB6612FNG / L298N)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Controls speed and directional rotation of the two differential drive DC gearmotors.
 * Accepts signed speed commands (-255 to +255):
 *   - Positive values (> 0)  : Rotate forward
 *   - Negative values (< 0)  : Rotate reverse (active braking / spin pivot)
 *   - Zero (0)               : Active motor stop / brake
 * 
 * HARDWARE H-BRIDGE LOGIC TABLE:
 * -------------------------------------------------------------------------
 * Motor | PWM Pin | IN1/IN3 | IN2/IN4 | Direction Mode
 * -------------------------------------------------------------------------
 * Left  | D5      | HIGH    | LOW     | Forward
 * Left  | D5      | LOW     | HIGH    | Reverse
 * Left  | D5      | LOW     | LOW     | Stop / Coast
 * Right | D6      | HIGH    | LOW     | Forward
 * Right | D6      | LOW     | HIGH    | Reverse
 * Right | D6      | LOW     | LOW     | Stop / Coast
 * -------------------------------------------------------------------------
 */

#include <Arduino.h>

void motor(int LPWM, int RPWM) {
  // 1. CLAMP SPEEDS TO VALID 8-BIT PWM INTERVAL [-255, +255]
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // 2. CONFIGURE LEFT MOTOR DIRECTION
  if (LPWM > 0) {
    digitalWrite(left_motor_forward, HIGH);
    digitalWrite(left_motor_backward, LOW);
  } else if (LPWM < 0) {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, HIGH);
  } else {
    // Active electronic braking
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, LOW);
  }

  // 3. CONFIGURE RIGHT MOTOR DIRECTION
  if (RPWM > 0) {
    digitalWrite(right_motor_forward, HIGH);
    digitalWrite(right_motor_backward, LOW);
  } else if (RPWM < 0) {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, HIGH);
  } else {
    // Active electronic braking
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, LOW);
  }

  // 4. APPLY PWM DUTY CYCLE VIA HARDWARE TIMER 0 (Pins D5 & D6)
  analogWrite(left_motor_speed, abs(LPWM));
  analogWrite(right_motor_speed, abs(RPWM));
}

/**
 * Diagnostic utility: Spins motors forward, reverse, and stops with delay
 * Use this to verify correct H-bridge wiring and forward direction polarity.
 */
void motor_test() {
  Serial.println(F("[MOTOR TEST] Testing Left & Right Forward (Speed = 150)..."));
  motor(150, 150);
  delay(1000);

  Serial.println(F("[MOTOR TEST] Testing Left & Right Reverse (Speed = -150)..."));
  motor(-150, -150);
  delay(1000);

  Serial.println(F("[MOTOR TEST] Testing In-Place Spin (Pivot Right)..."));
  motor(150, -150);
  delay(1000);

  Serial.println(F("[MOTOR TEST] Testing In-Place Spin (Pivot Left)..."));
  motor(-150, 150);
  delay(1000);

  Serial.println(F("[MOTOR TEST] Stopping Motors."));
  motor(0, 0);
}
