/**
 * ====================================================================================================
 * MODULE: SHARP 90-DEGREE TURN & RECOVERY MANEUVERS (PART 05)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Executes in-place differential pivot maneuvers when a sharp 90° angle, hairpin bend, or T-junction
 * is confirmed by the PID arbitrator.
 * 
 * EXECUTION MECHANICS:
 * 1. Pivot Spin: Drives wheels in opposite directions (Left forward / Right reverse for right turn).
 * 2. Continuous Scanning: Samples the sensor array on every iteration of the spin loop.
 * 3. Line Re-Lock: The spin loop terminates the instant the center sensor (Sensor 3 or 4) recaptures
 *    the line, returning control seamlessly to the PID loop.
 * 4. State Reset: Clears direction memory back to "straight".
 */

#include <Arduino.h>

/**
 * Executes an in-place right-hand pivot maneuver
 * Left wheel forward (+L), Right wheel backward (-R)
 */
void turnRight(int L, int R) {
  // Pivot until either center-left (S3) or center-right (S4) detects the line
  while (sensorDigital[4] != 1 && sensorDigital[3] != 1) {
    motor(L, -R);          // Differential opposite wheel rotation
    read_black_line();     // High-speed direct analog sampling
    direction = "straight";// Reset directional state memory
  }
  // Brief stabilization brake
  motor(0, 0);
}

/**
 * Executes an in-place left-hand pivot maneuver
 * Left wheel backward (-L), Right wheel forward (+R)
 */
void turnLeft(int L, int R) {
  // Pivot until either center-left (S3) or center-right (S4) detects the line
  while (sensorDigital[3] != 1 && sensorDigital[4] != 1) {
    motor(-L, R);          // Differential opposite wheel rotation
    read_black_line();     // High-speed direct analog sampling
    direction = "straight";// Reset directional state memory
  }
  // Brief stabilization brake
  motor(0, 0);
}

/**
 * Emergency hard stop: Cuts PWM output to both motors and enters infinite halt loop
 */
void hard_stop() {
  while (1) {
    motor(0, 0);
    digitalWrite(led, !digitalRead(led)); // Blink LED to signal halt
    delay(200);
  }
}
