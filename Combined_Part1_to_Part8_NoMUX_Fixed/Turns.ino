/**
 * ====================================================================================================
 * MODULE: SHARP 90-DEGREE TURN & RECOVERY MANEUVERS (PART 05)
 * WITH SAFETY WATCHDOG TIMEOUT CONTROLS
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Executes in-place differential spin pivots when a sharp 90° corner, hairpin bend, or T-junction
 * is confirmed by the PID supervisor.
 * 
 * EXECUTION MECHANICS:
 * 1. Differential Opposite Rotation: Wheels spin in inverse polarities (+L, -R for right turn;
 *    -L, +R for left turn) providing an instantaneous zero-radius pivot.
 * 2. High-Frequency Optical Sampling: Samples sensor array on every iteration of the spin loop.
 * 3. Line Re-Lock: The spin loop terminates immediately upon the center sensor (S3 or S4)
 *    detecting the track, seamlessly handing back control to the PID loop.
 * 4. Safety Watchdog Timeout: If the center sensor does not recapture the track within 1500ms
 *    (e.g., if the robot ran off the mat or was picked up), the loop automatically breaks,
 *    commanding an active electronic brake to avoid runaway spinning.
 */

#include <Arduino.h>

/**
 * Executes an in-place right-hand differential pivot
 * Left wheel forward (+L), Right wheel backward (-R)
 */
void turnRight(int L, int R) {
  unsigned long startTurnTime = millis();

  // Pivot until either center-left (S3) or center-right (S4) detects the line
  while (sensorDigital[4] != 1 && sensorDigital[3] != 1) {
    // WATCHDOG TIMEOUT: Abort spin if line is not recaptured within 1500 ms
    if (millis() - startTurnTime > 1500) {
      Serial.println(F("[WARN] Turn Right Watchdog Timeout Exceeded (Line Not Found)."));
      break;
    }

    motor(L, -R);          // Drive wheels in opposite directions
    read_black_line();     // High-speed direct analog conversion sweep
    direction = "straight";// Clear directional memory
  }

  // Active electronic brake to eliminate rotational inertia
  motor(0, 0);
}

/**
 * Executes an in-place left-hand differential pivot
 * Left wheel backward (-L), Right wheel forward (+R)
 */
void turnLeft(int L, int R) {
  unsigned long startTurnTime = millis();

  // Pivot until either center-left (S3) or center-right (S4) detects the line
  while (sensorDigital[3] != 1 && sensorDigital[4] != 1) {
    // WATCHDOG TIMEOUT: Abort spin if line is not recaptured within 1500 ms
    if (millis() - startTurnTime > 1500) {
      Serial.println(F("[WARN] Turn Left Watchdog Timeout Exceeded (Line Not Found)."));
      break;
    }

    motor(-L, R);          // Drive wheels in opposite directions
    read_black_line();     // High-speed direct analog conversion sweep
    direction = "straight";// Clear directional memory
  }

  // Active electronic brake to eliminate rotational inertia
  motor(0, 0);
}

/**
 * Emergency Hard Stop: Cuts all motor PWM and enters a visual halt blink loop
 */
void hard_stop() {
  while (1) {
    motor(0, 0);
    digitalWrite(led, !digitalRead(led)); // Toggle LED to signal hard halt
    delay(200);
  }
}
