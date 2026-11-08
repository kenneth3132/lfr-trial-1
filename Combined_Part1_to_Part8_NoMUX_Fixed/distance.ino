/**
 * ====================================================================================================
 * MODULE: DISTANCE & TRAJECTORY DEAD-RECKONING (PART 06)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Implements timed open-loop forward movement and active electronic braking routines.
 * Useful for calibrating baseline velocity and executing blind traverses over dashed or broken lines.
 * 
 * MATHEMATICAL CALIBRATION FORMULA:
 *   Run_Time_ms = (Target_Distance_cm * Test_Time_ms) / Measured_Distance_cm
 * 
 * PARAMETERS:
 *   - test_time         : Duration of open-loop test pulse in milliseconds (e.g., 500 ms)
 *   - measured_distance : Distance traveled during the benchmark pulse in centimeters (e.g., 30 cm)
 */

#include <Arduino.h>

/**
 * Commands the robot to travel forward for an estimated linear distance (in centimeters)
 */
void distance(int cm) {
  // Guard against division by zero
  if (measured_distance <= 0) {
    measured_distance = 30;
  }

  // Calculate required motor run time based on benchmark calibration ratio
  long run_time = ((long)cm * (long)test_time) / (long)measured_distance;

  Serial.print(F("[DISTANCE] Driving "));
  Serial.print(cm);
  Serial.print(F(" cm for "));
  Serial.print(run_time);
  Serial.println(F(" ms..."));

  // Drive forward at nominal base speed
  motor(base_speed, base_speed);
  delay(run_time);

  // Active electronic braking stop
  motor(0, 0);
  Serial.println(F("[DISTANCE] Motion complete."));
}

/**
 * Active Electronic Brake Stop: Cuts PWM to zero and pulls direction pins LOW
 */
void stop() {
  motor(0, 0);
}
