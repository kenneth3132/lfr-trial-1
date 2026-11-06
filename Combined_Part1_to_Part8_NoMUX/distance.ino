/**
 * ====================================================================================================
 * MODULE: DISTANCE & TRAJECTORY DEAD-RECKONING (PART 06)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Implements timed open-loop forward movement and braking routines.
 * Useful for calibrating baseline velocity and executing blind traverses over dashed or broken lines.
 * 
 * FORMULA:
 *   Run_Time_ms = (Target_Distance_cm * Test_Time_ms) / Measured_Distance_cm
 */

#include <Arduino.h>

/**
 * Commands the robot to travel forward for an estimated linear distance (in centimeters)
 */
void distance(int cm) {
  if (measured_distance <= 0) measured_distance = 30; // Guard against division by zero

  // Calculate execution time based on benchmarked calibration ratio
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
 * Active brake stop
 */
void stop() {
  motor(0, 0);
}
