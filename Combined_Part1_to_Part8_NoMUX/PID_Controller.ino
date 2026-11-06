/**
 * ====================================================================================================
 * MODULE: CLOSED-LOOP PROPORTIONAL-DERIVATIVE (PD) CONTROLLER & TURN ARBITRATOR
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Performs real-time closed-loop steering control based on the calculated centroid error.
 * Also acts as the supervisory arbitrator for 90° turn state transitions and line loss recovery.
 * 
 * CONTROL FORMULATION:
 *   1. Position Estimation : Position = sum(sensorDigital[i] * Weight[i]) / sum(sensorDigital[i])
 *   2. Error Metric        : Error = Center_Position (45.0) - Position
 *   3. Derivative Term     : Derivative = Error - Previous_Error
 *   4. Steering Output     : Correction = (Kp * Error) + (Kd * Derivative)
 *   5. Motor Actuation     : Left_Speed = Base_Speed - Correction
 *                            Right_Speed = Base_Speed + Correction
 */

#include <Arduino.h>

void PID_Controller(int base_spd, int p, int d) {
  // 1. SAMPLE ALL 8 SENSORS (DIRECT NO-MUX ACQUISITION)
  read_black_line();

  // 2. COMPUTE WEIGHTED CENTROID (Only update if at least one sensor sees the line)
  if (sumOnSensor > 0) {
    line_position = (float)sensorWight / (float)sumOnSensor;
  }
  // Error: >0 means line is to the left of center; <0 means line is to the right
  error = center_position - line_position;

  // 3. TURN PREDICTION PATTERN MATCHING (PART 05 STATE MACHINE)
  // Evaluates binary bitmask to anticipate upcoming 90° bends or acute angles
  switch (bitSensor) {
    // Left-biased patterns (Sensors S0-S3 strongly active)
    case 0b11111000:
    case 0b11110000:
    case 0b11111100:
    case 0b11100000:
      direction = "left";
      break;

    // Right-biased patterns (Sensors S4-S7 strongly active)
    case 0b00011111:
    case 0b00001111:
    case 0b00111111:
    case 0b00000111:
      direction = "right";
      break;

    // Center straight line (Sensors S3 and/or S4 active)
    case 0b00011000:
    case 0b00010000:
    case 0b00001000:
    case 0b00111100:
      direction = "straight";
      break;
  }

  // 4. TURN EXECUTION ON TRACK LOSS (White surface immediately following a biased pattern)
  if (bitSensor == 0) {
    if (direction != "straight") {
      // Turn indicator LED on
      digitalWrite(led, HIGH);
      
      // Delay before pivot: allows robot drive axle to reach the turn vertex
      delay(delay_before_turn);

      if (direction == "right") {
        turnRight(turnSpeed, turnSpeed);
      } else {
        turnLeft(turnSpeed, turnSpeed);
      }

      // Turn indicator LED off
      digitalWrite(led, LOW);
      return; // Return immediately to resume regular PID tracking
    }
  }

  // 5. CALCULATE DERIVATIVE (DAMPING TERM)
  derivative = error - previous_error;

  // 6. CALCULATE PROPORTIONAL + DERIVATIVE CORRECTION
  int correction = (int)((error * (float)p) + (derivative * (float)d));

  // 7. DIFFERENTIAL MOTOR STEERING ASSIGNMENT
  // When error > 0 (line is left of center): left motor slows down, right motor speeds up -> robot turns left
  // When error < 0 (line is right of center): left motor speeds up, right motor slows down -> robot turns right
  int left_motor_correction  = base_spd - correction;
  int right_motor_correction = base_spd + correction;

  // Store error for the next derivative cycle
  previous_error = error;

  // 8. COMMAND MOTOR DRIVER
  motor(left_motor_correction, right_motor_correction);
}
