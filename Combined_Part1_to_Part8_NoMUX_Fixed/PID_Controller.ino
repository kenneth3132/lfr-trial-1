/**
 * ====================================================================================================
 * MODULE: CLOSED-LOOP PROPORTIONAL-DERIVATIVE (PD) CONTROLLER & TURN ARBITRATOR
 * TARGET HARDWARE: ARDUINO NANO & TB6612FNG / L298N DUAL H-BRIDGE
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Performs real-time closed-loop steering control based on calculated centroid error.
 * Acts as the supervisory arbitrator for 90° sharp corners, acute angles, and off-track safety.
 * 
 * CONTROL THEORY & FORMULATION:
 *   1. Position Estimation : Position = sum(sensorDigital[i] * Weight[i]) / sum(sensorDigital[i])
 *                            Center Setpoint = 45.0 (Midpoint between 10 and 80)
 *   2. Error Metric        : Error = Center_Position (45.0) - Position
 *                            - Positive Error: Robot is right of line (line is to left) -> Turn Left
 *                            - Negative Error: Robot is left of line (line is to right) -> Turn Right
 *   3. Derivative Term     : Derivative = Error - Previous_Error (dampens rapid oscillations)
 *   4. Steering Output     : Correction = (Kp * Error) + (Kd * Derivative)
 *   5. Motor Speeds        : Left_Speed  = Base_Speed - Correction
 *                            Right_Speed = Base_Speed + Correction
 * 
 * ENHANCEMENTS IN THIS REVISED MODULE:
 * - Direction state memory is properly cleared to "straight" across all normal tracking patterns,
 *   preventing sticky turn memory from causing false turns on subsequent gaps.
 * - Off-track watchdog: if all sensors report white for > 350ms on a straight line, motors safely stop.
 * - Symmetrical forward creep during turn delay ensures wheel axle reaches turn vertex before pivoting.
 */

#include <Arduino.h>

void PID_Controller(int base_spd, int p, int d) {
  // --------------------------------------------------------------------------------------------------
  // 1. SAMPLE ALL 8 SENSORS VIA HIGH-SPEED DIRECT ANALOG CONVERSION
  // --------------------------------------------------------------------------------------------------
  read_black_line();

  // --------------------------------------------------------------------------------------------------
  // 2. COMPUTE WEIGHTED CENTROID & CENTERING ERROR
  // --------------------------------------------------------------------------------------------------
  if (sumOnSensor > 0) {
    // At least one sensor detects the line: calculate centroid position
    line_position = (float)sensorWight / (float)sumOnSensor;
    error = center_position - line_position;
    
    // Clear off-track timer since the line is actively visible
    offTrackStartTime = 0;
  } else {
    // No sensors see the line: track is lost or crossing a gap
    if (offTrackStartTime == 0) {
      offTrackStartTime = millis(); // Record timestamp when line was first lost
    }
  }

  // --------------------------------------------------------------------------------------------------
  // 3. TURN PREDICTION PATTERN MATCHING & DIRECTION STATE MEMORY (PART 05)
  // Evaluates binary bitmask to distinguish between normal curves and sharp 90° turn footprints
  // --------------------------------------------------------------------------------------------------
  switch (bitSensor) {
    // WIDE LEFT-BIASED PATTERNS (Sensors S0-S3 heavily active -> Upcoming sharp 90° left turn)
    case 0b11111000:
    case 0b11110000:
    case 0b11111100:
    case 0b11100000:
    case 0b11000000: // S0, S1 active
      direction = "left";
      break;

    // WIDE RIGHT-BIASED PATTERNS (Sensors S4-S7 heavily active -> Upcoming sharp 90° right turn)
    case 0b00011111:
    case 0b00001111:
    case 0b00111111:
    case 0b00000111:
    case 0b00000011: // S6, S7 active
      direction = "right";
      break;

    // NORMAL CENTERED & MILD CURVE PATTERNS (Reset direction memory to "straight"!)
    // Expanding these cases eliminates "sticky" direction memory bugs from previous turns
    case 0b00011000: // S3, S4 centered
    case 0b00010000: // S3 only
    case 0b00001000: // S4 only
    case 0b00111100: // S2, S3, S4, S5 centered
    case 0b00110000: // S2, S3 mild left drift
    case 0b00001100: // S4, S5 mild right drift
    case 0b01100000: // S1, S2 moderate left
    case 0b00000110: // S5, S6 moderate right
    case 0b00100000: // S2 only
    case 0b00000100: // S5 only
    case 0b01110000: // S1, S2, S3
    case 0b00001110: // S4, S5, S6
      direction = "straight";
      break;
  }

  // --------------------------------------------------------------------------------------------------
  // 4. SHARP TURN EXECUTION ON TRACK LOSS (WHITE SPACE IMMEDIATELY FOLLOWING A BIASED PATTERN)
  // --------------------------------------------------------------------------------------------------
  if (bitSensor == 0) {
    if (direction != "straight") {
      // Turn indicator LED illuminated
      digitalWrite(led, HIGH);

      // Controlled forward creep: moves drive axle directly over the corner vertex
      motor((int)(base_spd * 0.6), (int)(base_spd * 0.6));
      delay(delay_before_turn);

      // Execute differential in-place spin pivot until center sensors recapture the line
      if (direction == "right") {
        turnRight(turnSpeed, turnSpeed);
      } else if (direction == "left") {
        turnLeft(turnSpeed, turnSpeed);
      }

      // Turn indicator LED off
      digitalWrite(led, LOW);
      direction = "straight"; // Clear directional state
      offTrackStartTime = 0;  // Reset off-track safety timer
      return;                 // Immediately return to resume normal closed-loop tracking
    } else {
      // Direction is straight, but no line detected: Check for off-track runaway
      if (offTrackStartTime > 0 && (millis() - offTrackStartTime > 350)) {
        // Line has been lost for over 350 ms: automatically brake motors for safety
        motor(0, 0);
        return;
      }
    }
  }

  // --------------------------------------------------------------------------------------------------
  // 5. CALCULATE DERIVATIVE (RATE OF ERROR CHANGE)
  // --------------------------------------------------------------------------------------------------
  derivative = error - previous_error;

  // --------------------------------------------------------------------------------------------------
  // 6. CALCULATE STEERING CORRECTION (PROPORTIONAL + DERIVATIVE TERMS)
  // --------------------------------------------------------------------------------------------------
  int correction = (int)((error * (float)p) + (derivative * (float)d));

  // --------------------------------------------------------------------------------------------------
  // 7. DIFFERENTIAL MOTOR STEERING COMMANDS
  // Error > 0 (Line to left): Left motor slows down, Right motor speeds up -> Steer Left
  // Error < 0 (Line to right): Left motor speeds up, Right motor slows down -> Steer Right
  // --------------------------------------------------------------------------------------------------
  int left_motor_correction  = base_spd - correction;
  int right_motor_correction = base_spd + correction;

  // Store current error for next derivative calculation
  previous_error = error;

  // --------------------------------------------------------------------------------------------------
  // 8. COMMAND MOTOR DRIVER VIA HARDWARE PWM (TIMER 0 PINS D5 & D6)
  // --------------------------------------------------------------------------------------------------
  motor(left_motor_correction, right_motor_correction);
}
