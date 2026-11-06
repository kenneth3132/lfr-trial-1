/**
 * ====================================================================================================
 * PROJECT: FAST PID LINE FOLLOWER ROBOT (PARTS 1 TO 8 INTEGRATED — DIRECT NO-MUX ARCHITECTURE)
 * TARGET HARDWARE: ARDUINO NANO (ATmega328P, 16MHz)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * This production sketch integrates all core engineering features from Part 01 through Part 08
 * into a single unified architecture WITHOUT using any analog multiplexer (MUX).
 * 
 * Instead of routing sensor channels through a 16-channel MUX (such as CD74HC4067) with channel select
 * pins and settling delays, this implementation utilizes all 8 dedicated analog inputs of the
 * Arduino Nano (A0 through A7) in direct parallel read mode.
 * 
 * INTEGRATED SUBSYSTEMS (PARTS 1-8):
 * - Part 01: Decoupled power distribution, star grounding, and H-bridge electrical interfacing.
 * - Part 02: 8-IR sensor reading, analog-to-digital thresholding, weighted position calculation, bitmask.
 * - Part 03: Bidirectional differential motor driver (PWM speed control + directional GPIO logic).
 * - Part 04: Proportional-Derivative (PD/PID) closed-loop steering with center-of-mass weighted error.
 * - Part 05: 90-degree sharp turns, acute angles, and intersection handling with pattern-matching memory.
 * - Part 06: Timed dead-reckoning distance pulses and braking trajectory calibration.
 * - Part 07: Dynamic 4-sweep self-calibration, dynamic reference midpoint math, and EEPROM persistence.
 * - Part 08: Inverted track mode (normal black-on-white vs. inverted white-on-black surface).
 * 
 * PIN ALLOCATION TABLE (ARDUINO NANO):
 * ----------------------------------------------------------------------------------------------------
 * Function                 | Pin  | Type        | Hardware Description
 * ----------------------------------------------------------------------------------------------------
 * IR Sensor 0 (Far Left)   | A0   | Analog In   | Direct TCRT5000 phototransistor analog output
 * IR Sensor 1              | A1   | Analog In   | Direct TCRT5000 phototransistor analog output
 * IR Sensor 2              | A2   | Analog In   | Direct TCRT5000 phototransistor analog output
 * IR Sensor 3 (Mid Left)   | A3   | Analog In   | Direct TCRT5000 phototransistor analog output
 * IR Sensor 4 (Mid Right)  | A4   | Analog In   | Direct TCRT5000 phototransistor analog output
 * IR Sensor 5              | A5   | Analog In   | Direct TCRT5000 phototransistor analog output
 * IR Sensor 6              | A6   | Analog In   | Direct TCRT5000 phototransistor analog output (Analog-only)
 * IR Sensor 7 (Far Right)  | A7   | Analog In   | Direct TCRT5000 phototransistor analog output (Analog-only)
 * ----------------------------------------------------------------------------------------------------
 * Left Motor PWM (PWMB)    | D5   | Digital PWM | Timer 0 PWM (~976 Hz) speed modulation
 * Left Motor IN3 (BIN1)    | D2   | Digital Out | Direction logic line
 * Left Motor IN4 (BIN2)    | D4   | Digital Out | Direction logic line
 * Right Motor PWM (PWMA)   | D6   | Digital PWM | Timer 0 PWM (~976 Hz) speed modulation
 * Right Motor IN1 (AIN1)   | D7   | Digital Out | Direction logic line
 * Right Motor IN2 (AIN2)   | D8   | Digital Out | Direction logic line
 * ----------------------------------------------------------------------------------------------------
 * Button 1 (Start PID)     | D11  | Digital In  | Active-LOW with internal pull-up (INPUT_PULLUP)
 * Button 2 (Calibrate)     | D12  | Digital In  | Active-LOW with internal pull-up (INPUT_PULLUP)
 * Status / Turn LED        | D13  | Digital Out | Built-in onboard LED (turn indicator & status)
 * Serial Debug (RX / TX)   | D0/D1| Hardware    | UART Communication at 9600 Baud
 * ----------------------------------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <EEPROM.h>

// ====================================================================================================
// 1. PIN DEFINITIONS
// ====================================================================================================

// --- DIRECT ANALOG SENSOR INPUTS (NO MUX) ---
const uint8_t sensorPins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
#define sensorNumber 8

// --- MOTOR DRIVER PINS (TB6612FNG or L298N) ---
#define left_motor_speed     5   // Timer 0 PWM (PWMB / EnB)
#define left_motor_forward   2   // BIN1 / IN3
#define left_motor_backward  4   // BIN2 / IN4

#define right_motor_speed    6   // Timer 0 PWM (PWMA / EnA)
#define right_motor_forward  7   // AIN1 / IN1
#define right_motor_backward 8   // AIN2 / IN2

// --- USER INPUT BUTTONS & INDICATOR LED ---
#define button_start 11          // Button 1: Start PID Line Follow (Active-LOW)
#define button_calib 12          // Button 2: Run Auto-Calibration Sweep (Active-LOW)
#define led          13          // Built-in LED on Arduino Nano

// ====================================================================================================
// 2. GLOBAL SENSOR & POSITION VARIABLES (PARTS 02, 04, 07, 08)
// ====================================================================================================

int sensorADC[sensorNumber];      // Raw 10-bit analog values (0 to 1023)
int sensorDigital[sensorNumber];  // Binary states: 1 = line detected, 0 = background
int Max_ADC[sensorNumber];        // Maximum recorded ADC reading during calibration
int Min_ADC[sensorNumber];        // Minimum recorded ADC reading during calibration
int Reference_ADC[sensorNumber];  // Dynamic midpoint threshold = (Max_ADC + Min_ADC) / 2

// Positional weight array: Leftmost = 10, Rightmost = 80, Center = 45.0
const int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80 };
// Binary powers of 2 for fast bitmask synthesis: S0 = 1, S1 = 2, ..., S7 = 128
const int bitWeight[sensorNumber]   = { 1, 2, 4, 8, 16, 32, 64, 128 };

int fallback_threshold = 500;     // Default fallback threshold before EEPROM calibration is loaded
int sumOnSensor = 0;              // Count of active sensors currently seeing the line
long sensorWight = 0;             // Sum of (sensorDigital[i] * WeightValue[i])
int bitSensor = 0;                // 8-bit packed binary representation of the sensor array state

// Inversion mode flag: 0 = Normal track (Black line on White), 1 = Inverted track (White line on Black)
bool inverseON = 0;

// ====================================================================================================
// 3. PID CONTROLLER & KINEMATIC TUNING PARAMETERS (PARTS 04 & 06)
// ====================================================================================================

float line_position   = 45.0;     // Current estimated line centroid position
float center_position = 45.0;     // Target setpoint position (geometric center of 8 sensors)
float error           = 0.0;      // Current error = center_position - line_position
float previous_error  = 0.0;      // Previous error for derivative calculation
float derivative      = 0.0;      // Rate of change of error

// Tunable PID Constants (Adjust based on chassis weight, motor RPM, and surface traction)
int base_speed = 200;             // Nominal forward cruise speed (0 to 255)
int kp         = 8;               // Proportional gain: determines steering correction aggressiveness
int kd         = 80;              // Derivative gain: dampens high-frequency steering oscillation

// ====================================================================================================
// 4. TURN DETECTION & TRAJECTORY STATE MACHINE (PARTS 05 & 06)
// ====================================================================================================

String direction = "straight";    // Directional memory: "straight", "left", or "right"
#define delay_before_turn 80      // Milliseconds delay before pivoting (aligns wheel axle over corner)
#define turnSpeed         160     // Differential wheel speed during in-place pivot maneuvers (0 to 255)

// Dead-reckoning distance test parameters (Part 06)
uint16_t test_time = 500;         // Duration of open-loop test pulse in milliseconds
int measured_distance = 30;       // Benchmark distance in centimeters

// ====================================================================================================
// 5. FUNCTION DECLARATIONS
// ====================================================================================================

void read_black_line();
void motor(int LPWM, int RPWM);
void PID_Controller(int base_speed, int p, int d);
void turnRight(int L, int R);
void turnLeft(int L, int R);
void hard_stop();
void calibrateSensor();
void LoadCalibration();
void sensor_value_test();
void Bit_Sensor_Show();
void distance(int cm);
void stop();

// ====================================================================================================
// 6. SETUP ROUTINE
// ====================================================================================================

void setup() {
  // Initialize UART Serial for telemetry, debugging, and calibration inspection
  Serial.begin(9600);
  Serial.println(F("=================================================="));
  Serial.println(F("   FAST PID LINE FOLLOWER ROBOT (NO-MUX EDITION)  "));
  Serial.println(F("=================================================="));

  // Configure Analog Sensor Pins as High-Impedance Inputs
  for (int i = 0; i < sensorNumber; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Configure Motor Direction and PWM Pins as Outputs
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(right_motor_speed, OUTPUT);

  // Configure User Push Buttons with Internal Pull-Ups
  pinMode(button_start, INPUT_PULLUP);
  pinMode(button_calib, INPUT_PULLUP);

  // Configure Status LED Output
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Stop Motors initially for safety
  motor(0, 0);

  // Load Calibrated Sensor Thresholds from Non-Volatile EEPROM
  LoadCalibration();

  Serial.println(F("[SYSTEM READY] Press Button 1 to Run PID | Button 2 to Calibrate"));
}

// ====================================================================================================
// 7. MAIN CONTROL LOOP
// ====================================================================================================

void loop() {
  // BUTTON 2 PRESSED: Execute Autonomous 4-Sweep Dynamic Calibration (Part 07)
  if (digitalRead(button_calib) == LOW) {
    delay(200); // Debounce delay
    Serial.println(F("[ACTION] Starting Dynamic Sensor Calibration..."));
    digitalWrite(led, HIGH);
    calibrateSensor();
    digitalWrite(led, LOW);
    Serial.println(F("[ACTION] Calibration Complete! Stored in EEPROM."));
  }

  // BUTTON 1 PRESSED: Launch Continuous High-Speed PID Tracking (Part 04 + Part 05)
  if (digitalRead(button_start) == LOW) {
    delay(200); // Debounce delay
    Serial.println(F("[ACTION] Launching Closed-Loop PID Line Tracking..."));
    digitalWrite(led, HIGH);
    
    // Continuous closed-loop operation
    while (1) {
      PID_Controller(base_speed, kp, kd);
    }
  }
}
