/**
 * ====================================================================================================
 * PROJECT: FAST PID LINE FOLLOWER ROBOT (PARTS 1 TO 8 INTEGRATED — DIRECT NO-MUX ARCHITECTURE)
 * REVISION: ENHANCED & VERIFIED PRODUCTION RELEASE (WITH BUTTON DIAGNOSTICS & SAFETY CONTROLS)
 * TARGET HARDWARE: ARDUINO NANO (ATmega328P, 16MHz)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * This sketch unifies all functionality from Part 01 through Part 08 without an analog multiplexer (MUX).
 * All 8 IR TCRT5000 phototransistors connect directly to analog inputs A0-A7 of the Arduino Nano.
 * 
 * KEY ARCHITECTURAL & SAFETY IMPROVEMENTS IN THIS BUILD:
 * 1. REAL-TIME IDLE TELEMETRY:
 *    Streams button states (PRESSED vs HIGH), active sensor count, and line position every 500ms
 *    while waiting in loop(). You can immediately verify whether your buttons are working.
 * 2. 3-FLASH SAFETY COUNTDOWN:
 *    When Button 1 is pressed, the status LED flashes 3 times over 1 second before motors start,
 *    giving you time to release the robot safely.
 * 3. EMERGENCY USER HALT:
 *    Pressing Button 2 during PID line tracking immediately cuts motor power (motor(0,0)), turns off
 *    the LED, and returns the system cleanly back to idle mode.
 * 4. PIVOT WATCHDOG TIMEOUT:
 *    90° turn routines (turnRight/turnLeft) now have a 1500ms watchdog timeout to prevent infinite
 *    spinning if the robot is lifted or runs off the track.
 * 5. STICKY DIRECTION STATE FIX:
 *    Direction memory is now properly reset to "straight" on all centered/mild curve patterns,
 *    preventing accidental 90° pivots on minor track gaps.
 * 6. OFF-TRACK AUTO-BRAKE:
 *    If the line is lost completely for more than 350ms while driving straight, motors automatically
 *    stop for safety.
 * 7. NON-DESTRUCTIVE EEPROM VALIDATION:
 *    Sensor calibration thresholds are stored using EEPROM.update() to protect cell lifespan, and
 *    validated per-sensor rather than discarding the entire array.
 * 
 * ====================================================================================================
 * MASTER PIN ALLOCATION TABLE (ARDUINO NANO):
 * ====================================================================================================
 * Function                 | Pin  | Electrical Type | Hardware Description & Wiring
 * ----------------------------------------------------------------------------------------------------
 * IR Sensor 0 (Far Left)   | A0   | Analog In       | TCRT5000 Phototransistor Collector
 * IR Sensor 1              | A1   | Analog In       | TCRT5000 Phototransistor Collector
 * IR Sensor 2              | A2   | Analog In       | TCRT5000 Phototransistor Collector
 * IR Sensor 3 (Mid Left)   | A3   | Analog In       | TCRT5000 Phototransistor Collector
 * IR Sensor 4 (Mid Right)  | A4   | Analog In       | TCRT5000 Phototransistor Collector
 * IR Sensor 5              | A5   | Analog In       | TCRT5000 Phototransistor Collector
 * IR Sensor 6              | A6   | Analog In       | TCRT5000 Phototransistor Collector (Analog-only)
 * IR Sensor 7 (Far Right)  | A7   | Analog In       | TCRT5000 Phototransistor Collector (Analog-only)
 * ----------------------------------------------------------------------------------------------------
 * Left Motor PWM (PWMB)    | D5   | Digital PWM     | Timer 0 Hardware PWM (~976 Hz)
 * Left Motor IN3 (BIN1)    | D2   | Digital Out     | Direction logic line (HIGH = Forward)
 * Left Motor IN4 (BIN2)    | D4   | Digital Out     | Direction logic line (HIGH = Reverse)
 * Right Motor PWM (PWMA)   | D6   | Digital PWM     | Timer 0 Hardware PWM (~976 Hz)
 * Right Motor IN1 (AIN1)   | D7   | Digital Out     | Direction logic line (HIGH = Forward)
 * Right Motor IN2 (AIN2)   | D8   | Digital Out     | Direction logic line (HIGH = Reverse)
 * ----------------------------------------------------------------------------------------------------
 * Button 1 (Start PID)     | D11  | Digital In      | Active-LOW with internal pull-up (Wire: Pin -> GND)
 * Button 2 (Calibrate/Halt)| D12  | Digital In      | Active-LOW with internal pull-up (Wire: Pin -> GND)
 * Status / Turn LED        | D13  | Digital Out     | Built-in onboard LED (Blinks on turn/countdown)
 * Serial Debug (RX / TX)   | D0/D1| UART Hardware   | Serial Communication at 9600 Baud
 * ----------------------------------------------------------------------------------------------------
 * SPARE PINS AVAILABLE     | D3, D9, D10            | Free digital pins for custom features or remapping
 * ====================================================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// ====================================================================================================
// 1. PIN DEFINITIONS
// ====================================================================================================

// --- DIRECT ANALOG SENSOR INPUTS (8 DEDICATED ANALOG CHANNELS, NO MUX) ---
const uint8_t sensorPins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
#define sensorNumber 8

// --- MOTOR DRIVER PINS (TB6612FNG or L298N) ---
#define left_motor_speed     5   // Timer 0 PWM speed modulation (PWMB / EnB)
#define left_motor_forward   2   // Motor B direction line 1 (BIN1 / IN3)
#define left_motor_backward  4   // Motor B direction line 2 (BIN2 / IN4)

#define right_motor_speed    6   // Timer 0 PWM speed modulation (PWMA / EnA)
#define right_motor_forward  7   // Motor A direction line 1 (AIN1 / IN1)
#define right_motor_backward 8   // Motor A direction line 2 (AIN2 / IN2)

// --- USER INPUT BUTTONS & STATUS INDICATOR LED ---
// NOTE: Internal pull-ups are enabled. Connect switches between pin and GND (Active-LOW).
#define button_start 11          // Button 1: Start PID Line Follow (Active-LOW, Pin D11 -> GND)
#define button_calib 12          // Button 2: Auto-Calibration & Emergency Stop (Active-LOW, Pin D12 -> GND)
#define led          13          // Built-in LED on Arduino Nano

// ====================================================================================================
// 2. GLOBAL SENSOR & POSITION VARIABLES (PARTS 02, 04, 07, 08)
// ====================================================================================================

int sensorADC[sensorNumber];      // Raw 10-bit analog reflectance readings (0 to 1023)
int sensorDigital[sensorNumber];  // Binarized digital states: 1 = line detected, 0 = background
int Max_ADC[sensorNumber];        // Maximum recorded ADC reading during calibration (dark surface)
int Min_ADC[sensorNumber];        // Minimum recorded ADC reading during calibration (light surface)
int Reference_ADC[sensorNumber];  // Optimal dynamic midpoint threshold = (Max_ADC + Min_ADC) / 2

// Positional weight array: Leftmost (S0) = 10, Rightmost (S7) = 80, Center = 45.0
const int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80 };

// Binary powers of 2 for fast bitmask synthesis: S0 = 128 (MSB), ..., S7 = 1 (LSB)
const int bitWeight[sensorNumber]   = { 1, 2, 4, 8, 16, 32, 64, 128 };

int fallback_threshold = 500;     // Default fallback threshold if EEPROM has no valid calibration
int sumOnSensor = 0;              // Total number of sensors currently detecting the line
long sensorWight = 0;             // Sum of active sensor weights (sum of sensorDigital[i] * WeightValue[i])
int bitSensor = 0;                // 8-bit packed binary representation of the entire sensor array state

// Track Polarity Mode: 0 = Normal track (Black line on White), 1 = Inverted track (White line on Black)
bool inverseON = 0;

// ====================================================================================================
// 3. PID CONTROLLER & KINEMATIC TUNING PARAMETERS (PARTS 04 & 06)
// ====================================================================================================

float line_position   = 45.0;     // Current estimated line centroid position
float center_position = 45.0;     // Target setpoint position (geometric center of the 8 sensors)
float error           = 0.0;      // Current deviation error = center_position - line_position
float previous_error  = 0.0;      // Stored error from previous cycle for derivative calculation
float derivative      = 0.0;      // Rate of change of error

// Tunable PID Constants (Adjust based on chassis weight, motor RPM, and track traction)
int base_speed = 200;             // Nominal forward cruise PWM speed (0 to 255)
int kp         = 8;               // Proportional gain: governs aggressiveness of corrective steering
int kd         = 80;              // Derivative gain: suppresses high-frequency lateral oscillation

// ====================================================================================================
// 4. TURN DETECTION & TRAJECTORY STATE MACHINE (PARTS 05 & 06)
// ====================================================================================================

String direction = "straight";    // Directional memory: "straight", "left", or "right"
#define delay_before_turn 80      // Milliseconds forward drive before pivot (aligns wheel axle over corner)
#define turnSpeed         160     // Differential wheel speed during in-place pivot maneuvers (0 to 255)

// Off-track safety timer: stops motors if robot completely loses the line on a straight section
unsigned long offTrackStartTime = 0;

// Dead-reckoning distance test parameters (Part 06)
uint16_t test_time = 500;         // Duration of open-loop test pulse in milliseconds
int measured_distance = 30;       // Benchmark distance in centimeters

// Telemetry timer for periodic idle serial updates
unsigned long lastTelemetryTime = 0;

// ====================================================================================================
// 5. FUNCTION DECLARATIONS (FORWARD PROTOTYPES)
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
void motor_test();
void distance(int cm);
void stop();

// ====================================================================================================
// 6. SETUP ROUTINE (RUNS ONCE AT POWER-UP OR RESET)
// ====================================================================================================

void setup() {
  // Step 1: Initialize UART Serial for telemetry, diagnostics, and calibration inspection
  Serial.begin(9600);
  delay(100);
  Serial.println(F("\n========================================================"));
  Serial.println(F("   FAST PID LINE FOLLOWER ROBOT (DIRECT NO-MUX EDITION) "));
  Serial.println(F("           REVISED & ENHANCED PRODUCTION FIRMWARE        "));
  Serial.println(F("========================================================"));

  // Step 2: Configure Motor Direction and PWM Pins as Digital Outputs
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(right_motor_speed, OUTPUT);

  // Step 3: Configure User Push Buttons with Internal Pull-Up Resistors (Active-LOW)
  pinMode(button_start, INPUT_PULLUP);
  pinMode(button_calib, INPUT_PULLUP);

  // Step 4: Configure Status LED Output
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Step 5: Safe Initial State: Command Motors to 0 (Full Stop / Electronic Brake)
  motor(0, 0);

  // Step 6: Load Calibrated Sensor Midpoint Thresholds from Non-Volatile EEPROM
  LoadCalibration();

  // Step 7: Print System Ready Instructions
  Serial.println(F("\n[SYSTEM READY - CONTROLS GUIDE]:"));
  Serial.println(F("  - BUTTON 1 (Pin D11 -> GND): Press to Launch PID Line Tracking"));
  Serial.println(F("  - BUTTON 2 (Pin D12 -> GND): Press to Run Auto-Calibration Sweep"));
  Serial.println(F("  - EMERGENCY STOP: During PID tracking, press BUTTON 2 to Halt immediately\n"));
}

// ====================================================================================================
// 7. MAIN CONTROL LOOP (POLLS USER BUTTONS & MONITORS IDLE TELEMETRY)
// ====================================================================================================

void loop() {
  // --------------------------------------------------------------------------------------------------
  // EVENT 1: BUTTON 2 PRESSED -> EXECUTE AUTONOMOUS 4-SWEEP DYNAMIC CALIBRATION (PART 07)
  // --------------------------------------------------------------------------------------------------
  if (digitalRead(button_calib) == LOW) {
    delay(250); // Debounce delay
    
    // Wait for the operator to release the button before starting
    while (digitalRead(button_calib) == LOW) {
      delay(10);
    }
    delay(50);

    Serial.println(F("\n>>> [ACTION] Button 2 Pressed: Starting Dynamic 4-Sweep Auto-Calibration..."));
    digitalWrite(led, HIGH);
    
    // Execute calibration sweep sequence
    calibrateSensor();
    
    digitalWrite(led, LOW);
    Serial.println(F(">>> [ACTION] Auto-Calibration Complete! Midpoints Stored in EEPROM.\n"));
  }

  // --------------------------------------------------------------------------------------------------
  // EVENT 2: BUTTON 1 PRESSED -> LAUNCH CONTINUOUS HIGH-SPEED CLOSED-LOOP PID TRACKING (PARTS 04 & 05)
  // --------------------------------------------------------------------------------------------------
  if (digitalRead(button_start) == LOW) {
    delay(250); // Debounce delay
    
    // Wait for the operator to release the button before initiating countdown
    while (digitalRead(button_start) == LOW) {
      delay(10);
    }
    delay(50);

    Serial.println(F("\n>>> [ACTION] Button 1 Pressed: Launching Closed-Loop PID Line Tracking!"));
    Serial.println(F(">>> [SAFETY] Launching in 1 second... (Press Button 2 at any time to Emergency Halt)"));

    // Safety Countdown: 3 rapid LED flashes (1000ms total) to allow hand clearance
    for (int i = 0; i < 3; i++) {
      digitalWrite(led, HIGH);
      delay(160);
      digitalWrite(led, LOW);
      delay(160);
    }
    digitalWrite(led, HIGH); // LED remains ON while active tracking is underway

    // Reset tracking state variables
    previous_error     = 0.0;
    direction          = "straight";
    offTrackStartTime  = 0;

    // CONTINUOUS CLOSED-LOOP OPERATION LOOP (SUPERVISED WITH EMERGENCY HALT)
    while (1) {
      // Execute one iteration of the PID closed-loop control algorithm
      PID_Controller(base_speed, kp, kd);

      // EMERGENCY USER HALT CHECK: Pressing Button 2 halts the robot immediately
      if (digitalRead(button_calib) == LOW) {
        motor(0, 0);            // Immediate active electronic braking
        digitalWrite(led, LOW); // Turn off status LED
        Serial.println(F("\n>>> [EMERGENCY HALT] PID Line Tracking Stopped by User (Button 2)."));
        
        // Wait for button release
        while (digitalRead(button_calib) == LOW) {
          delay(10);
        }
        delay(300);
        break; // Return cleanly to the main idle loop()
      }
    }
  }

  // --------------------------------------------------------------------------------------------------
  // PERIODIC IDLE TELEMETRY MONITOR (STREAMS EVERY 500 MS WHILE AWAITING COMMAND)
  // Allows instant diagnosis of Button 1, Button 2, and all 8 TCRT5000 sensor channels
  // --------------------------------------------------------------------------------------------------
  if (millis() - lastTelemetryTime > 500) {
    lastTelemetryTime = millis();
    read_black_line(); // Update sensor values for display

    Serial.print(F("[IDLE MONITOR] Btn1(D11): "));
    Serial.print(digitalRead(button_start) == LOW ? F("PRESSED(0) ") : F("HIGH(1)    "));
    Serial.print(F(" | Btn2(D12): "));
    Serial.print(digitalRead(button_calib) == LOW ? F("PRESSED(0) ") : F("HIGH(1)    "));
    Serial.print(F(" | Bitmask: "));
    for (int i = 7; i >= 0; i--) {
      Serial.print(bitRead(bitSensor, i));
    }
    Serial.print(F(" | Active: "));
    Serial.print(sumOnSensor);
    Serial.print(F(" | Centroid Pos: "));
    if (sumOnSensor > 0) {
      Serial.print(line_position, 1);
    } else {
      Serial.print(F("NONE"));
    }
    Serial.println(F(" | (Press D11 to Run)"));
  }
}
