/**
 * ====================================================================================================
 * WOKWI SIMULATION FIRMWARE: FAST PID LINE FOLLOWER ROBOT (PARTS 1-8 COMBINED, NO-MUX)
 * PLATFORM: ARDUINO NANO (ATmega328P)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Single-file consolidated simulation firmware ready for direct execution in the Wokwi simulator.
 * Integrates all algorithms from Part 01 through Part 08:
 *   - Direct 8-channel analog acquisition on pins A0-A7 (no MUX).
 *   - Dynamic auto-calibration with EEPROM persistence.
 *   - Proportional-Derivative (PD/PID) line tracking with weighted centroid calculation.
 *   - State-machine 90° turn and intersection handling.
 *   - Normal (black-on-white) and Inverted (white-on-black) track modes.
 *   - Live Serial HUD and telemetry reporting.
 * 
 * WOKWI INTERACTION GUIDE:
 *   - S0 to S7 Potentiometers (A0 to A7) : Adjust dial to simulate line position over sensors.
 *     * Dial Low (<500)  : White background (reflected light).
 *     * Dial High (>500) : Black line (absorbed light).
 *   - Button 1 (Pin D11) : Press to start continuous PID Line Follow.
 *   - Button 2 (Pin D12) : Press to trigger 4-Sweep Auto-Calibration.
 *   - Status LED (Pin D13): Turns ON during calibration sweeps and sharp 90° pivots.
 * ====================================================================================================
 */

#include <Arduino.h>
#include <EEPROM.h>

// ====================================================================================================
// 1. PIN DEFINITIONS
// ====================================================================================================

// --- DIRECT ANALOG SENSOR INPUTS (PINS A0 to A7) ---
const uint8_t sensorPins[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
#define sensorNumber 8

// --- MOTOR DRIVER PINS (TB6612FNG / L298N) ---
#define left_motor_speed     5   // Timer 0 PWM (PWMB / EnB)
#define left_motor_forward   2   // BIN1 / IN3
#define left_motor_backward  4   // BIN2 / IN4

#define right_motor_speed    6   // Timer 0 PWM (PWMA / EnA)
#define right_motor_forward  7   // AIN1 / IN1
#define right_motor_backward 8   // AIN2 / IN2

// --- USER INPUT BUTTONS & STATUS LED ---
#define button_start 11          // Button 1: Start PID Line Follow (Active-LOW)
#define button_calib 12          // Button 2: Run Calibration Sweep (Active-LOW)
#define led          13          // Built-in LED / Turn Indicator

// ====================================================================================================
// 2. GLOBAL VARIABLES & ARRAYS
// ====================================================================================================

int sensorADC[sensorNumber];      // 10-bit raw ADC readings (0 to 1023)
int sensorDigital[sensorNumber];  // 1 = line detected, 0 = background
int Max_ADC[sensorNumber];        // Calibrated maximum values
int Min_ADC[sensorNumber];        // Calibrated minimum values
int Reference_ADC[sensorNumber];  // Midpoint reference thresholds

// Spatial weights: S0 = 10, S1 = 20, ..., S7 = 80 (Center = 45.0)
const int WeightValue[sensorNumber] = { 10, 20, 30, 40, 50, 60, 70, 80 };
// Binary powers of 2 for fast bitmask encoding
const int bitWeight[sensorNumber]   = { 1, 2, 4, 8, 16, 32, 64, 128 };

int fallback_threshold = 500;     // Default threshold before calibration
int sumOnSensor = 0;              // Active sensor count
long sensorWight = 0;             // Weighted moment accumulator
int bitSensor = 0;                // 8-bit packed binary state

// Inversion mode: 0 = Normal (Black on White), 1 = Inverted (White on Black)
bool inverseON = 0;

// PID Tuning Parameters
float line_position   = 45.0;     // Current estimated line centroid
float center_position = 45.0;     // Center setpoint
float error           = 0.0;      // Error = center - position
float previous_error  = 0.0;      // Error from prior iteration
float derivative      = 0.0;      // Error rate of change

int base_speed = 200;             // Base cruising speed (0 to 255)
int kp         = 8;               // Proportional gain
int kd         = 80;              // Derivative damping gain

// 90° Turn State Machine Variables
String direction = "straight";    // Directional memory: "straight", "left", "right"
#define delay_before_turn 80      // Alignment delay before pivot (ms)
#define turnSpeed         160     // Pivot rotation speed (0 to 255)

// Telemetry Timing
unsigned long lastTelemetryTime = 0;

// ====================================================================================================
// 3. FUNCTION PROTOTYPES
// ====================================================================================================

void read_black_line();
void motor(int LPWM, int RPWM);
void PID_Controller(int base_spd, int p, int d);
void turnRight(int L, int R);
void turnLeft(int L, int R);
void hard_stop();
void calibrateSensor();
void LoadCalibration();
void printTelemetry(int left_pwm, int right_pwm);

// ====================================================================================================
// 4. SETUP ROUTINE
// ====================================================================================================

void setup() {
  Serial.begin(115200); // High-speed 115200 baud for smooth Wokwi serial streaming
  while (!Serial && millis() < 2000);

  Serial.println(F("\n==========================================================="));
  Serial.println(F("   FAST PID LINE FOLLOWER ROBOT — WOKWI SIMULATOR EDITION   "));
  Serial.println(F("==========================================================="));

  // Configure Analog Inputs
  for (int i = 0; i < sensorNumber; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // Configure Motor Outputs
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);
  pinMode(right_motor_speed, OUTPUT);

  // Configure User Inputs (Buttons have internal pull-ups)
  pinMode(button_start, INPUT_PULLUP);
  pinMode(button_calib, INPUT_PULLUP);

  // Configure Status LED
  pinMode(led, OUTPUT);
  digitalWrite(led, LOW);

  // Stop Motors initially
  motor(0, 0);

  // Load Stored Calibration from EEPROM (or use fallback)
  LoadCalibration();

  Serial.println(F("\n[READY] Instructions:"));
  Serial.println(F("  - Adjust Potentiometers S0-S7 to simulate line position."));
  Serial.println(F("  - Press BUTTON 1 (Pin D11) to launch closed-loop PID."));
  Serial.println(F("  - Press BUTTON 2 (Pin D12) to execute Auto-Calibration.\n"));
}

// ====================================================================================================
// 5. MAIN LOOP
// ====================================================================================================

void loop() {
  // Check for Calibration Trigger (Button 2)
  if (digitalRead(button_calib) == LOW) {
    delay(200); // Debounce
    Serial.println(F("\n>>> [ACTION] Button 2 Pressed: Running Dynamic Auto-Calibration..."));
    digitalWrite(led, HIGH);
    calibrateSensor();
    digitalWrite(led, LOW);
    Serial.println(F(">>> [ACTION] Calibration Complete! Stored in EEPROM.\n"));
  }

  // Check for PID Run Trigger (Button 1)
  if (digitalRead(button_start) == LOW) {
    delay(200); // Debounce
    Serial.println(F("\n>>> [ACTION] Button 1 Pressed: Launching Closed-Loop PID Line Follower!"));
    digitalWrite(led, HIGH);
    delay(300);
    digitalWrite(led, LOW);

    // Enter continuous PID control loop
    while (1) {
      PID_Controller(base_speed, kp, kd);

      // Allow breaking out back to idle if Button 2 is held
      if (digitalRead(button_calib) == LOW) {
        motor(0, 0);
        Serial.println(F("\n>>> [HALT] PID Line Follow Stopped by User."));
        delay(500);
        break;
      }
    }
  }

  // While idle, print live sensor states every 500ms
  if (millis() - lastTelemetryTime > 500) {
    lastTelemetryTime = millis();
    read_black_line();
    Serial.print(F("[IDLE SENSOR MONITOR] Bitmask: "));
    for (int i = 7; i >= 0; i--) {
      Serial.print(bitRead(bitSensor, i));
    }
    Serial.print(F(" | Active: "));
    Serial.print(sumOnSensor);
    Serial.print(F(" | Pos: "));
    if (sumOnSensor > 0) {
      Serial.print((float)sensorWight / sumOnSensor, 1);
    } else {
      Serial.print(F("OFF_LINE"));
    }
    Serial.println(F(" (Press Button 1 on D11 to Run)"));
  }
}

// ====================================================================================================
// 6. SENSOR ACQUISITION MODULE (DIRECT NO-MUX)
// ====================================================================================================

void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor   = 0;

  for (int i = 0; i < sensorNumber; i++) {
    // Direct Analog Read from Nano Pin
    sensorADC[i] = analogRead(sensorPins[i]);

    // Use calibrated Reference threshold if available, otherwise fallback
    int thresh = (Reference_ADC[i] > 50) ? Reference_ADC[i] : fallback_threshold;

    // Binarize reading
    if (sensorADC[i] > thresh) {
      sensorDigital[i] = (inverseON == 0) ? 1 : 0;
    } else {
      sensorDigital[i] = (inverseON == 0) ? 0 : 1;
    }

    // Accumulate centroid moments
    sumOnSensor += sensorDigital[i];
    sensorWight += (long)sensorDigital[i] * WeightValue[i];

    // Pack into 8-bit representation: S0 (Left) = MSB, S7 (Right) = LSB
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

// ====================================================================================================
// 7. MOTOR DRIVER MODULE (TB6612FNG / L298N)
// ====================================================================================================

void motor(int LPWM, int RPWM) {
  // Clamp speed to 8-bit PWM bounds [-255, +255]
  LPWM = constrain(LPWM, -255, 255);
  RPWM = constrain(RPWM, -255, 255);

  // Left Motor Direction
  if (LPWM > 0) {
    digitalWrite(left_motor_forward, HIGH);
    digitalWrite(left_motor_backward, LOW);
  } else if (LPWM < 0) {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, HIGH);
  } else {
    digitalWrite(left_motor_forward, LOW);
    digitalWrite(left_motor_backward, LOW);
  }

  // Right Motor Direction
  if (RPWM > 0) {
    digitalWrite(right_motor_forward, HIGH);
    digitalWrite(right_motor_backward, LOW);
  } else if (RPWM < 0) {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, HIGH);
  } else {
    digitalWrite(right_motor_forward, LOW);
    digitalWrite(right_motor_backward, LOW);
  }

  // Output PWM duty cycle via Timer 0
  analogWrite(left_motor_speed, abs(LPWM));
  analogWrite(right_motor_speed, abs(RPWM));
}

// ====================================================================================================
// 8. CLOSED-LOOP PID CONTROLLER & TURN ARBITRATOR
// ====================================================================================================

void PID_Controller(int base_spd, int p, int d) {
  read_black_line();

  // Compute Centroid
  if (sumOnSensor > 0) {
    line_position = (float)sensorWight / (float)sumOnSensor;
  }
  error = center_position - line_position;

  // Turn Pattern Recognition (Part 05)
  switch (bitSensor) {
    // Left-biased patterns
    case 0b11111000:
    case 0b11110000:
    case 0b11111100:
    case 0b11100000:
      direction = "left";
      break;

    // Right-biased patterns
    case 0b00011111:
    case 0b00001111:
    case 0b00111111:
    case 0b00000111:
      direction = "right";
      break;

    // Centered tracking
    case 0b00011000:
    case 0b00010000:
    case 0b00001000:
    case 0b00111100:
      direction = "straight";
      break;
  }

  // 90° Turn Execution on Line Loss (White space following a biased pattern)
  if (bitSensor == 0) {
    if (direction != "straight") {
      digitalWrite(led, HIGH);
      delay(delay_before_turn);

      if (direction == "right") {
        Serial.println(F("[TURN EVENT] Executing 90° Pivot RIGHT..."));
        turnRight(turnSpeed, turnSpeed);
      } else {
        Serial.println(F("[TURN EVENT] Executing 90° Pivot LEFT..."));
        turnLeft(turnSpeed, turnSpeed);
      }

      digitalWrite(led, LOW);
      return;
    }
  }

  // Derivative calculation
  derivative = error - previous_error;

  // Correction calculation
  int correction = (int)((error * (float)p) + (derivative * (float)d));
  int left_motor_correction  = base_spd - correction;
  int right_motor_correction = base_spd + correction;

  previous_error = error;

  // Command motors
  motor(left_motor_correction, right_motor_correction);

  // Periodic Telemetry Reporting (Every 100ms)
  if (millis() - lastTelemetryTime > 100) {
    lastTelemetryTime = millis();
    printTelemetry(left_motor_correction, right_motor_correction);
  }
}

// ====================================================================================================
// 9. SHARP TURN RECOVERY ROUTINES
// ====================================================================================================

void turnRight(int L, int R) {
  // Pivot until center sensors recapture the line
  while (sensorDigital[4] != 1 && sensorDigital[3] != 1) {
    motor(L, -R);
    read_black_line();
    direction = "straight";
  }
  motor(0, 0);
}

void turnLeft(int L, int R) {
  // Pivot until center sensors recapture the line
  while (sensorDigital[3] != 1 && sensorDigital[4] != 1) {
    motor(-L, R);
    read_black_line();
    direction = "straight";
  }
  motor(0, 0);
}

void hard_stop() {
  while (1) {
    motor(0, 0);
    digitalWrite(led, !digitalRead(led));
    delay(200);
  }
}

// ====================================================================================================
// 10. DYNAMIC CALIBRATION & EEPROM STORAGE
// ====================================================================================================

void calibrateSensor() {
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;
    Min_ADC[i] = 1023;
  }

  int rotationDirections[4][2] = {
    { -1,  1 }, {  1, -1 }, {  1, -1 }, { -1,  1 }
  };

  for (int step = 0; step < 4; step++) {
    motor((int)(base_speed * 0.4 * rotationDirections[step][0]),
          (int)(base_speed * 0.4 * rotationDirections[step][1]));

    for (int sweep = 0; sweep < 100; sweep++) {
      for (int i = 0; i < sensorNumber; i++) {
        sensorADC[i] = analogRead(sensorPins[i]);
        if (sensorADC[i] > Max_ADC[i]) Max_ADC[i] = sensorADC[i];
        if (sensorADC[i] < Min_ADC[i]) Min_ADC[i] = sensorADC[i];
      }
      delayMicroseconds(200);
    }
    motor(0, 0);
    delay(100);
  }

  // Save to EEPROM
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2;
    EEPROM.write(i, (uint8_t)(Reference_ADC[i] / 4));
    delay(5);
    EEPROM.write(i + sensorNumber, (uint8_t)(Max_ADC[i] / 4));
    delay(5);
    EEPROM.write(i + (sensorNumber * 2), (uint8_t)(Min_ADC[i] / 4));
    delay(5);

    Serial.print(F("Sensor "));
    Serial.print(i);
    Serial.print(F(" -> Min: "));
    Serial.print(Min_ADC[i]);
    Serial.print(F(" | Ref: "));
    Serial.print(Reference_ADC[i]);
    Serial.print(F(" | Max: "));
    Serial.println(Max_ADC[i]);
  }
}

void LoadCalibration() {
  bool valid = true;
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 4;
    Max_ADC[i]       = EEPROM.read(i + sensorNumber) * 4;
    Min_ADC[i]       = EEPROM.read(i + (sensorNumber * 2)) * 4;

    if (Reference_ADC[i] < 50 || Reference_ADC[i] > 950) valid = false;
  }

  if (valid) {
    Serial.print(F("[EEPROM] Calibrated References Loaded: "));
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(Reference_ADC[i]);
      Serial.print(F(" "));
    }
    Serial.println();
  } else {
    Serial.println(F("[EEPROM] No valid calibration profile found. Using fallback threshold (500)."));
    for (int i = 0; i < sensorNumber; i++) {
      Reference_ADC[i] = fallback_threshold;
    }
  }
}

// ====================================================================================================
// 11. TELEMETRY STREAMING
// ====================================================================================================

void printTelemetry(int left_pwm, int right_pwm) {
  Serial.print(F("BIT: "));
  for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(bitSensor, i));
  }
  Serial.print(F(" | Pos: "));
  Serial.print(line_position, 1);
  Serial.print(F(" | Err: "));
  Serial.print(error, 1);
  Serial.print(F(" | LPWM: "));
  Serial.print(left_pwm);
  Serial.print(F(" | RPWM: "));
  Serial.print(right_pwm);
  Serial.print(F(" | Dir: "));
  Serial.println(direction);
}
