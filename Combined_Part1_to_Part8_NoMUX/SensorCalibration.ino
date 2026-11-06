/**
 * ====================================================================================================
 * MODULE: AUTONOMOUS DYNAMIC SENSOR CALIBRATION & EEPROM PERSISTENCE (PART 07)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Eliminates manual potentiometer adjustments and hardcoded reflectance thresholds.
 * When triggered, the robot executes 4 oscillating sweeps across the line:
 *   1. Measures dynamic maximum (dark line) and minimum (light background) for all 8 sensors.
 *   2. Calculates the optimal midpoint threshold: Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.
 *   3. Stores the calibrated parameters in non-volatile EEPROM (divided by 4 to fit in 8-bit cells).
 *   4. Automatically reloads calibration data upon reboot via LoadCalibration().
 * 
 * EEPROM ADDRESS MEMORY MAP:
 * -------------------------------------------------------------------------
 * Address Range | Parameter Stored       | Data Encoding
 * -------------------------------------------------------------------------
 * 0 to 7        | Reference_ADC[0..7]    | 8-bit (ADC / 4)
 * 8 to 15       | Max_ADC[0..7]          | 8-bit (ADC / 4)
 * 16 to 23      | Min_ADC[0..7]          | 8-bit (ADC / 4)
 * -------------------------------------------------------------------------
 */

#include <Arduino.h>
#include <EEPROM.h>

void calibrateSensor() {
  Serial.println(F("[CALIB] Initializing dynamic sweep calibration..."));

  // 1. INITIALIZE MIN/MAX SEARCH ENVELOPES
  for (int i = 0; i < sensorNumber; i++) {
    Max_ADC[i] = 0;     // Lowest possible 10-bit ADC reading
    Min_ADC[i] = 1023;  // Highest possible 10-bit ADC reading
  }

  // 2. DEFINE 4-STAGE ROTATIONAL SWEEP SEQUENCE
  // Sweeps left and right to ensure all 8 sensors traverse both line and background
  int rotationDirections[4][2] = {
    { -1,  1 },  // Sweep Left
    {  1, -1 },  // Sweep Right
    {  1, -1 },  // Continue Right
    { -1,  1 }   // Return to Center
  };

  // 3. EXECUTE CALIBRATION SWEEPS
  for (int step = 0; step < 4; step++) {
    // Drive motors at low speed (40% base speed) for controlled rotational sweeps
    motor((int)(base_speed * 0.4 * rotationDirections[step][0]),
          (int)(base_speed * 0.4 * rotationDirections[step][1]));

    // Sample 400 times during rotation
    for (int sweep = 0; sweep < 400; sweep++) {
      for (int i = 0; i < sensorNumber; i++) {
        // Direct parallel analog read (No MUX switching)
        sensorADC[i] = analogRead(sensorPins[i]);

        // Track minimum and maximum values seen by this sensor
        if (sensorADC[i] > Max_ADC[i]) Max_ADC[i] = sensorADC[i];
        if (sensorADC[i] < Min_ADC[i]) Min_ADC[i] = sensorADC[i];
      }
      delayMicroseconds(250);
    }

    // Stop momentarily between directional shifts
    motor(0, 0);
    delay(150);
  }

  // 4. COMPUTE MIDPOINT THRESHOLDS & WRITE TO EEPROM
  Serial.println(F("[CALIB] Writing calibrated thresholds to EEPROM:"));
  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2;

    // Scale from 10-bit (0-1023) to 8-bit (0-255) for EEPROM storage
    EEPROM.write(i, (uint8_t)(Reference_ADC[i] / 4));
    delay(5);
    EEPROM.write(i + sensorNumber, (uint8_t)(Max_ADC[i] / 4));
    delay(5);
    EEPROM.write(i + (sensorNumber * 2), (uint8_t)(Min_ADC[i] / 4));
    delay(5);

    Serial.print(F("S"));
    Serial.print(i);
    Serial.print(F(": Min="));
    Serial.print(Min_ADC[i]);
    Serial.print(F(" Mid="));
    Serial.print(Reference_ADC[i]);
    Serial.print(F(" Max="));
    Serial.println(Max_ADC[i]);
  }
}

/**
 * Loads calibrated sensor thresholds from EEPROM upon system startup
 */
void LoadCalibration() {
  Serial.println(F("[EEPROM] Loading stored calibration parameters..."));
  bool valid_data = true;

  for (int i = 0; i < sensorNumber; i++) {
    Reference_ADC[i] = EEPROM.read(i) * 4;
    Max_ADC[i]       = EEPROM.read(i + sensorNumber) * 4;
    Min_ADC[i]       = EEPROM.read(i + (sensorNumber * 2)) * 4;

    // Sanity check: If EEPROM is fresh/unwritten (255 * 4 = 1020 or 0), flag invalid
    if (Reference_ADC[i] < 50 || Reference_ADC[i] > 950) {
      valid_data = false;
    }
  }

  if (valid_data) {
    Serial.print(F("[EEPROM LOADED] Reference Midpoints: "));
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(Reference_ADC[i]);
      Serial.print(F(" "));
    }
    Serial.println();
  } else {
    Serial.println(F("[EEPROM NOTICE] No valid calibration profile found in EEPROM."));
    Serial.println(F("[EEPROM NOTICE] Using default fallback threshold (500). Please run calibration!"));
    for (int i = 0; i < sensorNumber; i++) {
      Reference_ADC[i] = fallback_threshold;
    }
  }
}
