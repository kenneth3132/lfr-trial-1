/**
 * ====================================================================================================
 * MODULE: AUTONOMOUS DYNAMIC SENSOR CALIBRATION & EEPROM PERSISTENCE (PART 07)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Eliminates manual potentiometer adjustments and static hardcoded thresholds.
 * When triggered via Button 2, the robot performs 4 in-place rotational sweeps across the track:
 *   1. Measures dynamic maximum (dark line) and minimum (light background) for all 8 sensors.
 *   2. Calculates optimal midpoint threshold: Reference_ADC[i] = (Max_ADC[i] + Min_ADC[i]) / 2.
 *   3. Stores calibrated values in non-volatile EEPROM using EEPROM.update() to protect cell lifespan.
 *   4. Automatically reloads calibration data upon reboot via LoadCalibration().
 * 
 * EEPROM MEMORY MAP:
 * ----------------------------------------------------------------------------------------------------
 * Address Range | Parameter Stored       | Data Encoding
 * ----------------------------------------------------------------------------------------------------
 * 0 to 7        | Reference_ADC[0..7]    | 8-bit unsigned char (ADC / 4, range 0-255)
 * 8 to 15       | Max_ADC[0..7]          | 8-bit unsigned char (ADC / 4, range 0-255)
 * 16 to 23      | Min_ADC[0..7]          | 8-bit unsigned char (ADC / 4, range 0-255)
 * ----------------------------------------------------------------------------------------------------
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
    { -1,  1 },  // Stage 1: Sweep Left
    {  1, -1 },  // Stage 2: Sweep Right
    {  1, -1 },  // Stage 3: Continue Right
    { -1,  1 }   // Stage 4: Return to Center
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
    // EEPROM.update() only writes if value has changed, preventing wear on EEPROM cells
    EEPROM.update(i, (uint8_t)(Reference_ADC[i] / 4));
    delay(5);
    EEPROM.update(i + sensorNumber, (uint8_t)(Max_ADC[i] / 4));
    delay(5);
    EEPROM.update(i + (sensorNumber * 2), (uint8_t)(Min_ADC[i] / 4));
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
 * Validates each sensor channel individually.
 */
void LoadCalibration() {
  Serial.println(F("[EEPROM] Loading stored calibration parameters..."));
  int validCount = 0;

  for (int i = 0; i < sensorNumber; i++) {
    uint8_t rawRef = EEPROM.read(i);
    uint8_t rawMax = EEPROM.read(i + sensorNumber);
    uint8_t rawMin = EEPROM.read(i + (sensorNumber * 2));

    // Reconstruct 10-bit values (multiply by 4)
    int loadedRef = (int)rawRef * 4;
    int loadedMax = (int)rawMax * 4;
    int loadedMin = (int)rawMin * 4;

    // Individual sensor sanity check:
    // EEPROM unwritten state is 255 (255 * 4 = 1020). Empty/zero is 0.
    // Ensure sensor has at least 40 ADC counts of contrast (Max > Min + 40)
    if (rawRef != 255 && rawRef != 0 && (loadedMax > loadedMin + 40)) {
      Reference_ADC[i] = loadedRef;
      Max_ADC[i]       = loadedMax;
      Min_ADC[i]       = loadedMin;
      validCount++;
    } else {
      // Fallback for this specific channel if uncalibrated
      Reference_ADC[i] = fallback_threshold;
      Max_ADC[i]       = 800;
      Min_ADC[i]       = 200;
    }
  }

  if (validCount >= (sensorNumber / 2)) {
    Serial.print(F("[EEPROM LOADED] Calibrated Midpoints: "));
    for (int i = 0; i < sensorNumber; i++) {
      Serial.print(Reference_ADC[i]);
      Serial.print(F(" "));
    }
    Serial.println();
  } else {
    Serial.println(F("[EEPROM NOTICE] No valid calibration profile found in EEPROM."));
    Serial.println(F("[EEPROM NOTICE] Using default fallback threshold (500). Please press Button 2 to Calibrate!"));
    for (int i = 0; i < sensorNumber; i++) {
      Reference_ADC[i] = fallback_threshold;
    }
  }
}
