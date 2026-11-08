/**
 * ====================================================================================================
 * MODULE: SENSOR ACQUISITION & CENTROID POSITION CALCULATION (DIRECT NO-MUX MODE)
 * TARGET HARDWARE: ARDUINO NANO (PINS A0 TO A7)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Reads all 8 TCRT5000 infrared reflectance sensors directly through the Arduino Nano analog inputs
 * (A0 to A7). Eliminating the CD74HC4067 multiplexer removes channel-switching latency, select line
 * GPIO overhead, and settling delays, achieving loop rates in excess of 1.2 kHz.
 * 
 * LOGIC STEPS:
 * 1. Direct ADC Sampling: Successive 10-bit analog conversion across all 8 pins (0 to 1023).
 * 2. Threshold Binarization: Compares raw ADC against calibrated per-sensor Reference_ADC[i].
 * 3. Track Polarity Inversion (inverseON):
 *    - Normal Mode (0): Dark line on light background -> ADC > Threshold yields digital '1'.
 *    - Inverted Mode (1): Light line on dark background -> ADC <= Threshold yields digital '1'.
 * 4. Weighted Centroid Accumulation:
 *    - sumOnSensor = count of active sensors detecting the track.
 *    - sensorWight = sum of (sensorDigital[i] * WeightValue[i]).
 *    - line_position = sensorWight / sumOnSensor.
 * 5. Bitmask Synthesis: Constructs an 8-bit packed binary byte (bitSensor) for instant pattern matching.
 *    S0 (Far Left) = Bit 7 (0b10000000), S7 (Far Right) = Bit 0 (0b00000001).
 */

#include <Arduino.h>

void read_black_line() {
  // Clear accumulators for the new acquisition cycle
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor   = 0;

  // Sample all 8 IR phototransistor channels sequentially
  for (int i = 0; i < sensorNumber; i++) {
    // 1. DIRECT ANALOG ACQUISITION (No multiplexer select lines or settling delay)
    sensorADC[i] = analogRead(sensorPins[i]);

    // 2. DYNAMIC THRESHOLD SELECTION
    // Use calibrated per-sensor midpoint threshold if valid (> 0), otherwise use fallback (500)
    int thresh = (Reference_ADC[i] > 0) ? Reference_ADC[i] : fallback_threshold;

    // 3. DIGITAL BINARIZATION WITH INVERSION SUPPORT (Part 08)
    // Phototransistor physics: Higher ADC reading indicates less reflected IR (darker surface).
    if (sensorADC[i] > thresh) {
      if (inverseON == 0) {
        sensorDigital[i] = 1;  // Black line detected on white background (Normal Mode)
      } else {
        sensorDigital[i] = 0;  // Black background in Inverted Mode
      }
    } else {
      if (inverseON == 0) {
        sensorDigital[i] = 0;  // White background in Normal Mode
      } else {
        sensorDigital[i] = 1;  // White line detected on black background (Inverted Mode)
      }
    }

    // 4. ACCUMULATE MOMENTS FOR CENTER-OF-MASS WEIGHTED POSITION
    sumOnSensor += sensorDigital[i];
    sensorWight += (long)sensorDigital[i] * WeightValue[i];

    // 5. PACK SENSOR STATES INTO 8-BIT BITMASK
    // S0 maps to Bit 7 (MSB: 128), S7 maps to Bit 0 (LSB: 1)
    bitSensor |= (sensorDigital[i] * bitWeight[(sensorNumber - 1) - i]);
  }
}

/**
 * Diagnostic Tool: Continuously streams raw 10-bit ADC values (0 to 1023) for all 8 sensors.
 * Use this to check ambient optical noise, surface reflectance, and sensor mounting height (3-5mm).
 */
void sensor_value_test() {
  Serial.println(F("\n--- Live Raw Analog ADC Stream (Pins A0 to A7) ---"));
  Serial.println(F("S0(A0)\tS1(A1)\tS2(A2)\tS3(A3)\tS4(A4)\tS5(A5)\tS6(A6)\tS7(A7)"));
  
  while (1) {
    for (int i = 0; i < sensorNumber; i++) {
      sensorADC[i] = analogRead(sensorPins[i]);
      Serial.print(sensorADC[i]);
      Serial.print(F("\t"));
    }
    Serial.println();
    delay(100);
  }
}

/**
 * Diagnostic Tool: Continuously displays the 8-bit digital bitmask and computed centroid position.
 * Use this to verify threshold crossing, sensor alignment, and track centering.
 */
void Bit_Sensor_Show() {
  Serial.println(F("\n--- Live Digital Bitmask Stream [S0(Left) -> S7(Right)] ---"));
  while (1) {
    read_black_line();
    Serial.print(F("Dec: "));
    Serial.print(bitSensor);
    Serial.print(F("\tBitmask: "));
    for (int i = (sensorNumber - 1); i >= 0; i--) {
      Serial.print(bitRead(bitSensor, i));
    }
    Serial.print(F("\tActive Count: "));
    Serial.print(sumOnSensor);
    Serial.print(F("\tCentroid: "));
    if (sumOnSensor > 0) {
      Serial.print((float)sensorWight / (float)sumOnSensor);
    } else {
      Serial.print(F("OFF_LINE"));
    }
    Serial.println();
    delay(80);
  }
}
