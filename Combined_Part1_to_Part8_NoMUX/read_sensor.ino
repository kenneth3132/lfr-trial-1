/**
 * ====================================================================================================
 * MODULE: SENSOR ACQUISITION & CENTROID CALCULATION (DIRECT NO-MUX MODE)
 * ====================================================================================================
 * 
 * DESCRIPTION:
 * Reads all 8 TCRT5000 IR phototransistors directly through Arduino Nano analog pins A0-A7.
 * Eliminates multiplexer switching latency, settling delays, and digital select line toggling.
 * 
 * KEY OPERATIONS:
 * 1. Direct ADC Reading: Samples each pin (A0-A7) sequentially using ATmega328P 10-bit ADC.
 * 2. Dynamic Threshold Comparison: Evaluates reading against calibrated Reference_ADC[i].
 * 3. Track Polarity Normalization: Applies inverseON logic to support both black and white tracks.
 * 4. Centroid Math: Accumulates active sensor count (sumOnSensor) and weighted moments (sensorWight).
 * 5. Bitmask Synthesis: Constructs an 8-bit packed representation (bitSensor) for fast state checking.
 */

#include <Arduino.h>

void read_black_line() {
  sumOnSensor = 0;
  sensorWight = 0;
  bitSensor   = 0;

  for (int i = 0; i < sensorNumber; i++) {
    // 1. DIRECT ANALOG READ (No MUX channel switching required)
    sensorADC[i] = analogRead(sensorPins[i]);

    // 2. THRESHOLD SELECTION (Calibrated per-sensor Reference or fallback)
    int thresh = (Reference_ADC[i] > 50) ? Reference_ADC[i] : fallback_threshold;

    // 3. DIGITAL BINARIZATION WITH NORMAL / INVERSE SUPPORT
    // Higher ADC value = Darker surface (greater IR absorption, higher phototransistor resistance)
    if (sensorADC[i] > thresh) {
      if (inverseON == 0) {
        sensorDigital[i] = 1;  // Black line on White background (Normal Mode)
      } else {
        sensorDigital[i] = 0;  // Black line in Inverted Mode
      }
    } else {
      if (inverseON == 0) {
        sensorDigital[i] = 0;  // White background in Normal Mode
      } else {
        sensorDigital[i] = 1;  // White line on Black background (Inverted Mode)
      }
    }

    // 4. ACCUMULATE MOMENTS FOR CENTROID CALCULATION
    sumOnSensor += sensorDigital[i];
    sensorWight += (long)sensorDigital[i] * WeightValue[i];

    // 5. PACK SENSOR STATES INTO AN 8-BIT BITMASK
    // S0 (Leftmost) maps to MSB (bit 7, 0b10000000)
    // S7 (Rightmost) maps to LSB (bit 0, 0b00000001)
    bitSensor += sensorDigital[i] * bitWeight[(sensorNumber - 1) - i];
  }
}

/**
 * Diagnostic utility: Continuously prints raw 10-bit ADC values (0-1023)
 * Use this to verify ambient reflectance and sensor ground clearance.
 */
void sensor_value_test() {
  Serial.println(F("--- Live Analog ADC Stream (A0 to A7) ---"));
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
 * Diagnostic utility: Continuously displays binary bitmask (e.g., 00011000)
 * Use this to verify sensor alignment, threshold crossing, and line detection.
 */
void Bit_Sensor_Show() {
  Serial.println(F("--- Live Digital Bitmask Stream [S0(Left) -> S7(Right)] ---"));
  while (1) {
    read_black_line();
    Serial.print(F("Dec: "));
    Serial.print(bitSensor);
    Serial.print(F("\tBin: "));
    for (int i = (sensorNumber - 1); i >= 0; i--) {
      Serial.print(bitRead(bitSensor, i));
    }
    Serial.print(F("\tActive Count: "));
    Serial.print(sumOnSensor);
    Serial.print(F("\tWeighted Pos: "));
    if (sumOnSensor > 0) {
      Serial.print((float)sensorWight / sumOnSensor);
    } else {
      Serial.print(F("NONE"));
    }
    Serial.println();
    delay(80);
  }
}
