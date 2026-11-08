# Combined Line Follower Robot (Parts 1 to 8) — Enhanced No-MUX Edition

[![Platform](https://img.shields.io/badge/Platform-Arduino%20Nano%20(ATmega328P)-blue.svg)](https://www.arduino.cc/)
[![Architecture](https://img.shields.io/badge/Architecture-Direct%20ADC%20(No%20MUX)-orange.svg)]()
[![Control](https://img.shields.io/badge/Control-Closed--Loop%20PD%20%2B%20State%20Machine-green.svg)]()
[![Status](https://img.shields.io/badge/Build-Verified%20%26%20Enhanced-brightgreen.svg)]()

This folder contains the complete, verified, and heavily commented production Arduino firmware uniting **Part 01 through Part 08** into an optimized **direct analog (No-MUX)** architecture for the **Arduino Nano**.

---

## 1. Key Enhancements & Bug Fixes in This Build

1. **Button 1 Diagnostics & Real-Time Idle Stream**:
   - Streams button states (`Btn1(D11): PRESSED/HIGH`, `Btn2(D12): PRESSED/HIGH`), active sensor count, and computed line centroid every 500ms on Serial (9600 Baud).
   - If your Button 1 wiring is loose or on the wrong pin, you see it instantly on the Serial Monitor.
2. **3-Flash Safety Launch Countdown**:
   - Pressing Button 1 triggers a 3-flash LED countdown (1000ms total) before motors engage, allowing you to release the robot safely.
3. **Emergency Stop / User Halt Breakout**:
   - Pressing Button 2 during PID tracking immediately engages active electronic motor braking (`motor(0, 0)`), turns off the LED, and cleanly returns to idle mode.
4. **90° Turn Watchdog Timeout (1500ms)**:
   - Prevents the robot from getting trapped in infinite pivot loops if lifted off the track or if it loses line lock during a sharp turn.
5. **Sticky Direction State Memory Fix**:
   - Centered and mild curve patterns (`0b00110000`, `0b00001100`, etc.) now reset `direction = "straight"`, preventing false turns on track gaps.
6. **Off-Track Runaway Protection**:
   - If the robot loses the line completely on a straight line for $>350\text{ ms}$, motors automatically brake to prevent crashing.
7. **Safe EEPROM Threshold Storage**:
   - Uses `EEPROM.update()` to preserve EEPROM cell lifespan and validates sensors individually rather than discarding all 8 sensors.

---

## 2. Directory Structure

```
Combined_Part1_to_Part8_NoMUX_Fixed/
├── Combined_Part1_to_Part8_NoMUX_Fixed.ino  # Main entry point (Setup, Loop, Controls & Globals)
├── read_sensor.ino                          # Direct A0-A7 ADC sampling, binarization, centroid math
├── PID_Controller.ino                       # Closed-loop PD steering with 90° state arbitrator
├── Turns.ino                                # Differential in-place spin pivots with 1.5s watchdog
├── SensorCalibration.ino                    # 4-sweep auto-calibration & EEPROM.update persistence
├── motor.ino                                # Differential drive H-bridge driver with signed PWM clamping
├── distance.ino                             # Part 06 dead-reckoning distance movements and braking
├── connections.txt                          # Comprehensive wiring guide in clean tabular format
├── functions and working.txt                # Chronological breakdown of all functions in order of execution
└── README.md                                # Quick start and architecture guide (this file)
```

---

## 3. Quick Pin & Button Mapping

| Control / Subsystem | Nano Pin | Wire Leg 1 | Wire Leg 2 | Operating Behavior |
| :--- | :--- | :--- | :--- | :--- |
| **Button 1 (START)** | **`D11`** | Pin `D11` | Common `GND` | Active-LOW (`INPUT_PULLUP`). Launches PID tracking. |
| **Button 2 (CALIB / HALT)** | **`D12`** | Pin `D12` | Common `GND` | Active-LOW (`INPUT_PULLUP`). In idle: Calibrates. During run: Emergency Stop. |
| **Status / Turn LED** | **`D13`** | On-board | On-board | Blinks 3x on countdown; On during turns & calib. |
| **IR Sensors (8 Channels)**| **`A0` to `A7`** | Sensor OUT | 5V & GND | Direct analog phototransistor inputs (No MUX). |
| **Left Motor (PWM / IN3 / IN4)** | **`D5` / `D2` / `D4`** | Driver PWMB | BIN1 / BIN2 | Timer 0 PWM speed and direction logic. |
| **Right Motor (PWM / IN1 / IN2)**| **`D6` / `D7` / `D8`** | Driver PWMA | AIN1 / AIN2 | Timer 0 PWM speed and direction logic. |

> [!IMPORTANT]
> **TB6612FNG Users**: You **MUST** connect the motor driver `STBY` pin to **`5V`**. If `STBY` is floating or grounded, the motor driver is disabled and motors will not move!

---

## 4. Operating Instructions

### Step 1: Upload Firmware
1. Open [`Combined_Part1_to_Part8_NoMUX_Fixed.ino`](file:///C:/Users/kenne/Desktop/lfr/Combined_Part1_to_Part8_NoMUX_Fixed/Combined_Part1_to_Part8_NoMUX_Fixed.ino) in the Arduino IDE.
2. Select **Board**: `Arduino Nano`, **Processor**: `ATmega328P` (or `Old Bootloader` for clones).
3. Connect USB and click **Upload**.
4. Open the Serial Monitor at **9600 Baud**.

### Step 2: Auto-Calibration (Button 2)
1. Place the robot over the track with the black line centered under the sensors.
2. Press **Button 2 (Pin D12)**.
3. The robot will execute 4 rotational sweeps across the track, recording minimum and maximum reflectance for all 8 sensors, and permanently save the midpoints to EEPROM.

### Step 3: PID Line Tracking (Button 1)
1. Place the robot at the starting gate.
2. Press **Button 1 (Pin D11)**.
3. The LED will flash 3 times (1-second safety countdown).
4. The robot launches into high-speed closed-loop PID tracking.
5. **Emergency Stop**: Press **Button 2 (Pin D12)** at any time during the run to halt the robot instantly.
