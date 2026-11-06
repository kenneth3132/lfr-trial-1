# Combined Line Follower Robot (Parts 1 to 8) — Direct No-MUX Edition

[![Platform](https://img.shields.io/badge/Platform-Arduino%20Nano%20(ATmega328P)-blue.svg)](https://www.arduino.cc/)
[![Architecture](https://img.shields.io/badge/Architecture-Direct%20ADC%20(No%20MUX)-orange.svg)]()
[![Control](https://img.shields.io/badge/Control-Closed--Loop%20PD%20%2B%20State%20Machine-green.svg)]()
[![Calibration](https://img.shields.io/badge/Calibration-Autonomous%20Sweep%20%2B%20EEPROM-yellow.svg)]()

This folder contains a fully unified, multi-tab Arduino firmware uniting all functional principles from **Part 01 through Part 08** into an optimized, **multiplexer-free (No-MUX)** architecture specifically designed for the **Arduino Nano**.

---

## 1. Why Eliminate the Multiplexer (MUX)?

In earlier iterations (Parts 02–05), an analog multiplexer (e.g., CD74HC4067) was used to route 8 sensor signals to a single analog pin using 4 digital select lines (`S0`–`S3`).

While multiplexing reduces analog pin consumption, the **Arduino Nano (TQFP ATmega328P)** uniquely exposes **8 dedicated analog input pins (`A0` to `A7`)**. By connecting the 8 reflectance sensors directly to pins `A0`–`A7`:

1. **Hardware Simplification**: Eliminates the 16-pin or 24-pin MUX IC, reducing PCB footprint, solder joints, routing congestion, and failure points.
2. **Speed & Zero Channel-Switching Latency**: Eliminates the digital select pin manipulation and the `delayMicroseconds(20)` settling delay required between channels, boosting loop update frequency to **$>1.2\text{ kHz}$**.
3. **GPIO Conservation**: Eliminates the 4 digital select lines (`S0`–`S3`), freeing up digital pins for dedicated motor PWM, directional logic, user buttons, and diagnostic LEDs.
4. **Hardware-Matched PWM Timers**: Allows left and right motors to both use **Timer 0 PWM pins (`D5` and `D6`)**, guaranteeing perfectly identical PWM carrier frequencies (~976 Hz) for symmetrical motor response.

---

## 2. Master Pin Mapping (Arduino Nano)

| Function | Pin | Hardware Type | Description |
| :--- | :--- | :--- | :--- |
| **Sensor 0 (Far Left)** | `A0` | Analog In | Direct phototransistor analog input |
| **Sensor 1** | `A1` | Analog In | Direct phototransistor analog input |
| **Sensor 2** | `A2` | Analog In | Direct phototransistor analog input |
| **Sensor 3 (Mid Left)** | `A3` | Analog In | Direct phototransistor analog input |
| **Sensor 4 (Mid Right)** | `A4` | Analog In | Direct phototransistor analog input |
| **Sensor 5** | `A5` | Analog In | Direct phototransistor analog input |
| **Sensor 6** | `A6` | Analog In | Direct phototransistor analog input (Hardware analog-only pin) |
| **Sensor 7 (Far Right)** | `A7` | Analog In | Direct phototransistor analog input (Hardware analog-only pin) |
| **Left Motor PWM (PWMB)** | `D5` | Digital PWM | Timer 0 PWM (~976 Hz) speed control |
| **Left Motor IN3 (BIN1)** | `D2` | Digital Out | Direction logic |
| **Left Motor IN4 (BIN2)** | `D4` | Digital Out | Direction logic |
| **Right Motor PWM (PWMA)**| `D6` | Digital PWM | Timer 0 PWM (~976 Hz) speed control |
| **Right Motor IN1 (AIN1)** | `D7` | Digital Out | Direction logic |
| **Right Motor IN2 (AIN2)** | `D8` | Digital Out | Direction logic |
| **Button 1 (Start PID)** | `D11`| Digital In | Active-LOW with internal pull-up (`INPUT_PULLUP`) |
| **Button 2 (Calibrate)** | `D12`| Digital In | Active-LOW with internal pull-up (`INPUT_PULLUP`) |
| **Status / Turn Indicator**| `D13`| Digital Out | Built-in Arduino Nano LED |
| **Serial Telemetry** | `D0/D1` | Hardware UART | 9600 Baud serial communication |

---

## 3. Modular File Architecture

```
Combined_Part1_to_Part8_NoMUX/
├── Combined_Part1_to_Part8_NoMUX.ino  # Main entry point (Setup, Loop, Pin & Global Definitions)
├── read_sensor.ino                    # Direct A0-A7 analog sampling, dynamic thresholding, bitmask
├── motor.ino                          # Differential drive H-bridge control with signed PWM clamping
├── PID_Controller.ino                 # Closed-loop PD steering with 90° turn state arbitrator
├── Turns.ino                          # Differential in-place spin pivot and line recovery routines
├── SensorCalibration.ino              # 4-sweep rotational auto-calibration and EEPROM storage
├── distance.ino                       # Part 06 dead-reckoning distance movements and braking
├── README.md                          # Architecture & quick start manual (this file)
└── ANALYSIS_AND_VERIFICATION.md       # Rigorous 3x cross-check report (Electronics, Kinematics, etc.)
```

---

## 4. Integrated Feature Summary (Parts 1 to 8)

- **Part 01 (Power & Layout)**: Clean power separation between motor drive rail (7.4V/11.1V) and 5V logic bus with $470\mu\text{F}$–$1000\mu\text{F}$ decoupling capacitor.
- **Part 02 (8-IR Acquisition)**: Direct analog reading across 8 channels, dynamic threshold comparison, active sensor counting, and bitmask packing.
- **Part 03 (Motor Kinematics)**: Differential velocity handling via `motor(LPWM, RPWM)` with active electronic braking when commanded to 0.
- **Part 04 (PID Closed-Loop Control)**: Sub-millimeter position estimation using center-of-mass weighted averaging ($\text{center} = 45.0$). Proportional correction and derivative damping.
- **Part 05 (Sharp 90° Turn State Machine)**: Pattern matching on bitmask (`0b11110000`, `0b00001111`, etc.) remembers turn direction. When the line is lost, an in-place differential pivot is executed until the center sensor recaptures the line.
- **Part 06 (Trajectory & Braking)**: Timed dead-reckoning movement (`distance(cm)`) for crossing broken track segments.
- **Part 07 (Dynamic Auto-Calibration)**: Robot rotates left and right across the line across 4 sweeps, tracks individual minimums and maximums, calculates dynamic midpoints, and writes them to non-volatile EEPROM.
- **Part 08 (Inverted Track Support)**: Toggle `inverseON = 1` in firmware to seamlessly run on white tracks with black backgrounds without modifying the downstream PID math.

---

## 5. Operating Instructions

### Step 1: Uploading the Firmware
1. Open [`Combined_Part1_to_Part8_NoMUX.ino`](file:///C:/Users/kenne/Desktop/lfr/Combined_Part1_to_Part8_NoMUX/Combined_Part1_to_Part8_NoMUX.ino) in the Arduino IDE.
2. Select **Board**: `Arduino Nano`, **Processor**: `ATmega328P` (or `ATmega328P (Old Bootloader)` depending on your Nano clone).
3. Connect USB and click **Upload**.

### Step 2: Calibrating the Robot (Button 2)
1. Place the robot over the track with the line centered beneath the sensor array.
2. Press **Button 2 (Pin D12)**.
3. The robot will execute 4 rotational sweeps across the line while recording surface reflectance.
4. The status LED (`D13`) will turn off when calibration is complete. Stored thresholds are permanently written to EEPROM.

### Step 3: High-Speed Line Following (Button 1)
1. Place the robot at the starting gate.
2. Press **Button 1 (Pin D11)**.
3. The robot will launch into continuous high-speed closed-loop PID tracking.
