# High-Speed PID Line Follower Robot (LFR) — Complete Engineering Guide

[![Platform](https://img.shields.io/badge/Platform-Arduino%20Nano%20%7C%20Uno%20%7C%20ESP32-blue.svg)](https://www.arduino.cc/)
[![Algorithm](https://img.shields.io/badge/Algorithm-PID%20Closed--Loop%20Control-green.svg)]()
[![Sensors](https://img.shields.io/badge/Sensors-8--Channel%20IR%20Array%20%2B%20MUX-orange.svg)]()
[![Display](https://img.shields.io/badge/UI-SSD1306%20OLED%20(128x64)-yellow.svg)]()

Welcome to the comprehensive technical documentation and engineering reference for the **Fast PID Line Follower Robot (LFR)** tutorial series created by [Aslam Hossain (RoboTech Innovator)](https://github.com/AslamHossain-YT/Line-Follower-Robot-Coding-Tutorial).

This repository contains the complete progressive evolution of firmware, schematics, simulations, and diagnostic tools for building a competition-grade Line Follower Robot from first principles up to a standalone menu-driven robot with dynamic sensor auto-calibration and non-volatile EEPROM storage.

---

## Table of Contents
1. [System Overview & Architecture](#system-overview--architecture)
2. [Hardware Bill of Materials (BOM)](#hardware-bill-of-materials-bom)
3. [Repository Directory Structure](#repository-directory-structure)
4. [Master Pin Mapping & Hardware Matrix](#master-pin-mapping--hardware-matrix)
5. [In-Depth Breakdown of Every Part (01 to 11)](#in-depth-breakdown-of-every-part-01-to-11)
   - [Part-01: Circuit Architecture & Power Distribution](#part-01-circuit-architecture--power-distribution)
   - [Part-02: 8-IR Sensor Array & Multiplexer Interfacing](#part-02-8-ir-sensor-array--multiplexer-interfacing)
   - [Part-03: Dual Motor Driver & Proteus Simulation](#part-03-dual-motor-driver--proteus-simulation)
   - [Part-04: PID Closed-Loop Algorithm Implementation](#part-04-pid-closed-loop-algorithm-implementation)
   - [Part-05: 90° Turn, Intersection & Line-Loss Handling](#part-05-90-turn-intersection--line-loss-handling)
   - [Part-06: Trajectory Speed Profiles & Distance Calibration](#part-06-trajectory-speed-profiles--distance-calibration)
   - [Part-07: Dynamic Auto-Calibration & EEPROM Persistence](#part-07-dynamic-auto-calibration--eeprom-persistence)
   - [Part-08: Inverse Track Mode (Black/White Inversion)](#part-08-inverse-track-mode-blackwhite-inversion)
   - [Part-09: On-Board OLED Interactive Menu System](#part-09-on-board-oled-interactive-menu-system)
   - [Part-10: Master Integrated Production Firmware](#part-10-master-integrated-production-firmware)
   - [Part-11: Hardware Diagnostics, Calibration & Platform Binaries](#part-11-hardware-diagnostics-calibration--platform-binaries)
6. [Mathematical Foundations & PID Theory](#mathematical-foundations--pid-theory)
7. [Tuning & Calibration Procedures](#tuning--calibration-procedures)
8. [Troubleshooting Guide](#troubleshooting-guide)

---

## System Overview & Architecture

A competitive Line Follower Robot operates on high-speed closed-loop feedback:
```
+-------------------------------------------------------------------------+
|                           LINE FOLLOWER ROBOT                           |
|                                                                         |
|  +--------------------+        +---------------------+                  |
|  |  8-IR TCRT5000     |------->| 16-Ch MUX (CD74HC)  |                  |
|  |  Sensor Array      |        | S0, S1, S2, S3      |                  |
|  +--------------------+        +----------+----------+                  |
|                                           | SIG (Analog Pin)            |
|                                           v                             |
|                                +---------------------+                  |
|  +--------------------+        | Arduino Nano (MCU)  |                  |
|  | Push Buttons &     |<------>| - Position Math     |                  |
|  | SSD1306 OLED UI    |        | - PID Closed Loop   |                  |
|  +--------------------+        | - State Machine     |                  |
|                                +----------+----------+                  |
|                                           | PWM / Direction Logic       |
|                                           v                             |
|                                +---------------------+                  |
|                                | Dual Motor Driver   |                  |
|                                | (TB6612 / L298N)    |                  |
|                                +----------+----------+                  |
|                                           |                             |
|                     +---------------------+---------------------+       |
|                     v                                           v       |
|            +-----------------+                         +-----------------+
|            | Left N20 Motor  |                         | Right N20 Motor |
|            +-----------------+                         +-----------------+
+-------------------------------------------------------------------------+
```

### Core Performance Principles:
1. **Low-Latency Multiplexing**: Reads 8 analog reflectance sensors through 4 digital select lines (`S0`–`S3`) and 1 analog channel, conserving MCU pins while achieving high sample rates.
2. **Weighted Position Estimation**: Normalizes sensor readings into sub-millimeter line position errors.
3. **Proportional-Derivative (PD/PID) Steering**: Provides smooth high-speed line tracking with proportional corrections and derivative damping that prevents high-frequency oscillations.
4. **State-Machine Turn Handling**: Recognizes acute turns, 90° bends, T-junctions, and full track departures using historical directional memory.
5. **Dynamic Self-Calibration**: Eliminates manual potentiometer adjustments by rotating over the track surface, measuring dynamic ambient minimums and maximums, and storing per-sensor thresholds in non-volatile EEPROM.

---

## Hardware Bill of Materials (BOM)

| Component | Recommended Specification | Function |
| :--- | :--- | :--- |
| **Microcontroller** | Arduino Nano (ATmega328P, 16MHz) or ESP32 | Main computation, PID loop, display rendering |
| **Sensor Array** | 8x TCRT5000 Phototransistors or QTR-8A module | Ground reflectance sensing (IR emitter + detector) |
| **Multiplexer** | CD74HC4067 (16-channel) or 74HC4051 (8-channel) | Analog channel routing to a single ADC pin |
| **Motor Driver** | TB6612FNG Dual MOSFET H-Bridge (or L298N) | Dual DC motor speed and direction driver |
| **Motors** | 2x N20 Micro Metal Gearmotors (6V, 500–1000 RPM) | Low inertia, high torque differential drive |
| **Wheels** | Silicone high-grip mini racing wheels (32–42mm) | High coefficient of friction for sharp turning |
| **Display** | 0.96" I2C OLED (SSD1306, 128x64 pixels) | On-board menu, live sensor visualization, tuning |
| **Inputs** | 3x Tactile Push Buttons (Up, Select, Down) | Menu navigation, calibration trigger, mode selection |
| **Power Supply** | 2S (7.4V) or 3S (11.1V) LiPo Battery (450–850mAh) | High burst discharge for aggressive acceleration |
| **Voltage Regulator** | LM7805 or Buck Converter (5V 2A Output) | Stable 5V logic rail for MCU, sensors, and OLED |
| **Chassis** | Custom 3D Printed / Laser-cut Acrylic or PCB chassis | Compact, lightweight, low center of gravity |

---

## Repository Directory Structure

```
lfr/
├── README.md                      # Complete Technical Guide (this file)
├── IMPLEMENTATION_PLAN.md         # Phased Step-by-Step Implementation Plan
├── Part-01 Files/                 # Initial Circuit Schematics & Power
│   └── Circuit-Diagram.jpg
├── Part-02 Files/                 # MUX Analog Reading & Bitmask Math
│   ├── Circuit-Diagram.jpg
│   └── 8IR_sensor_reading/
│       ├── 8IR_sensor_reading.ino
│       └── read_sensor.ino
├── Part-03 Files/                 # Motor Control & Proteus Simulation
│   ├── Diagram 1.jpg
│   ├── Diagram 2.jpg
│   ├── Proteus Simulation/
│   │   ├── motor_simulation_proteus.pdsprj
│   │   └── motor_simulation_proteus.pdsprj...
│   └── 8IR_MUX_Motor/
│       ├── 8IR_MUX_Motor.ino
│       ├── motor.ino
│       └── read_sensor.ino
├── Part-04 Files/                 # Proportional-Derivative (PID) Controller
│   ├── Circuit-Diagram.jpg
│   └── 8IR_MUX_PID_Algorithm/
│       ├── 8IR_MUX_PID_Algorithm.ino
│       ├── PID_Controller.ino
│       ├── motor.ino
│       └── read_sensor.ino
├── Part-05 Files/                 # 90° Sharp Turn & Junction Logic
│   ├── Circuit-Diagram.jpg
│   └── 8IR_MUX_PID_Turns/...
│       ├── 8IR_MUX_PID_Turns.ino
│       ├── PID_Controller.ino
│       ├── Turns.ino
│       ├── motor.ino
│       └── read_sensor.ino
├── Part-06 Files/                 # Trajectory Braking & Distance Tracking
│   └── 8IR_MUX_PID_Part-06/
│       ├── 8IR_MUX_PID_Part-06.ino
│       ├── PID_Controller.ino
│       ├── Turns.ino
│       ├── motor.ino
│       └── read_sensor.ino
├── Part-07 Files/                 # Dynamic Auto-Calibration & EEPROM
│   └── 8IR_MUX_PID_Part-07/
│       ├── 8IR_MUX_PID_Part-07.ino
│       ├── PID_Controller.ino
│       ├── SensorCalibration.ino
│       ├── Turns.ino
│       ├── motor.ino
│       └── read_sensor.ino
├── Part-08 Files/                 # Inverted Track Mode (Normal vs. Inverted)
│   └── 8IR_MUX_PID_Part-08/
│       ├── 8IR_MUX_PID_Part-08.ino
│       ├── PID_Controller.ino
│       ├── SensorCalibration.ino
│       ├── Turns.ino
│       ├── motor.ino
│       └── read_sensor.ino
├── Part-09 Files/                 # Interactive SSD1306 OLED UI & Menu System
│   └── 8IR_MUX_PID_Part-09/
│       ├── 8IR_MUX_PID_Part-09.ino
│       ├── PID_Controller.ino
│       ├── SensorCalibration.ino
│       ├── Turns.ino
│       ├── function_button.ino
│       ├── menu.ino
│       ├── motor.ino
│       └── read_sensor.ino
├── Part-10 Files/                 # Master Integrated Production System
│   ├── 1.jpg
│   ├── Circuit-Diagram.jpg
│   ├── Sensor Connection.jpg
│   ├── Specification.jpg
│   └── 8IR_MUX_PID_Part-10/
│       ├── 8IR_MUX_PID_Part-10.ino
│       ├── PID_Controller.ino
│       ├── SensorCalibration.ino
│       ├── Turns.ino
│       ├── function_button.ino
│       ├── menu.ino
│       ├── motor.ino
│       └── read_sensor.ino
└── Part-11 Files/                 # Diagnostics, Calibrator & Binaries
    ├── Steps.pdf                  # Step-by-step pictorial reference guide
    ├── Sensor-Analog-Read/        # Raw multi-channel ADC monitor
    │   └── Sensor-Analog-Read.ino
    └── tcrt-test/                 # Single sensor sanity check & compiled firmware
        ├── tcrt-test.ino
        └── build/                 # Precompiled ELF/HEX/BIN for Nano & ESP32
```

---

## Master Pin Mapping & Hardware Matrix

As features expanded across parts (adding OLED displays and buttons), pin assignments were optimized to free up hardware I2C (`A4`/`A5`) and analog pins. The table below provides a unified comparison:

| Signal Function | Part 02 - 05 | Part 06 | Part 07 - 08 | Part 10 (Master Final) | Notes |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Right Motor IN1 (AIN2)** | `D2` | `A2` | `A2` | `D6` | Direction logic |
| **Right Motor IN2 (AIN1)** | `D4` | `D4` | `D4` | `D4` | Direction logic |
| **Left Motor IN3 (BIN1)** | `D5` | `D5` | `D5` | `D3` | Direction logic |
| **Left Motor IN4 (BIN2)** | `D6` | `D6` | `D6` | `D2` | Direction logic |
| **Right Motor PWM (PWMA)** | `D3` (PWM) | `D3` (PWM) | `D3` (PWM) | `D9` (PWM) | Timer PWM speed |
| **Left Motor PWM (PWMB)** | `D9` (PWM) | `D9` (PWM) | `D9` (PWM) | `D5` (PWM) | Timer PWM speed |
| **MUX Select Line S0** | `D11` | `D11` | `D11` | `D8` | LSB binary channel select |
| **MUX Select Line S1** | `D10` | `D10` | `D10` | `D10` | Binary channel select |
| **MUX Select Line S2** | `D8` | `D8` | `D8` | `D11` | Binary channel select |
| **MUX Select Line S3** | `D7` | `D7` | `D7` | `D12` | MSB binary channel select |
| **MUX Analog In (SIG)** | `A3` | `A7` | `A7` | `A7` | High-impedance analog input |
| **OLED SDA** | — | — | — | `A4` (SDA) | Hardware I2C Data |
| **OLED SCL** | — | — | — | `A5` (SCL) | Hardware I2C Clock |
| **Button 1 / Up Button** | `A0` | `A0` | `A0` | `D7` | Active-low with internal pull-up |
| **Button 2 / Down Button** | `D12` | `A1` | `A5` | `A3` | Active-low with internal pull-up |
| **Button 3 / Select Button** | — | `D12` | `D12` | `A2` | Active-low with internal pull-up |
| **LED Status Indicator** | — | `D13` | `D13` | `D13` | On-board LED indicator |

---

## In-Depth Breakdown of Every Part (01 to 11)

### Part-01: Circuit Architecture & Power Distribution
- **Reference Diagram**: [`Part-01 Files/Circuit-Diagram.jpg`](file:///C:/Users/kenne/Desktop/lfr/Part-01%20Files/Circuit-Diagram.jpg)
- **Objective**: Establish clean, decoupled power delivery and isolation between inductive motor loads and sensitive analog logic.
- **Key Concepts**:
  - **Dual Power Rail Architecture**: Direct battery voltage (7.4V–11.1V) powers the motor driver `VM` / `Vcc2` terminal, while a regulated 5V rail powers the Arduino Nano `5V` pin, sensor array, and multiplexer.
  - **Decoupling Capacitors**: Prevents micro-brownouts caused by motor inrush currents from resetting the ATmega328P.
  - **Common Ground Plane**: All grounds (Battery (-), Motor Driver GND, Arduino GND, and Sensor GND) are tied into a star-ground configuration to eliminate ground bounce.

---

### Part-02: 8-IR Sensor Array & Multiplexer Interfacing
- **Source Code**: [`Part-02 Files/8IR_sensor_reading/`](file:///C:/Users/kenne/Desktop/lfr/Part-02%20Files/8IR_sensor_reading)
- **Objective**: Multiplex 8 reflectance channels onto 1 ADC pin and compute digital bitmasks and weighted centroids.
- **Implementation Details**:
  - The 16-channel analog multiplexer (CD74HC4067) uses digital select pins `S0`–`S3` to route sensor channels 0 to 7 to pin `SIG_PIN` (`A3`):
    ```cpp
    void selectChannel(int channel) {
      digitalWrite(S0, bitRead(channel, 0));
      digitalWrite(S1, bitRead(channel, 1));
      digitalWrite(S2, bitRead(channel, 2));
      digitalWrite(S3, bitRead(channel, 3));
    }
    ```
  - **Settling Delay**: A brief 20 µs delay (`delayMicroseconds(20)`) is introduced after switching channels before `analogRead()` is triggered, allowing the multiplexer's internal RC transient to settle.
  - **Thresholding & Representation**:
    - `sensorADC[i]` raw value (0–1023) is evaluated against `theshold = 500`.
    - If `sensorADC[i] > theshold`, `sensorDigital[i] = 1` (line detected); otherwise `0`.
    - `bitSensor`: An 8-bit integer where each bit corresponds directly to a physical sensor's digital state (e.g., `0b00011000` indicates line centered).
    - `sensorWight`: Sum of activated sensor weights (`WeightValue = {10, 20, 30, 40, 50, 60, 70, 80}`).
    - `sumOnSensor`: Count of active sensors (used as denominator for position calculation).

---

### Part-03: Dual Motor Driver & Proteus Simulation
- **Source Code**: [`Part-03 Files/8IR_MUX_Motor/`](file:///C:/Users/kenne/Desktop/lfr/Part-03%20Files/8IR_MUX_Motor)
- **Simulation**: [`Part-03 Files/Proteus Simulation/`](file:///C:/Users/kenne/Desktop/lfr/Part-03%20Files/Proteus%20Simulation)
- **Objective**: Implement bidirectional differential drive motor functions and validate electrical schematics under Proteus CAD simulation.
- **Implementation Details**:
  - Function `motor(int LPWM, int RPWM)` handles signed velocity inputs (`-255` to `+255`):
    - Positive values configure Forward logic (`IN1=HIGH, IN2=LOW` or `IN3=HIGH, IN4=LOW`).
    - Negative values configure Reverse logic (`IN1=LOW, IN2=HIGH` or `IN3=LOW, IN4=HIGH`).
    - Speed is modulated using 8-bit hardware timer PWM (`analogWrite()` on pins `3` and `9`).
  - Inputs are clamped safely using `constrain(LPWM, -255, 255)`.
  - Button 1 tests the sensor array; Button 2 performs a full-throttle forward motor spin test.

---

### Part-04: PID Closed-Loop Algorithm Implementation
- **Source Code**: [`Part-04 Files/8IR_MUX_PID_Algorithm/`](file:///C:/Users/kenne/Desktop/lfr/Part-04%20Files/8IR_MUX_PID_Algorithm)
- **Objective**: Implement the Proportional-Derivative (PD) control algorithm for smooth line following without overshooting.
- **Mathematical Logic**:
  1. **Line Position Calculation**:
     $$\text{line\_position} = \frac{\sum (\text{sensorDigital}[i] \times \text{WeightValue}[i])}{\sum \text{sensorDigital}[i]} = \frac{\text{sensorWight}}{\text{sumOnSensor}}$$
     With 8 sensors weighted $10, 20, \dots, 80$, the exact geometric center is:
     $$\text{center\_position} = 45.0$$
  2. **Error Calculation**:
     $$\text{error} = \text{center\_position} - \text{line\_position}$$
     - Negative error: Robot shifted to the left (line is right of center).
     - Positive error: Robot shifted to the right (line is left of center).
  3. **Derivative (Rate of Change)**:
     $$\text{derivative} = \text{error} - \text{previous\_error}$$
  4. **Motor Speed Correction**:
     $$\text{correction} = (K_p \times \text{error}) + (K_d \times \text{derivative})$$
     $$\text{left\_motor\_speed} = \text{base\_speed} - \text{correction}$$
     $$\text{right\_motor\_speed} = \text{base\_speed} + \text{correction}$$

---

### Part-05: 90° Turn, Intersection & Line-Loss Handling
- **Source Code**: [`Part-05 Files/8IR_MUX_PID_Turns/`](file:///C:/Users/kenne/Desktop/lfr/Part-05%20Files/8IR_MUX_PID_Turns)
- **Objective**: Solve acute angles, 90° right-angle bends, and T-junctions using pattern matching and state memory.
- **Key Algorithms**:
  - **Turn Prediction by Bitmask**:
    - Left-biased patterns (`0b11111000`, `0b11110000`, `0b11111100`) set `direction = "left"`.
    - Right-biased patterns (`0b00011111`, `0b00001111`, `0b00111111`) set `direction = "right"`.
  - **Turn Execution on White Space (`bitSensor == 0`)**:
    - When all sensors lose the line immediately after detecting a biased pattern, a 90° turn is confirmed.
    - An LED indicator illuminates on pin `13`.
    - `turnLeft(turnSpeed, turnSpeed)` or `turnRight(turnSpeed, turnSpeed)` executes differential spin:
      ```cpp
      void turnRight(int L, int R) {
        while (sensorDigital[4] != 1) { // Pivot until center sensor recaptures line
          motor(L, -R);
          read_black_line();
          direction = "straight";
        }
      }
      ```
    - The robot pivots in-place until the center sensor (`sensorDigital[4]`) recaptures the line, returning control to the PID loop.

---

### Part-06: Trajectory Speed Profiles & Distance Calibration
- **Source Code**: [`Part-06 Files/8IR_MUX_PID_Part-06/`](file:///C:/Users/kenne/Desktop/lfr/Part-06%20Files/8IR_MUX_PID_Part-06)
- **Objective**: Calibrate distance-based braking and introduce fine-tuned delay profiles prior to turn maneuvers.
- **Enhancements**:
  - Introduced `distance(10)` testing function allowing open-loop pulse timing (`test_time = 500 ms`) for dead reckoning over measured track segments.
  - Repurposed pins: shifted `right_motor_forward` to `A2` and `SIG_PIN` to `A7` (dedicated analog-only pin on the ATmega328P Nano), optimizing standard digital GPIO availability.

---

### Part-07: Dynamic Auto-Calibration & EEPROM Persistence
- **Source Code**: [`Part-07 Files/8IR_MUX_PID_Part-07/`](file:///C:/Users/kenne/Desktop/lfr/Part-07%20Files/8IR_MUX_PID_Part-07)
- **Objective**: Eliminate hardcoded thresholds by dynamically scanning track reflectance and saving calibration profiles to non-volatile EEPROM.
- **Calibration Algorithm**:
  1. Robot oscillates left and right across the line across 4 sequential sweeps:
     $$\text{rotationDirections} = \{(-1, 1), (1, -1), (1, -1), (-1, 1)\}$$
  2. For 500 sample iterations per sweep, each sensor's minimum (`Min_ADC[i]`) and maximum (`Max_ADC[i]`) reflectance values are recorded.
  3. The optimal dynamic threshold midpoint is calculated:
     $$\text{Reference\_ADC}[i] = \frac{\text{Max\_ADC}[i] + \text{Min\_ADC}[i]}{2.0}$$
  4. Values are scaled by $/ 4$ (mapping 0–1023 into 0–255) and written to EEPROM:
     - Addresses `0` to `7`: Reference thresholds
     - Addresses `8` to `15`: Maximum values
     - Addresses `16` to `23`: Minimum values
  5. `LoadCalibration()` runs automatically inside `setup()`, ensuring calibration survives power cycles without requiring recalibration.

---

### Part-08: Inverse Track Mode (Normal vs. Inverted)
- **Source Code**: [`Part-08 Files/8IR_MUX_PID_Part-08/`](file:///C:/Users/kenne/Desktop/lfr/Part-08%20Files/8IR_MUX_PID_Part-08)
- **Objective**: Provide dual-track capability: black line on white surface OR white line on black surface (inverse track).
- **Implementation**:
  - Introduced boolean flag `inverseON`:
    ```cpp
    if (sensorADC[i] > Reference_ADC[i]) {
      sensorDigital[i] = (inverseON == 0) ? 1 : 0;
    } else {
      sensorDigital[i] = (inverseON == 0) ? 0 : 1;
    }
    ```
  - Standardizes the downstream PID math so line position calculation remains completely identical regardless of track color polarity.

---

### Part-09: On-Board OLED Interactive Menu System
- **Source Code**: [`Part-09 Files/8IR_MUX_PID_Part-09/`](file:///C:/Users/kenne/Desktop/lfr/Part-09%20Files/8IR_MUX_PID_Part-09)
- **Objective**: Provide a graphical user interface (GUI) on an SSD1306 128x64 OLED display using `U8glib`, navigated with 3 tactile push buttons.
- **Menu Structure**:
  ```
  +-------------------------------+
  |       ROBOTECH INNOVATOR      |
  +-------------------------------+
  |  [ ] Line Follow              |
  |  [>] Calibration              |
  |  [ ] Analog Value             |
  |  [ ] Digital Value            |
  |  [ ] Motor Test               |
  +-------------------------------+
  ```
- **Operational Features**:
  - `BUTTON_UP_PIN` / `BUTTON_DOWN_PIN`: Cycle through list items with custom scrollbar indicators.
  - `BUTTON_SELECT_PIN`: Executes selected routine:
    1. **Line Follow**: Enters active high-speed PID tracking loop.
    2. **Calibration**: Executes dynamic sweep auto-calibration and writes to EEPROM.
    3. **Analog Value**: Displays live multi-sensor analog readings for sensor diagnostics.
    4. **Digital Value**: Displays binary bitmask for sensor alignment.
    5. **Motor Test**: Spins motors forward and backward to verify H-bridge wiring.

---

### Part-10: Master Integrated Production Firmware
- **Source Code**: [`Part-10 Files/8IR_MUX_PID_Part-10/`](file:///C:/Users/kenne/Desktop/lfr/Part-10%20Files/8IR_MUX_PID_Part-10)
- **Reference Docs**:
  - [`Circuit-Diagram.jpg`](file:///C:/Users/kenne/Desktop/lfr/Part-10%20Files/Circuit-Diagram.jpg)
  - [`Sensor Connection.jpg`](file:///C:/Users/kenne/Desktop/lfr/Part-10%20Files/Sensor%20Connection.jpg)
  - [`Specification.jpg`](file:///C:/Users/kenne/Desktop/lfr/Part-10%20Files/Specification.jpg)
- **Objective**: Full production-ready firmware consolidating PID control, OLED HUD graphics, button debouncing, EEPROM loading, and high-frequency multiplexing into an optimized system.
- **Key Enhancements**:
  - **Live Sensor Gauge on Home Screen**: Renders visual representation of all 8 sensor states directly on the OLED home screen alongside team branding.
  - **Optimized Master Pinout**: Full remapping of pins to prevent timer PWM and I2C conflict (see [Master Pin Mapping Matrix](#master-pin-mapping--hardware-matrix)).
  - **Production PID Constants**: Preconfigured for high speed: `base_speed = 250`, `kp = 10`, `kd = 800`.

---

### Part-11: Hardware Diagnostics, Calibration & Platform Binaries
- **Directory**: [`Part-11 Files/`](file:///C:/Users/kenne/Desktop/lfr/Part-11%20Files)
- **Key Assets**:
  - [`Steps.pdf`](file:///C:/Users/kenne/Desktop/lfr/Part-11%20Files/Steps.pdf): Comprehensive illustrated visual assembly, soldering, and alignment guide.
  - [`Sensor-Analog-Read.ino`](file:///C:/Users/kenne/Desktop/lfr/Part-11%20Files/Sensor-Analog-Read/Sensor-Analog-Read.ino): Bare-metal 8-channel direct analog stream reader for diagnosing phototransistor balance.
  - [`tcrt-test.ino`](file:///C:/Users/kenne/Desktop/lfr/Part-11%20Files/tcrt-test/tcrt-test.ino): Minimal single-channel TCRT5000 test sketch.
  - [`build/`](file:///C:/Users/kenne/Desktop/lfr/Part-11%20Files/tcrt-test/build): Precompiled production binaries (`.hex`, `.elf`, `.bin`) ready for flashing directly to **Arduino Nano (AVR)** and **ESP32** using tools like `avrdude` or `esptool.py`.

---

## Mathematical Foundations & PID Theory

### 1. Centroid Formulation
When the robot is over a line, multiple phototransistors may partially detect the line. Rather than treating sensor values as discrete points, the line position is computed using center-of-mass weighted averaging:

$$\text{Position} = \frac{\sum_{i=0}^{N-1} (S_i \times W_i)}{\sum_{i=0}^{N-1} S_i}$$

Where:
- $S_i \in \{0, 1\}$ (Digital state of sensor $i$)
- $W_i \in \{10, 20, 30, 40, 50, 60, 70, 80\}$ (Positional weights)
- Center reference $= \frac{10 + 80}{2} = 45$

### 2. Proportional-Derivative (PD) Formulation
In high-speed differential drive robots, the Integral term ($K_i$) is often set to $0$ because steady-state bias is minimal and integral windup causes overshoot on sharp transitions:

$$e(t) = \text{Target} - \text{Position}(t) = 45 - \text{Position}(t)$$

$$\Delta e(t) = e(t) - e(t - \Delta t)$$

$$u(t) = K_p \cdot e(t) + K_d \cdot \Delta e(t)$$

$$\text{PWM}_{\text{Left}} = \text{BaseSpeed} - u(t)$$

$$\text{PWM}_{\text{Right}} = \text{BaseSpeed} + u(t)$$

- **$K_p$ Effect**: Generates a steering torque proportional to distance off-center. If too low, robot cuts corners or wanders off track; if too high, severe side-to-side wobble occurs.
- **$K_d$ Effect**: Measures the rate of approach toward or away from the center. Opposes rapid changes in error, providing critical damping that suppresses oscillation.

---

## Tuning & Calibration Procedures

To achieve maximum stability and track speed:

```
                      +-----------------------------+
                      | Start: BaseSpeed = 100-150  |
                      | Kp = 0, Kd = 0              |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | Increase Kp incrementally   |
                      | until robot follows curve   |
                      | but oscillates steadily     |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | Increase Kd until           |
                      | oscillations are completely |
                      | damped out on straights     |
                      +--------------+--------------+
                                     |
                                     v
                      +-----------------------------+
                      | Increase BaseSpeed by +25   |
                      | Repeat fine tuning          |
                      +-----------------------------+
```

1. **Perform Dynamic Calibration**: Place the robot over the track with the line centered under the array. Select `Calibration` from the OLED menu or execute `calibrateSensor()`. Ensure the robot sweeps both black and white areas.
2. **Verify EEPROM Load**: Check the Serial Monitor (9600 baud) on boot to confirm that `LoadCalibration()` loaded clean thresholds.
3. **Start Low-Speed Tuning**:
   - Set `base_speed = 120`, `kp = 4`, `kd = 30`.
   - Gradually increase `kp` until the robot negotiates moderate curves.
   - Increase `kd` until high-speed oscillations on straight segments vanish.
4. **Push High-Speed Boundary**:
   - Step `base_speed` up to 200–255.
   - Adjust `delay_before_turn` (default: 100 ms) and `turnSpeed` (default: 200) to ensure sharp 90° bends are executed without skipping past the turn.

---

## Troubleshooting Guide

| Issue | Root Cause | Solution |
| :--- | :--- | :--- |
| **Robot oscillates violently on straight lines** | $K_p$ too high or $K_d$ too low | Reduce $K_p$ by 20–30% or increase $K_d$ to add damping. |
| **Robot runs off track on sharp corners** | Base speed too high or $K_p$ too low | Increase $K_p$; verify `turnRight()` and `turnLeft()` bitmask thresholds. |
| **Robot turns in reverse or spins in circles** | Inverted motor polarity or swapped sensor channels | Swap motor terminals or swap left/right motor pins in definitions. |
| **OLED display remains black or freezes** | I2C address conflict or missing pull-up resistors | Verify `0x3C` I2C address in `U8glib`; ensure SDA/SCL pull-up resistors (4.7kΩ) are present. |
| **Calibration produces identical min/max values** | Sensor ground disconnected or MUX select pins misplaced | Test sensors with `Sensor-Analog-Read.ino` to verify analog responsiveness. |
| **Arduino resets when motors accelerate** | Battery voltage sag / lack of decoupling capacitors | Add a 470µF–1000µF electrolytic capacitor across the battery/motor driver power terminals. |

---

## Author & Attribution
- **Original Code & Design**: Aslam Hossain ([RoboTech Innovator](https://github.com/AslamHossain-YT))
- **YouTube Tutorials**: [RoboTech Innovator YouTube Channel](https://www.youtube.com/@RoboTechInnovator)
- **Contact / Kit Inquiries**: [WhatsApp (+8801647467658)](https://wa.me/c/8801647467658)