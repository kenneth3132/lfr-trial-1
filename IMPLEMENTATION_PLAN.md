# Line Follower Robot (LFR) — Engineering Implementation Plan

[![Status](https://img.shields.io/badge/Status-Ready%20for%20Execution-brightgreen.svg)]()
[![Type](https://img.shields.io/badge/Document-Implementation%20Plan-blue.svg)]()
[![Architecture](https://img.shields.io/badge/Architecture-PID%20Differential%20Drive-orange.svg)]()

This document outlines the structured, phase-by-phase engineering implementation roadmap for constructing, programming, calibrating, and optimizing the **Fast PID Line Follower Robot**.

---

## 1. Project Overview & Objectives

### 1.1 Goal
Construct a competition-grade Line Follower Robot capable of traversing complex black-on-white or white-on-black tracks at high velocity ($\ge 1.0\text{ m/s}$), negotiating 90° sharp turns, acute angles, and intersections through closed-loop PID control and autonomous dynamic sensor calibration.

### 1.2 Target Specifications
- **Microcontroller**: Arduino Nano (ATmega328P, 16MHz) or ESP32
- **Sensor Array**: 8-Channel Infrared Reflectance Array (TCRT5000 / QTR-8A)
- **Multiplexing**: CD74HC4067 (16-Ch) or 74HC4051 (8-Ch) analog multiplexer
- **Motor Control**: TB6612FNG Dual MOSFET H-Bridge (or L298N)
- **Actuators**: 2x Micro Metal Gearmotors (N20, 6V, 500–1000 RPM)
- **User Interface**: 0.96" I2C OLED (SSD1306, 128x64) + 3 Tactile Push Buttons
- **Non-Volatile Storage**: On-board EEPROM for calibration persistence
- **Power**: 7.4V (2S) or 11.1V (3S) LiPo battery with regulated 5V logic bus

---

## 2. Phased Implementation Roadmap

```
+-------------------------------------------------------------------------------+
|                         IMPLEMENTATION TIMELINE                               |
|                                                                               |
|  [Phase 1] Component Acquisition & Bench Testing                              |
|       |                                                                       |
|       v                                                                       |
|  [Phase 2] Chassis Assembly & Power Distribution Hardware                     |
|       |                                                                       |
|       v                                                                       |
|  [Phase 3] Sensor Array & Multiplexer Integration (Parts 01 & 02)             |
|       |                                                                       |
|       v                                                                       |
|  [Phase 4] Dual Motor Driver Kinematics & PWM Testing (Part 03)               |
|       |                                                                       |
|       v                                                                       |
|  [Phase 5] PID Closed-Loop Algorithm Tuning (Part 04)                         |
|       |                                                                       |
|       v                                                                       |
|  [Phase 6] 90-Degree Turn & Intersection State Machine (Parts 05 & 06)        |
|       |                                                                       |
|       v                                                                       |
|  [Phase 7] Dynamic Auto-Calibration & EEPROM Persistence (Parts 07 & 08)      |
|       |                                                                       |
|       v                                                                       |
|  [Phase 8] OLED Display Menu UI & Final Integration (Parts 09 & 10)           |
|       |                                                                       |
|       v                                                                       |
|  [Phase 9] High-Speed Track Testing & Final Acceptance Validation             |
+-------------------------------------------------------------------------------+
```

---

## Phase 1: Component Acquisition & Bench Verification

### Objectives:
Verify the electrical integrity of all sensors, drivers, and microcontrollers prior to physical chassis assembly.

### Steps:
1. **Bill of Materials Audit**:
   - Inspect Arduino Nano, TB6612FNG / L298N, 8-IR array, CD74HC4067 MUX, 0.96" I2C OLED, N20 motors, and power regulators against the BOM in [`README.md`](file:///C:/Users/kenne/Desktop/lfr/README.md).
2. **Individual Sensor Bench Test**:
   - Flash [`Part-11 Files/tcrt-test/tcrt-test.ino`](file:///C:/Users/kenne/Desktop/lfr/Part-11%20Files/tcrt-test/tcrt-test.ino) onto the Arduino Nano.
   - Connect a single TCRT5000 IR sensor to pin `A5`.
   - Open Serial Monitor (9600 baud) and verify that analog readings change distinctly when moving over white paper vs. black electrical tape (e.g., $<200$ on white vs. $>800$ on black).
3. **Multi-Channel Raw ADC Test**:
   - Flash [`Part-11 Files/Sensor-Analog-Read/Sensor-Analog-Read.ino`](file:///C:/Users/kenne/Desktop/lfr/Part-11%20Files/Sensor-Analog-Read/Sensor-Analog-Read.ino).
   - Verify analog read stability across all 8 pins (`A0`–`A7`).
4. **Driver Circuit Simulation**:
   - Open [`Part-03 Files/Proteus Simulation/motor_simulation_proteus.pdsprj`](file:///C:/Users/kenne/Desktop/lfr/Part-03%20Files/Proteus%20Simulation/motor_simulation_proteus.pdsprj) in Proteus to simulate H-bridge motor switching logic under simulated load.

### Acceptance Criteria:
- All 8 phototransistors respond with at least a 400 ADC unit delta between black and white surfaces.
- Microcontroller boots cleanly over USB and serial communication is established.

---

## Phase 2: Chassis & Power Distribution Assembly

### Objectives:
Construct the physical mechanical platform, mount actuators and sensors, and build a noise-isolated power supply.

### Steps:
1. **Mechanical Mounting**:
   - Secure the two N20 gearmotors into bracket mounts ensuring wheel axles are collinear and symmetrical.
   - Install a smooth caster ball or low-friction nylon skid pad at the rear to balance the robot.
   - Mount the 8-IR sensor array at the front of the robot.
   - **Crucial Dimension**: Maintain a ground clearance of **3mm to 5mm** between the TCRT5000 sensor faces and the ground. Clearances outside this envelope degrade optical reflectance sensitivity.
2. **Power Rail Fabrication**:
   - Connect the LiPo battery (7.4V / 11.1V) through a physical toggle switch and in-line 2A fuse.
   - Feed raw battery voltage directly to the motor driver `VM` terminal.
   - Route battery voltage through a 5V buck converter (or LM7805) to supply the Arduino `5V` pin, MUX `VCC`, sensor array `VCC`, and OLED `VCC`.
3. **Decoupling & Ground Integrity**:
   - Solder a $470\mu\text{F}$ to $1000\mu\text{F}$ electrolytic capacitor directly across the motor driver `VM` and `GND` pins to absorb motor inductance spikes.
   - Tie logic ground and power ground together at a single common star-ground junction point.

### Acceptance Criteria:
- Wheels spin smoothly without mechanical wobble or chassis misalignment.
- Regulated 5V logic rail remains stable ($5.0\text{V} \pm 0.1\text{V}$) during simultaneous motor starts.

---

## Phase 3: Sensor Array & Multiplexer Integration

### Target Firmware:
- [`Part-02 Files/8IR_sensor_reading/8IR_sensor_reading.ino`](file:///C:/Users/kenne/Desktop/lfr/Part-02%20Files/8IR_sensor_reading/8IR_sensor_reading.ino)

### Steps:
1. **Multiplexer Wiring**:
   - Connect MUX select pins: `S0` $\rightarrow$ `D11`, `S1` $\rightarrow$ `D10`, `S2` $\rightarrow$ `D8`, `S3` $\rightarrow$ `D7`.
   - Connect MUX common analog output `SIG` $\rightarrow$ `A3` (or `A7` in later revisions).
   - Wire MUX `EN` (Enable) pin to `GND` to enable continuous routing.
2. **Channel Mapping Validation**:
   - Connect sensor outputs 0 through 7 to MUX input channels `C0` through `C7` in direct spatial sequence (left to right).
3. **Code Deployment & Serial Calibration**:
   - Upload `8IR_sensor_reading.ino`.
   - Press Push Button 1 (`A0`).
   - Observe the live 8-bit binary stream (`bitSensor`) on Serial Monitor.
   - Sweep a black strip across the sensors from left to right; verify the active high bit shifts smoothly from bit 0 (`00000001`) to bit 7 (`10000000`).

### Acceptance Criteria:
- Correct sequential channel switching confirmed without cross-talk or adjacent channel leakage.
- `bitSensor` accurately reflects track position with $<1\text{ ms}$ latency per 8-channel sweep.

---

## Phase 4: Motor Driver & Kinematics Control

### Target Firmware:
- [`Part-03 Files/8IR_MUX_Motor/8IR_MUX_Motor.ino`](file:///C:/Users/kenne/Desktop/lfr/Part-03%20Files/8IR_MUX_Motor/8IR_MUX_Motor.ino)

### Steps:
1. **Motor H-Bridge Wiring**:
   - Connect Left Motor control lines: `BIN1` $\rightarrow$ `D5`, `BIN2` $\rightarrow$ `D6`, `PWMB` $\rightarrow$ `D9`.
   - Connect Right Motor control lines: `AIN2` $\rightarrow$ `D2`, `AIN1` $\rightarrow$ `D4`, `PWMA` $\rightarrow$ `D3`.
   - If using TB6612FNG, pull the `STBY` (Standby) pin to `5V` (HIGH).
2. **Direction & Polarity Test**:
   - Upload `8IR_MUX_Motor.ino`.
   - Trigger Button 2 (`D12`) to run `motor(255, 255)`.
   - Verify that both wheels rotate in the **forward** direction.
   - If a wheel rotates backward, swap the corresponding motor lead wires or swap the pin definitions in the code.
3. **PWM Linearity Check**:
   - Test differential spin by calling `motor(150, -150)` and `motor(-150, 150)` to ensure smooth in-place pivoting.

### Acceptance Criteria:
- Forward, reverse, and differential spin commands execute without motor stall or jitter.
- PWM frequency operates without audible high-pitch coil whine.

---

## Phase 5: Closed-Loop PD/PID Controller Implementation

### Target Firmware:
- [`Part-04 Files/8IR_MUX_PID_Algorithm/8IR_MUX_PID_Algorithm.ino`](file:///C:/Users/kenne/Desktop/lfr/Part-04%20Files/8IR_MUX_PID_Algorithm/8IR_MUX_PID_Algorithm.ino)

### Steps:
1. **Weighted Centroid Position Verification**:
   - Place robot on line; verify `center_position = 45`.
   - Verify that when the line is centered, `error = 0`.
2. **PID Tuning Protocol**:
   - **Step 5.1**: Start with conservative parameters:
     - `base_speed = 120`, `kp = 4`, `kd = 0`.
   - **Step 5.2**: Place robot on a track with mild curves. Gradually increase `kp` in steps of $+1$ until the robot follows curves, even if it displays noticeable lateral oscillation.
   - **Step 5.3**: Introduce the derivative term `kd`. Increase `kd` in steps of $+20$ until high-frequency oscillations on straight tracks are completely damped out.
   - **Step 5.4**: Gradually increase `base_speed` (e.g., $150 \rightarrow 180 \rightarrow 220 \rightarrow 250$), re-balancing $K_p$ and $K_d$ at each speed increment.

### Acceptance Criteria:
- Robot tracks straight lines and gradual curves smoothly at high speed without veering off the line.

---

## Phase 6: 90° Turn, Intersection & Line-Loss Handling

### Target Firmware:
- [`Part-05 Files/8IR_MUX_PID_Turns/`](file:///C:/Users/kenne/Desktop/lfr/Part-05%20Files/8IR_MUX_PID_Turns)
- [`Part-06 Files/8IR_MUX_PID_Part-06/`](file:///C:/Users/kenne/Desktop/lfr/Part-06%20Files/8IR_MUX_PID_Part-06)

### Steps:
1. **Turn Pattern Recognition**:
   - Configure bitmask classification in `PID_Controller()`:
     - Left turns: `0b11111000`, `0b11110000`, `0b11111100` $\rightarrow$ `direction = "left"`.
     - Right turns: `0b00011111`, `0b00001111`, `0b00111111` $\rightarrow$ `direction = "right"`.
2. **White Space In-Place Spin Execution**:
   - When all sensors report white (`bitSensor == 0`), check the last registered direction.
   - Execute in-place pivot (`turnLeft` / `turnRight`) with `turnSpeed = 150–200`.
   - The loop continues rotating until center sensor `sensorDigital[4] == 1` signals that the path has been reacquired.
3. **Turn Delay Tuning**:
   - Adjust `delay_before_turn` (default: 100 ms) to align the robot's turning axle with the corner vertex before initiating the pivot.

### Acceptance Criteria:
- Robot successfully navigates 90° right-angle bends in both directions without losing track lock.
- Cross-intersections (`0b11111111`) are crossed straight through without accidental turning.

---

## Phase 7: Dynamic Auto-Calibration & EEPROM Persistence

### Target Firmware:
- [`Part-07 Files/8IR_MUX_PID_Part-07/`](file:///C:/Users/kenne/Desktop/lfr/Part-07%20Files/8IR_MUX_PID_Part-07)
- [`Part-08 Files/8IR_MUX_PID_Part-08/`](file:///C:/Users/kenne/Desktop/lfr/Part-08%20Files/8IR_MUX_PID_Part-08)

### Steps:
1. **Autonomous Calibration Sweep**:
   - Implement `calibrateSensor()` routine: robot rotates left and right across the line in 4 sweeps.
   - Record maximum (`Max_ADC[i]`) and minimum (`Min_ADC[i]`) ADC values for each of the 8 sensors across 500 samples per sweep.
2. **Threshold Calculation & EEPROM Storage**:
   - Calculate dynamic threshold: $\text{Reference\_ADC}[i] = (\text{Max\_ADC}[i] + \text{Min\_ADC}[i]) / 2$.
   - Scale values by $/4$ to store in 8-bit non-volatile EEPROM cells (addresses 0 to 23).
3. **Boot-Time Verification**:
   - Verify that `LoadCalibration()` runs in `setup()` and restores stored values on power cycle.
4. **Inverse Track Validation**:
   - Test `inverseON = 1` mode on a white-line-on-black surface; verify identical tracking accuracy.

### Acceptance Criteria:
- Robot successfully self-calibrates without a computer or manual potentiometer tuning.
- Calibrated thresholds persist across power cycles and hard resets.

---

## Phase 8: OLED Menu UI & Master System Integration

### Target Firmware:
- [`Part-09 Files/8IR_MUX_PID_Part-09/`](file:///C:/Users/kenne/Desktop/lfr/Part-09%20Files/8IR_MUX_PID_Part-09)
- [`Part-10 Files/8IR_MUX_PID_Part-10/`](file:///C:/Users/kenne/Desktop/lfr/Part-10%20Files/8IR_MUX_PID_Part-10)

### Steps:
1. **OLED & Button Wiring**:
   - Connect SSD1306 OLED via I2C: `SDA` $\rightarrow$ `A4`, `SCL` $\rightarrow$ `A5`.
   - Connect 3 control buttons: `BUTTON_UP_PIN` $\rightarrow$ `D7`, `BUTTON_SELECT_PIN` $\rightarrow$ `A2`, `BUTTON_DOWN_PIN` $\rightarrow$ `A3` (all configured with `INPUT_PULLUP`).
   - Reassign motor and MUX pins according to the Master Pinout in [`README.md`](file:///C:/Users/kenne/Desktop/lfr/README.md#master-pin-mapping--hardware-matrix).
2. **Library Setup**:
   - Ensure the `U8glib` library is installed in the Arduino IDE environment.
3. **Menu Flow Validation**:
   - Test Up/Down button navigation through the 5 menu screens:
     - `Line Follow`
     - `Calibration`
     - `Analog Value`
     - `Digital Value`
     - `Motor Test`
   - Test the Select button to trigger each mode.
4. **Live Sensor Gauge on Home Screen**:
   - Verify the OLED home screen displays the team banner and live 8-sensor graphical state.

### Acceptance Criteria:
- Standalone operation achieved: robot can be calibrated, tested, and launched entirely via the OLED menu without any PC or USB tether.

---

## Phase 9: High-Speed Track Testing & Final Acceptance Validation

### Steps:
1. **Track Preparation**:
   - Prepare a competition-grade test track containing:
     - Straight segments ($\ge 2\text{ meters}$)
     - Gentle S-curves
     - Sharp 90° left and right turns
     - Acute hairpin turns ($< 90^\circ$)
     - Cross-intersections and T-junctions
2. **High-Speed Boundary Exploration**:
   - Benchmark lap times starting at `base_speed = 180`, progressing up to `250`.
   - Refine $K_p$, $K_d$, and `delay_before_turn` values at maximum speed.
3. **Safety & Battery Health Checks**:
   - Ensure battery voltage does not drop below $3.5\text{V}$ per cell under load.
   - Ensure motor driver heatsink temperature remains below $65^\circ\text{C}$ during continuous 5-minute runs.

---

## 3. Risk Management & Troubleshooting Matrix

| Risk / Failure Mode | Probability | Impact | Mitigation Strategy |
| :--- | :--- | :--- | :--- |
| **Microcontroller brownout on motor burst** | High | Severe | Add $1000\mu\text{F}$ capacitor across motor driver power; separate motor battery feed from logic rail. |
| **Ambient light interference** | Medium | Moderate | Use dynamic auto-calibration (Part 07); add 3D-printed shroud to shield IR sensor array from overhead light. |
| **Mechanical wheel slip on sharp corners** | Medium | Moderate | Use high-friction silicone tires; reduce `delay_before_turn`; tune $K_d$ to avoid jerky steering. |
| **Multiplexer channel switching noise** | Low | Moderate | Ensure `delayMicroseconds(20)` settling delay is maintained after each channel selection before `analogRead()`. |
| **OLED I2C bus hang** | Low | Severe | Add 4.7kΩ external pull-up resistors on SDA/SCL lines; initialize U8glib in non-blocking mode. |

---

## 4. Verification & Sign-off Checklist

- [ ] Bench test completed: all 8 sensors verified with `Sensor-Analog-Read.ino`
- [ ] Motor H-bridge polarity and PWM range verified with `motor.ino`
- [ ] Sensor clearance set to 3mm–5mm above surface
- [ ] Dynamic auto-calibration verified and stored in EEPROM
- [ ] PID parameters tuned on curved test track
- [ ] 90° right-angle and acute turn routines verified
- [ ] OLED menu, buttons, and live sensor HUD verified
- [ ] Continuous 5-minute high-speed competition run completed without track loss
