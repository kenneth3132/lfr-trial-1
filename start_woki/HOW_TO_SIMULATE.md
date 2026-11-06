# How to Simulate the Fast PID Line Follower Robot on Wokwi

[![Platform](https://img.shields.io/badge/Simulator-Wokwi%20Online%20%7C%20VS%20Code-brightgreen.svg)](https://wokwi.com)
[![MCU](https://img.shields.io/badge/Microcontroller-Arduino%20Nano-blue.svg)]()
[![Telemetry](https://img.shields.io/badge/Telemetry-115200%20Baud%20HUD-orange.svg)]()

This step-by-step guide explains how to simulate the complete **No-MUX PID Line Follower Robot** directly in your browser using [Wokwi](https://wokwi.com) without installing any local software or connecting physical hardware.

---

## 1. Simulation Files Overview

The `start_woki` directory contains everything required by the Wokwi simulation engine:

| File | Purpose |
| :--- | :--- |
| [`sketch.ino`](file:///C:/Users/kenne/Desktop/lfr/start_woki/sketch.ino) | Consolidated Arduino Nano firmware containing sensor reading, PID control, turn logic, and telemetry. |
| [`diagram.json`](file:///C:/Users/kenne/Desktop/lfr/start_woki/diagram.json) | Complete graphical schematic containing the Arduino Nano, 8 reflectance potentiometers, push buttons, and motor LEDs. |
| [`connections_and_requirements.txt`](file:///C:/Users/kenne/Desktop/lfr/start_woki/connections_and_requirements.txt) | Master hardware pin mapping and component specification table. |

---

## 2. Launching the Simulation (Two Methods)

### Method A: Free Browser-Based Simulation (Recommended — Zero Install)

1. Open your web browser and navigate to: **[https://wokwi.com/projects/new/arduino-nano](https://wokwi.com/projects/new/arduino-nano)**
2. In the code editor on the left:
   - Select all text in `sketch.ino` and replace it with the entire contents of [`start_woki/sketch.ino`](file:///C:/Users/kenne/Desktop/lfr/start_woki/sketch.ino).
3. In the graphical canvas on the right:
   - Click on the **`diagram.json`** tab (or press the `{}` icon).
   - Select all text and replace it with the entire contents of [`start_woki/diagram.json`](file:///C:/Users/kenne/Desktop/lfr/start_woki/diagram.json).
4. Click the green **Play (Start Simulation)** button in the top navigation bar.
5. Open the **Serial Monitor** at the bottom of the screen (configured to **115200 baud**).

---

### Method B: VS Code Desktop Simulation

1. Install the **Wokwi Simulator** extension in VS Code.
2. In VS Code, open the folder: `C:\Users\kenne\Desktop\lfr\start_woki`.
3. Open `diagram.json`.
4. Press `Ctrl + Shift + P` (or `F1`), type **`Wokwi: Start Simulator`**, and press `Enter`.
5. The simulation will boot interactively inside VS Code.

---

## 3. Interactive Component Guide

```
  [S0]   [S1]   [S2]   [S3]   [S4]   [S5]   [S6]   [S7]  <-- 8 Potentiometers (A0-A7)
  (FarL)               (Center)               (FarR)

           +-------------------------+
 [LEDs]    |      ARDUINO NANO       |    [LEDs]
 L_FWD (D2)|                         |    R_FWD (D7)
 L_REV (D4)|                         |    R_REV (D8)
 L_PWM (D5)|                         |    R_PWM (D6)
           +-------------------------+

           [START D11]   [CALIB D12]   [LED_STATUS D13]
```

### Visual & Interactive Controls:
- **8 Potentiometers (`S0` to `S7`)**: Represent the 8 physical TCRT5000 IR reflectance sensors connected to pins `A0` through `A7`:
  - **Dial Turned Fully Counter-Clockwise (0V / ADC < 500)**: Simulates **White Background** (reflective).
  - **Dial Turned Clockwise (5V / ADC > 500)**: Simulates **Black Line** (absorptive).
- **Green Button (`btn_start` on Pin D11)**: Starts the continuous closed-loop PID Line Following loop.
- **Blue Button (`btn_calib` on Pin D12)**: Triggers the 4-sweep autonomous self-calibration routine and writes to simulated EEPROM.
- **Motor Status LEDs**:
  - `Left Motor FWD (D2)` [Green]: Lights up when left motor drives forward.
  - `Left Motor REV (D4)` [Red]: Lights up when left motor engages reverse braking.
  - `Left PWM Speed (D5)` [Yellow]: Modulates brightness proportional to left wheel speed.
  - `Right Motor FWD (D7)` [Green]: Lights up when right motor drives forward.
  - `Right Motor REV (D8)` [Red]: Lights up when right motor engages reverse braking.
  - `Right PWM Speed (D6)` [Yellow]: Modulates brightness proportional to right wheel speed.
- **Blue Indicator LED (`led_status` on Pin D13)**: Illuminates during calibration sweeps and sharp 90° turn pivots.

---

## 4. Step-by-Step Test Scenarios

### Scenario 1: Live Sensor Telemetry (Idle Mode)
1. Start the simulation. Do not press any buttons yet.
2. Observe the Serial Monitor: every 500ms, the system reports the live 8-bit binary bitmask:
   ```
   [IDLE SENSOR MONITOR] Bitmask: 00011000 | Active: 2 | Pos: 45.0 (Press Button 1 on D11 to Run)
   ```
3. Rotate dial **`S3`** and **`S4`** (center sensors) to max; observe `Bitmask: 00011000` and `Pos: 45.0`.
4. Rotate dial **`S0`** (far left) to max; observe `Bitmask: 10000000` and `Pos: 10.0`.

---

### Scenario 2: Dynamic Auto-Calibration Test
1. Press the **Blue Button (CALIBRATE on D12)**.
2. The blue status LED on `D13` will light up.
3. The robot executes 4 sweeps, tracking minimum and maximum reflectance for each sensor.
4. Watch the Serial Monitor report the calibrated midpoint thresholds and confirm write to non-volatile EEPROM:
   ```
   Sensor 0 -> Min: 0 | Ref: 511 | Max: 1023
   Sensor 1 -> Min: 0 | Ref: 511 | Max: 1023
   ...
   >>> [ACTION] Calibration Complete! Stored in EEPROM.
   ```

---

### Scenario 3: PID Closed-Loop Line Following
1. Set potentiometers **`S3`** and **`S4`** to around **70%** (Black Line Centered), and leave all other pots at 0% (White Background).
2. Click the **Green Button (START PID on D11)**.
3. Observe the Serial Monitor streaming real-time PID telemetry:
   ```
   BIT: 00011000 | Pos: 45.0 | Err: 0.0 | LPWM: 200 | RPWM: 200 | Dir: straight
   ```
   - Notice both left (`D5`) and right (`D6`) yellow PWM LEDs light up with equal intensity.
   - Both green forward LEDs (`D2` and `D7`) are illuminated.

---

### Scenario 4: Simulating a Right-Hand Curve
1. While PID is running, turn dial **`S0`** (far left) to **80%** and turn **`S3` / `S4`** down to 0%.
2. This simulates the line drifting to the far left of the robot:
   ```
   BIT: 10000000 | Pos: 10.0 | Err: 35.0 | LPWM: -80 | RPWM: 255 | Dir: left
   ```
3. **Observe the Motor LEDs**:
   - The right motor speeds up (`RPWM = 255`, yellow LED at max brightness, green FWD on).
   - The left motor reverses (`LPWM = -80`, red REV LED on `D4` illuminates).
   - The robot initiates an aggressive differential leftward turn to steer back to center!

---

### Scenario 5: Simulating a 90° Right Angle Turn
1. Turn dials **`S4`**, **`S5`**, **`S6`**, and **`S7`** to high (`0b00001111` right-biased pattern).
2. The controller registers: `Dir: right`.
3. Now quickly turn all dials to 0% (simulating track departure at the corner vertex).
4. The controller detects `bitSensor == 0` with historical direction `"right"`:
   ```
   [TURN EVENT] Executing 90° Pivot RIGHT...
   ```
5. The blue LED on `D13` lights up, left motor drives forward (`+160`), right motor drives reverse (`-160`), pivoting the robot until dial `S3` or `S4` is rotated back up (simulating line recapture).

---

## 5. Summary of Simulation Hotkeys & Tips

- **Pause/Resume**: Click the Pause icon at the top of Wokwi.
- **Restart Simulation**: Click the Restart circular arrow icon to reset the MCU and test EEPROM reload on boot.
- **Serial Plotter**: You can open the Wokwi Serial Plotter by clicking the graph icon next to the serial monitor to visualize the PID error curve graphically!
