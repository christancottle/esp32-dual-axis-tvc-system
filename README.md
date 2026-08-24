# ESP32 Dual-Axis Thrust Vector Control (TVC) Flight Computer

An embedded 6-DOF flight computer designed for active thrust vector control, real-time wireless ground station telemetry, and high-rate flash logging. Built around the **ESP32**, **MPU-6050** IMU, and **BMP280** barometric sensor.

![Flight Telemetry Plot](IMG_5555.png)

---

## Key Features

* **Dual-Axis Active Stabilization:** Controls Pitch and Yaw servos using sensor fusion logic bounded within $\pm 30^\circ$ deflection limits.
* **Low-Noise Filter Pipeline:** Employs an Exponential Moving Average (EMA) low-pass filter on orientation angles to eliminate motor servo jitter caused by high-G accelerometer noise.
* **High-Rate Flash Logging:** Writes telemetry data at ~30 Hz directly to onboard **LittleFS** physical flash memory with immediate buffer flushing (`logFile.flush()`) to prevent data loss during power brownouts.
* **5 Hz Wireless Telemetry:** Broadcasts telemetry packets over **ESP-NOW** to a ground telemetry station.
* **Deterministic State Machine:** Robust state engine (`ARMED` $\rightarrow$ `FLIGHT` $\rightarrow$ `DESCENT` $\rightarrow$ `LANDED`) with manual serial debug overrides (`reset` & `dump`).

---

## Hardware Specifications & Pinout

| Component | Interface / Pin | Description |
| :--- | :--- | :--- |
| **Microcontroller** | ESP32-WROOM-32 | Dual-core @ 240 MHz, LittleFS, ESP-NOW Wireless |
| **Pitch Servo** | GPIO 18 (PWM) | Dedicated PWM Timer 0 allocation (50 Hz) |
| **Yaw Servo** | GPIO 19 (PWM) | Dedicated PWM Timer 0 allocation (50 Hz) |
| **IMU (MPU-6050)** | I2C (SDA: 21, SCL: 22) | 6-DOF Accelerometer & Gyroscope ($\pm 8g$ range) |
| **Barometer (BMP280)**| I2C (Address 0x76/0x77) | High-precision altitude tracking with I2C recovery |

---

## Telemetry Log & Flight Performance

Flight performance data captured during test logging (stored in [`telemetry/espnowfile.csv`](espnowfile.csv)):

* **Peak Altitude:** `0.61 m` (Barometric filtered)
* **Peak Acceleration:** `1.40 g`
* **Log Sample Rate:** ~30 Hz (~33ms log interval)
* **Attitude Limits:** Pitch range $[-5.4^\circ, +35.7^\circ]$, Yaw range $[-0.8^\circ, +15.9^\circ]$

---

## Testing & Debugging Strategy

Building a custom avionics stack requires isolated validation of hardware, firmware, and wireless protocols before integration. The system was engineered and verified using a modular, step-by-step testing process:

### 1. Wireless Telemetry & Ground Station Validation
* **Mock Data Injection:** Before binding the actual ESP32 flight computer over ESP-NOW, the ground station display and receiving logic were validated by broadcasting simulated sine waves and randomized telemetry packets from a secondary microelectronics test rig.
* **Link Quality & Range Testing:** Verified packet delivery rates and RSSI stability to ensure zero-loss packet parsing prior to mounting hardware.

### 2. Sensor Calibration & Noise Filtering
* **Static Baseline Bench Calibration:** The BMP280 barometric pressure sensor was calibrated against local ambient pressure across multiple 10-sample rolling averages to establish an accurate zero-altitude reference.
* **Jitter Mitigation (EMA Filtering):** Raw MPU-6050 accelerometer readings exhibited high-frequency noise that translated into servo chatter. An Exponential Moving Average (EMA) low-pass filter ($\alpha = 0.15$) was implemented on the Yaw axis, successfully smoothing actuation without introducing control lag.

### 3. Flash Storage & Crash Resilience
* **LittleFS File Integrity Tests:** Initial testing revealed buffered writes were lost when power dropped during reset. The flight loop was updated to enforce immediate physical commits (`logFile.flush()`) right after writing telemetry rows.
* **Serial Log Dumping:** Integrated non-blocking serial commands (`dump` and `reset`) to easily extract flash logs to `.csv` format and re-zero altitude baselines between bench tests.

### 4. Hardware Safety & Servo Management
* **Shared PWM Timer Allocation:** Consolidated both servo channels onto ESP32 PWM Timer 0 to resolve hardware timer allocation conflicts and prevent signal dropping.
* **State Machine Latches:** Added persistent lock states in `LANDED` mode to keep servos centered and prevent unintentional actuation after flight completion.

---

## Repository Structure

```text
├── firmware/
│   └── TVC_Flight_Computer.ino     # Main ESP32 flight software
├── telemetry/
│   ├── espnowfile.csv              # Raw logged telemetry CSV dataset
│   └── plot_telemetry.py           # Python data analysis & plotting script
├── assets/
│   └── flight_telemetry_clean.png  # Generated telemetry visualization
└── README.md


