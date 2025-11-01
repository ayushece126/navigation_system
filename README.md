# 🧠 Smart Obstacle Detection & Navigation System
### Powered by Raspberry Pi 4 + LiDAR + Multi-Sensor Fusion

This project implements a **real-time navigation and obstacle detection system** designed for assistive or autonomous mobility devices.  
It fuses data from **LiDAR, infrared, proximity, and edge sensors** to detect obstacles and provides **audio, haptic, and actuator feedback** to guide safe navigation.

---

## 🎯 Objectives
- Real-time obstacle detection using **TF-Luna LiDAR**
- Short-range edge and proximity detection
- Intelligent **state-machine-based navigation**
- Audio & haptic feedback for user alerts
- Modular codebase for easy integration of cameras and motors

---

## ⚙️ Hardware Components

| Category | Component | Function |
|-----------|------------|-----------|
| Controller | Raspberry Pi 4 Model B | Main processor & I/O hub |
| Distance | Benewake TF-Luna LiDAR | Long-range distance measurement |
| Proximity | VCNL4040 Sensor | Ambient light & short-range proximity |
| Edge | TCRT5000 IR Sensor | Edge / line / drop detection |
| Input | TTP223 Touch Sensor | Start / stop control |
| Feedback | Piezo Buzzer (PWM) | Audio alert |
| Feedback | Haptic ERM Motor (PWM via NPN) | Vibration feedback |
| Actuator | Relay Module | Safety brake / actuator trigger |
| Power | Li-Po Battery (3.7 V 1000 mAh) | Portable power source |
| Interface | Breadboard + Logic Level Shifter | Safe prototyping & voltage translation |

---

## 🪛 Wiring Summary (BCM Numbering)

| Module | VCC | GND | Signal Pins (→ Pi GPIO) | Notes |
|---------|------|------|----------------|--------|
| **TF-Luna LiDAR** | 5 V | GND | TX → GPIO15 (RX) / RX ← GPIO14 (TX via level shifter) | UART @ 115200 baud |
| **VCNL4040** | 3.3 V | GND | SDA → GPIO2 / SCL → GPIO3 | I²C bus 1 |
| **TCRT5000** | 5 V | GND | OUT → GPIO17 (via shifter) | Digital edge signal |
| **TTP223 Touch** | 3.3 V | GND | OUT → GPIO22 | Start/stop input |
| **Buzzer** | 3.3 V | GND | SIG → GPIO18 | PWM tone output |
| **Haptic Motor** | 3.3 V | GND | PWM → GPIO23 | Drives NPN → ERM motor |
| **Relay / Actuator** | 5 V | GND | SIG → GPIO24 | Active HIGH relay control |

> ⚠️ **All grounds must be common.**  
> Use a **logic-level converter** between any 5 V outputs and Pi inputs.

---

## 🧩 Repository Structure

navigation_system/
│
├── tests/
│ ├── test_tfluna.py
│ ├── test_vcnl4040.py
│ ├── test_tcrt5000.py
│ └── test_touch_buzzer.py
│
├── nav_fsm_verified.py # Main integrated controller (FSM + haptics + actuator)
├── wiring_diagram.png # (optional schematic)
├── README.md # You are here
└── requirements.txt



---

## 🧠 Software Overview

### 🏗️ Finite State Machine (FSM)

| State | Meaning | Action |
|--------|----------|---------|
| **IDLE** | System inactive | All outputs off |
| **SCAN** | Monitoring environment | Sensors running |
| **CLEAR_PATH** | Safe to move | Short beeps + light vibration |
| **CAUTION** | Nearby obstacle | Medium beeps + gentle vibration |
| **AVOID** | Close obstacle | Long beep + strong vibration + relay ON |
| **EDGE_ALERT** | Edge/drop detected | Continuous warning + relay ON |
| **SENSOR_FAULT** | Timeout/no data | Error beep + actuator ON |

---

## 🧪 Setup & Run

### 1️⃣ Enable Interfaces

sudo raspi-config
# Interfacing Options → enable I2C, SPI, and Serial (“no login shell”, “enable hardware”)
sudo reboot
sudo apt update
---

## 2️⃣ Install Dependencies
sudo apt install -y python3-pip python3-rpi.gpio i2c-tools
pip3 install -r requirements.txt

3️⃣ Verify Sensors

Run the individual tests in the tests/ folder to confirm wiring.

4️⃣ Run the Main System
python3 nav_fsm_verified.py

Touch pad → toggle start/stop.
Watch the console for [STATE] -> … and listen/feel feedback.
