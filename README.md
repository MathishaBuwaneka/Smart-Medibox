# 💊 Medibox – Smart Medicine Management System

This project presents a smart, IoT-enabled Medibox designed to assist users in managing their medication routines and ensure optimal storage conditions for light- and temperature-sensitive medicines. Developed as part of the **EN2853: Embedded Systems and Applications** course at the **University of Moratuwa**, the Medibox project includes two progressive versions, each enhancing the functionality and user experience.

---

## 📦 Project Versions

### ✅ Medibox v1.0 – Alarm & Health Monitoring System
A simulation-based Medibox built on **ESP32** using the **Wokwi** simulator. This version focuses on time management and environmental monitoring.

#### Features:
- **NTP-based time sync** and **time zone configuration**
- **Dual alarm system** with set, view, delete, and snooze options
- **OLED display** for real-time clock and alarm information
- **Buzzer and OLED messages** to alert during alarms or unsafe conditions
- **Push button interface** to stop or snooze alarms
- **Temperature and humidity monitoring** using a DHT11 sensor
- **Warning indications** for out-of-range temperature or humidity

---

### ✅ Medibox v2.0 – Light-Sensitive Smart Storage System
An upgraded Medibox system featuring **real-time control**, **environment-responsive servo motor** operations, and **MQTT + Node-RED dashboard integration**.

#### Features:
- **LDR sensor** to monitor light intensity with configurable sampling and upload intervals
- **Temperature sensor (DHT11)** to monitor internal temperature
- **Servo motor-controlled shaded sliding window** that auto-adjusts based on light and temperature conditions
- **Real-time control equation**:
`θ = θoffset + (180−θoffset) × I × γ × ln(ts/tu) × (T/Tmed)`

- **Node-RED Dashboard** includes:
- Chart and gauge for visualizing light intensity
- Sliders for adjusting:
  - Sampling interval (ts)
  - Sending interval (tu)
  - Minimum servo angle (θoffset)
  - Control factor (γ)
  - Ideal storage temperature (Tmed)
- **MQTT integration** using `test.mosquitto.org` broker

---

## ⚙️ Technologies Used
- **ESP32** microcontroller
- **Arduino IDE / PlatformIO**
- **Wokwi Simulator** (for v1.0)
- **DHT11** (Temperature & Humidity Sensor)
- **LDR Sensor** (Light Intensity Monitoring)
- **Servo Motor** (Light control mechanism)
- **Node-RED** for dashboard visualization and user interaction
- **MQTT Protocol** for data transfer (`test.mosquitto.org`)
- **JSON** for Node-RED flow sharing

---

## 📁 Repository Structure

`medibox/
├── v1.0/
│ ├── medibox_alarm_system.ino
│ └── README_v1.md
├── v2.0/
│ ├── medibox_smart_storage.ino
│ ├── node_red_flow.json
│ └── README_v2.md
├── media/
│ ├── demo_video.mp4
│ └── screenshots/
└── README.md`


---

## 🚀 Getting Started

### For Medibox v1.0
1. Open `medibox_alarm_system.ino` in **Arduino IDE** or simulate using **Wokwi**.
2. Set time zone and configure alarms using the menu.
3. Test alarm triggering, DHT sensor warnings, and OLED outputs.

### For Medibox v2.0
1. Open `medibox_smart_storage.ino` in **Arduino IDE**.
2. Connect an **ESP32**, **DHT11**, **LDR**, and **servo motor**.
3. Import `node_red_flow.json` into **Node-RED**.
4. Connect to MQTT broker `test.mosquitto.org`.
5. Use the Node-RED dashboard to control parameters and monitor the Medibox in real time.

---

## 🎯 Key Outcomes

- Time-based medicine reminder system with user control
- Real-time monitoring of environmental parameters
- Dynamic hardware control using mathematical logic
- Cloud-ready design with MQTT and Node-RED integration

---
