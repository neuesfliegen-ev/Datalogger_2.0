# 🛩️ Datalogger_2.0 

Datalogger for collecting, logging and transmitting RC flight data
Continuation of project Datalogger (first version)

---

## 🧩 Planned Improvements

- Remove SIM card and use radio communication for telemetry
- Improve IMU sensor stability on flight
- Develop more efficient calibration process

---

## 📦 Features

- Logs data to SD card in `.csv` format
- Captures:
  - 9-axis IMU data (acceleration, gyroscope, magnetometer)
  - Barometric pressure, altitude, and temperature
  - GPS coordinates, altitude, speed, and satellite count
//to be refined

## 🧠 Core Components

| Component                  | Description |
|---------------------------|-------------|
| **Arduino**               | Primary microcontroller (Nano Ble 33 Sense REV2) |
| **BMI270_BMM150**         | 9-DOF IMU (acceleration, gyroscope, magnetometer) integrated on Arduino |
| **LPS22HB**               | Barometric pressure sensor integrated on Arduino |
| **GEP M10**               | GEP-M10 GPS module utilizing the u-blox M10050 GPS chip |
| **SD Card Module**        | Stores `.csv` data logs |
//more to be added

---

## 🔧 Circuit Diagram
//to be added

---

## 👨‍💻 Credits

- Developed by the **Neues Fliegen e.V.** team
- Based on libraries by Arduino...

---

## 📷 License

- This project is shared publicly for educational and personal use only.
Commercial use or sale is not permitted without explicit permission
from Neues Fliegen e.V.
- see LICENSE file

---
