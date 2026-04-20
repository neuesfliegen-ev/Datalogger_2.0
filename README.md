# 🛩️ Datalogger_2.0 

Datalogger for collecting, logging and transmitting RC flight data during the New Flying Competition
organised and held by the Neues Fliegen e.V.
Continuation of project Datalogger (first version)

---

## 🧩 Improvements

- Remove SIM card and use radio communication for telemetry
- Improve IMU sensor stability on flight
- Develop more efficient calibration process
- Develop a ground station capable of receiving and uploading live telemetry
- Implement FreeRTOS on ESP32

---

## 📦 Features

- Logs data to SD card in `.csv` format
- Captures:
  - 9-axis IMU data (acceleration, gyroscope, magnetometer)
  - Barometric pressure, altitude, and temperature
  - GPS coordinates, altitude, speed, and satellite count
- Streams telemetry to ground station through radio communication
- Ground station streams telemetry to web server
//to be refined

## 🧠 Core Components

| Component                  	| Description |
|-------------------------------|-------------|
| **ESP32**               		| Primary microcontroller |
| **WT-901B**         			  | 10-DOF IMU (acceleration, gyroscope, magnetometer + barometer)  |
| **GEP M10**               	| GEP-M10 GPS module utilizing the u-blox M10050 GPS chip |
| **Digital pitot sensor**    | PixHawk PX4 digital airspeed sensor |
| **E22-900T30D LoRa module**	| Enables the transmission of live telemetry via radio communication to ground station|
| **SD Card Module**        	| Stores `.csv` data logs |
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
