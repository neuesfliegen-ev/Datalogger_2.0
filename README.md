# 🛩️ Datalogger_2.0

Datalogger for collecting, logging and transmitting RC flight data during the New Flying Competition
organised and held by the Neues Fliegen e.V.
Continuation of project Datalogger (first version).

---

## 📚 Documentation

| Document | Contents |
|---|---|
| [Architecture](03_Docs/architecture.md) | Startup sequence, FreeRTOS tasks, data paths, main components |
| [Hardware](03_Docs/hardware.md) | ESP32 pin map, buses, wiring notes |
| [Radio commands](03_Docs/commands.md) | Command protocol, command list, telemetry output format |
| [Branch convention](03_Docs/branch-convention.md) | Branch naming and PR workflow |
| [GitHub best practices](03_Docs/github-best-practices.md) | Commit, push and branching habits |

Firmware build instructions: [01_Datalogger_firmware/README.md](01_Datalogger_firmware/README.md)

---

## 🗂️ Repository Layout

```text
00_Datasheet/            Datasheets for ESP32, WT901B, PX4 airspeed, E22 LoRa, battery
01_Datalogger_firmware/  ESP-IDF firmware (entry point: main/datalogger.cpp)
└── hardware_test/       Standalone tests for IMU, GPS and SD card
02_PCB/Datalogger-v2/    KiCad schematic and PCB
03_Docs/                 Project documentation
```

---

## 🧩 Improvements over v1

- Remove SIM card and use radio communication for telemetry
- Improve IMU sensor stability on flight
- Develop more efficient calibration process
- Develop a ground station capable of receiving and uploading live telemetry
- Implement FreeRTOS on ESP32

---

## 📦 Features

- ESP-IDF firmware with two FreeRTOS tasks: sensor polling (core 0) and SD logging (core 1)
- Logs packed binary datasets to the SD card (`/sdcard/flights/<n>.bin`)
- Captures:
  - 9-axis IMU data (acceleration, gyroscope, magnetometer), roll/pitch/yaw
  - Barometric pressure, altitude and temperature
  - GPS data
  - Airspeed
- Streams telemetry to the ground station over LoRa radio
- Accepts text commands over the radio link (start/stop logging, telemetry, IMU calibration) — see [commands](03_Docs/commands.md)
- Ground station streams telemetry to web server *(planned)*

---

## 🧠 Core Components

| Component | Description | Interface |
|---|---|---|
| **ESP32** | Primary microcontroller | — |
| **WT-901B** | 10-DOF IMU (acceleration, gyroscope, magnetometer + barometer) | I2C1, `0x50` |
| **GEP M10** | GPS module using the u-blox M10050 chip | UART1, 115200 baud |
| **PX4 digital airspeed sensor** | Pitot-based airspeed measurement | I2C0, `0x28` |
| **E22-900T30D LoRa module** | Live telemetry and commands to/from the ground station | UART2, 9600 baud |
| **SD card module** | Stores flight logs | SPI3 |

Full pin assignments: [03_Docs/hardware.md](03_Docs/hardware.md)

---

## 🔧 Circuit Diagram

See the KiCad project in [02_PCB/Datalogger-v2](02_PCB/Datalogger-v2/).

---

## 👨‍💻 Credits

- Developed by the **Neues Fliegen e.V.** team

---

## 📷 License

- This project is shared publicly for educational and personal use only.
  Commercial use or sale is not permitted without explicit permission
  from Neues Fliegen e.V.
- Licensed under CC BY-NC 4.0 — see [LICENSE](LICENSE)
