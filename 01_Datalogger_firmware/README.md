# Datalogger Firmware

ESP-IDF firmware for an ESP32-based flight data logger. It reads an IMU, GPS, and airspeed sensor, stores binary samples on an SD card, and exchanges commands and telemetry over a radio UART.

## Start here

1. Read [`docs/architecture.md`](docs/architecture.md) for the runtime flow.
2. Read [`docs/commands.md`](docs/commands.md) before using the radio link.
3. Check [`docs/hardware.md`](docs/hardware.md) before connecting hardware.

The program entry point is `app_main()` in `main/datalogger.cpp`.

## Build and flash

Requirements:

- ESP-IDF with ESP32 support
- A configured serial connection to the target board

```bash
idf.py set-target esp32
idf.py build
idf.py -p <PORT> flash monitor
```

Replace `<PORT>` with the board's serial port.

## Source layout

```text
main/
├── datalogger.cpp          Application entry point and FreeRTOS tasks
├── setup.cpp               UART and I2C initialization
├── pins.h                  GPIO assignments
├── hal/                    Sensor, radio, and SD-card drivers
└── modules/
    ├── telemetry.*         Combines sensor readings into one dataset
    └── commandHandler.*    Executes received radio commands
```

## Current limitations

- Hardware initialization assumes all configured devices are connected.
- Radio commands use a simple text protocol without checksums.
- SD files contain packed binary `SDataset` records and are not portable if that structure changes.
- Several devices and runtime flags are global; this is a future refactoring target.

