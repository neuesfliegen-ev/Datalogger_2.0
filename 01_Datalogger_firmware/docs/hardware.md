# Hardware

The current configuration targets an ESP32.

## Connections

| Device | Signal | ESP32 GPIO | Interface |
|---|---|---:|---|
| Radio | TX from ESP32 | 17 | UART2, 9600 baud |
| Radio | RX to ESP32 | 16 | UART2, 9600 baud |
| Radio | M0 | 34 | Mode pin or ground |
| Radio | M1 | 35 | Mode pin or ground |
| GPS | TX from ESP32 | 33 | UART1, 115200 baud |
| GPS | RX to ESP32 | 32 | UART1, 115200 baud |
| IMU | SDA | 21 | I2C1, address `0x50` |
| IMU | SCL | 22 | I2C1, 100 kHz |
| Airspeed sensor | SDA | 27 | I2C0, address `0x28` |
| Airspeed sensor | SCL | 14 | I2C0, 100 kHz |
| SD card | MISO | 19 | SPI3 |
| SD card | MOSI | 23 | SPI3 |
| SD card | SCK | 18 | SPI3 |
| SD card | CS | 5 | SPI3 |
| LED | Signal | 2 | GPIO |

The ESP32 TX pin connects to the peripheral RX pin, and the ESP32 RX pin connects to the peripheral TX pin.

## Important notes

- The code enables internal I2C pull-ups, but the existing pin definitions also note external pull-ups for the IMU and airspeed buses.
- GPIO34 and GPIO35 are input-only on the ESP32. The current firmware stores the radio M0/M1 assignments but does not configure or drive them.
- `serial_buses_setup()` initializes all configured buses. Do not run it without checking the connected hardware.
- The SD card is mounted at `/sdcard`.
