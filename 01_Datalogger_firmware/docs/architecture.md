# Architecture

## Startup

`app_main()` performs these steps:

1. Mount the SD card.
2. Initialize the IMU, airspeed sensor, radio UART, and GPS UART.
3. Create `radioQueue` and `datasetQueue`.
4. Start `polling_task` on core 0.
5. Start `logging_task` on core 1.

## Runtime flow

### `polling_task`

Runs approximately every 50 ms and:

1. Updates sensor data.
2. Builds the current `SDataset`.
3. Sends the dataset to `datasetQueue`.
4. Reads and executes radio commands.
5. Sends queued radio messages over UART.
6. Queues telemetry when telemetry is enabled.

### `logging_task`

Receives datasets from `datasetQueue`, writes them to the open SD file in blocks of 32, and periodically flushes the file.

## Data paths

```text
IMU + GPS + airspeed
        |
        v
    Telemetry
        |
        +--> datasetQueue --> logging_task --> SD card
        |
        +--> radioQueue --------------------> radio UART

radio UART --> readCommand() --> CommandHandler --> radioQueue
```

## Main components

| Component | Responsibility |
|---|---|
| `datalogger.cpp` | Owns application state and FreeRTOS tasks |
| `setup.cpp` | Configures UART and I2C buses |
| `Telemetry` | Copies sensor values into `SDataset` |
| `CommandHandler` | Applies commands to sensors, logging, and telemetry |
| `RadioClass` | Parses commands and queues outgoing messages |
| `SDCard` | Mounts the card and writes binary datasets |

