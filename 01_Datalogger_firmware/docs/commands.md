# Radio Commands

## Input format

Send one ASCII command per line:

```text
<command> <option>\n
```

Both integers are currently required. Example:

```text
3 0
```

Commands are received on UART2 at 9600 baud, 8 data bits, no parity, and 1 stop bit.

## Command list

| Command | Option | Action |
|---:|---:|---|
| `0` | `0` | Start accelerometer calibration |
| `0` | `1` | Intended to start magnetometer calibration; currently rejected by the handler condition |
| `1` | any integer | Stop IMU calibration |
| `2` | file number | Open `/flights/<option>.bin` and enable logging |
| `3` | any integer | Enable telemetry |
| `4` | any integer | Flush the log, close/unmount the SD card, and disable logging |
| `5` | any integer | Disable telemetry |
| `6` | any integer | Send the help acknowledgement |

## Output format

Information messages start with `I: `.

```text
I: Received command 3: starting telemetry...
```

Telemetry messages start with `D: ` and contain comma-separated values. The current radio telemetry contains:

```text
t, ax, ay, az, gx, gy, gz, hx, hy, hz,
roll, pitch, yaw, temperature, height, pressure
```

## Parser behavior

- A command is processed only after `\n` or `\r` is received.
- The input buffer holds at most 63 characters plus the terminator.
- Invalid input currently produces no parser error response.

