# LoRa Radio Module Configuration & Message Transmission

## Radio Module Configuration
Problem with radio link? It's usually the hardware settings (module configuration).


Configuration settings to be set:
| AT command | Meaning |
|---|---|
| AT+UART=7,0 | 115200 baud rate, 8N1 |
| AT+RATE=5 | 19.2 kbps |
| AT+CHANNEL=1 | using channel 1 |
| AT+ADDR=1 | set different number for each module |

### 1. Wiring

Wire the USB-TTL converter to the LoRa module.

| Pin | Level |
|---|---|
| M1 | HIGH |
| M0 | GND |

### 2. Configure the Serial Port

In the **left terminal window**, connect to the correct `/dev/ttyUSB0` port:

```bash
sudo stty -F /dev/ttyUSB0 115200 cs8 -cstopb -parenb raw -echo
```

If the module hasn't been set to 115200 baud rate yet, then use the default 9600.

!!!!!You need the raw or else u get an error

### 3. Listen to the Port

In the **left terminal window**:

```bash
cat /dev/ttyUSB0
```

Keep this running to listen for responses from the module.

### 4. Send an AT Command

In the **right terminal window**:

```bash
sudo echo "AT+HELP=?" > /dev/ttyUSB0
```

### Common Errors

#### Port Busy

Usually caused by another program or terminal session already using `/dev/ttyUSB0`.

Close the other session or application using the port.

#### Permission Denied

If you get a permission error when accessing the serial port, add your user to the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in for the group change to take effect.

---

# Sending Messages

For normal transmission mode, set both pins to GND:

| Pin | Level |
|---|---|
| M1 | GND |
| M0 | GND |

### 1. Keep the Listener Running

Keep the following command running in the **left terminal window**:

```bash
cat /dev/ttyUSB0
```

### 2. Send Binary Data

In the **right terminal window**:

```bash
printf '\x01\x02\x03\x04' > /dev/ttyUSB0
```

### 3. Send Text

Send plain text:

```bash
printf 'text' > /dev/ttyUSB0
```

Or send text followed by a newline:

```bash
printf 'text\n' > /dev/ttyUSB0
```

The receiving LoRa module should output the transmitted data through its serial interface.


<img width="1918" height="718" alt="image" src="https://github.com/user-attachments/assets/b347c34d-f611-4019-88bf-331b83d70862" />
