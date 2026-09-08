###Radio module configuration###
Wire the USB TTL converter to the LoRa module.
M1 to high, M0 to GND.

In left window
1. Connect to correct ttyUSB0 port: 
  sudo stty -F /dev/ttyUSB0 9600 cs8 -cstopb -parenb
2. Listen to the port:
  cat /dev/ttyUSB0

In right window
1. sudo echo "AT+HELP=?" > /dev/ttyUSB0

Possible errors: 
1. port busy
  Usually because of opened session in another window.


###Sending messages###
Both M1 and M0 to GND for transmission mode.

1. Keep cat window for listening
2. For writing #
   printf '\x01\x02\x03\x04' > /dev/ttyUSB0
   OR
   printf 'text' > /dev/ttyUSB0

