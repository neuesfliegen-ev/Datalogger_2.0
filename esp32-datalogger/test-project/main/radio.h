#ifndef RADIO_CLASS_H
#define RADIO_CLASS_H

#include "driver/uart.h"

/* Classes */

// Radio class to handle the communication with the radio module, using UART.
class RadioClass
{
public:
  RadioClass() {};
  void startUART(uart_port_t); // called from main
  /* Send for laptop RF, Read for commmand from laptop - methods should be defined here */
private:
  uart_port_t uart_port;
};

#endif