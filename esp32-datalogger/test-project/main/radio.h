#ifndef RADIO_CLASS_H
#define RADIO_CLASS_H

#include "driver/uart.h"

class RadioClass 
{
  public: 
    RadioClass(){};
    void startUART(uart_port_t);
  private:
    uart_port_t uart_port;
    void readBytes();
    void writeBytes();

};

#endif