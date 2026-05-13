#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

#include "telemetry.h"

struct RadioCommand {
    int command;
    int option;
};

class RadioClass {
private:
    gpio_num_t M0_PIN;
    gpio_num_t M1_PIN;
    uart_port_t uart_num;

public:
    RadioClass(gpio_num_t, gpio_num_t);

    void startUART(uart_port_t p);

    bool readCommand(int& cmd, int& option);

    int readBytes(void *buf, uint32_t length);

    int writeBytes(const uint8_t* data, size_t length);

    void writeMessage(const char* msg);

    void sendData(Telemetry* dataset);
};

#endif