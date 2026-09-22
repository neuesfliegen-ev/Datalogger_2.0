#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "telemetry.h"

enum class RadioTxType : uint8_t {
    Telemetry = 1,
    Info      = 2,
    Ack       = 3
};

struct RadioHeader {
    uint8_t type;
    uint16_t len;
};

struct RadioCommand {
    uint8_t command;
    uint8_t option;
};

struct RadioMessage {
    char text[256];
    size_t len;
};

class RadioClass {
public:
    RadioClass(gpio_num_t, gpio_num_t);
    void setQueue(QueueHandle_t);
    void startUART(uart_port_t p);

    bool readCommand(int& cmd, int& option);

    int readBytes(void *buf, uint32_t length);
    int writeBytes(const uint8_t* data, size_t length);

    int sendMessage(const char* msg);
    int sendDataset(Telemetry*);

private:
    gpio_num_t M0_PIN;
    gpio_num_t M1_PIN;
    uart_port_t uart_num;
    QueueHandle_t radioQueue;
};
