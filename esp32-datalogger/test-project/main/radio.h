#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "telemetry.h"

typedef struct {
    char type[2]; //"D:" or "I:"
    char text[1024];
    size_t len;
} RadioMessage;

typedef struct {
    uint8_t type;
    Telemetry& data;
} RadioDataBin;

typedef struct{

} RadioInfoBin;

struct RadioCommand {
    int command;
    int option;
};

class RadioClass {
private:
    gpio_num_t M0_PIN;
    gpio_num_t M1_PIN;
    uart_port_t uart_num;
    QueueHandle_t radioQueue;

public:
    RadioClass(gpio_num_t, gpio_num_t);

    void setQueue(QueueHandle_t);

    void startUART(uart_port_t p);

    bool readCommand(int& cmd, int& option);

    int readBytes(void *buf, uint32_t length);

    int writeBytes(const uint8_t* data, size_t length);

    int sendMessage(const char* msg);

    int sendData(const uint8_t*, size_t);

    int sendDataset(Telemetry*);
};

#endif