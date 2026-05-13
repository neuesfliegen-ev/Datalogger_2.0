#include "radio.h"
#include "telemetry.h"


RadioClass::RadioClass(gpio_num_t M0, gpio_num_t M1){
	M0_PIN = M0;
	M1_PIN = M1;
};

void RadioClass::startUART(uart_port_t p){
	uart_num = p;
};

bool RadioClass::readCommand(int& command, int& option) {
    static char buffer[64];
    static int index = 0;

    uint8_t byte;

    int len = uart_read_bytes(uart_num, &byte, 1, pdMS_TO_TICKS(10));

    if (len <= 0) {return false; }

    if (byte == '\n') {
       buffer[index] = '\0';
       int parsed = sscanf(buffer, "%d %d", &command, &option);
       index = 0;
       return parsed == 2;
    }

    if (index >= sizeof(buffer) - 1) {
        index = 0;
        uart_flush(uart_num);
        writeMessage("Command too long, flushing UART\n");
        return false;
    }

    buffer[index++] = byte;
    return false;
}

int RadioClass::readBytes(void *buf, uint32_t length){
    return uart_read_bytes(uart_num, buf, length, pdMS_TO_TICKS(100));
}

int RadioClass::writeBytes(const uint8_t* data, size_t length){
	return uart_write_bytes(uart_num, (const char*)data, length);
};

void RadioClass::writeMessage(const char* msg) {
    uart_write_bytes(uart_num, msg, strlen(msg));
};

void RadioClass::sendData(Telemetry* dataset){
	uart_write_bytes(uart_num, (const char*)dataset, sizeof(Telemetry));
};


