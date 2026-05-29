#include "hal/radio.h"
#include "modules/telemetry.h"
#include "esp_log.h"

extern bool TELEMETRY_ENABLED;
extern bool DEBUG;

const char DATA_HEADER[] = "D: ";
const char INFO_HEADER[] = "I: ";

//SETUP FUNCTIONS
RadioClass::RadioClass(gpio_num_t M0, gpio_num_t M1){
	M0_PIN = M0;
	M1_PIN = M1;
};

void RadioClass::setQueue(QueueHandle_t qh){
    radioQueue = qh;
}

void RadioClass::startUART(uart_port_t p){
	uart_num = p;
};

//OPERATIONAL FUNCTIONS
bool RadioClass::readCommand(int& command, int& option) {
    static char buffer[64];
    static int index = 0;
    uint8_t byte;

    int len = uart_read_bytes(uart_num, &byte, 1, 10);

    if (len <= 0) {return false; }

    if (byte == '\n' || byte == '\r') {
        if (index == 0) return false;

        buffer[index] = '\0';
        int parsed = sscanf(buffer, "%d %d", &command, &option);
        index = 0;
        return parsed == 2;
    }

    if (index >= sizeof(buffer) - 1) {
        index = 0;
        uart_flush(uart_num);
        sendMessage("Command too long, flushing UART\n");
        if(DEBUG)printf("Command too long, flushing UART\n");
        return false;
    }

    buffer[index++] = byte;
    return false;
}

int RadioClass::sendMessage(const char* text) {
    RadioMessage msg{};
    size_t pos = 0;

    constexpr const char* header = "I: ";

    memcpy(msg.text + pos, header, 3);
    pos += 3;

    size_t text_len = strlen(text);

    if (text_len > sizeof(msg.text) - pos - 2) {
        text_len = sizeof(msg.text) - pos - 2;
    }

    memcpy(msg.text + pos, text, text_len);
    pos += text_len;

    memcpy(msg.text + pos, "\r\n", 2);
    pos += 2;

    msg.len = pos;

    return xQueueSend(radioQueue, &msg, 0);
}

int RadioClass::sendDataset(Telemetry* telemetry){
    RadioMessage msg;  
    SDataset& d = telemetry->dataset;

    int len = snprintf(
        msg.text,
        sizeof(msg.text),
        "D: %lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
        d.t,
        d.ax, d.ay, d.az,
        d.gx, d.gy, d.gz,
        d.hx, d.hy, d.hz,
        d.roll, d.pitch, d.yaw,
        d.tmp,
        d.hght, d.press
    );

    if (len < 0 || len >= sizeof(msg.text)) {
        return -1;
    }

    msg.len = len;

    return xQueueSend(radioQueue, &msg, 0);
};


/* --DO NOT USE; ONLY FOR TESTING----
int RadioClass::readBytes(void *buf, uint32_t length){
    return uart_read_bytes(uart_num, buf, length, pdMS_TO_TICKS(100));
}

int RadioClass::writeBytes(const uint8_t* data, size_t length){
    return uart_write_bytes(uart_num, (const char*)data, length);
};
*/
