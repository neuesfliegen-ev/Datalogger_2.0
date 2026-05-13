#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include "commandHandler.h"
#include "radio.h"
#include "WT901_I2C.h"
#include "telemetry.h"

CommandHandler::CommandHandler(RadioClass& radio, CJY901& imu, Telemetry& telemetry, bool& TELEMETRY_ENABLED) : radio(radio), imu(imu), telemetry(telemetry), telemetryEnabled(TELEMETRY_ENABLED){}

esp_err_t CommandHandler::executeCommand(int command, int option) {

    switch(command) {
        case 0:
            // calibrate IMU
            radio.writeMessage("Calibrating IMU...\n");
            break;

        case 1:
            // start telemetry
            radio.writeMessage("Starting telemetry...\n");
            break;

        case 2:
            // stop telemetry
            radio.writeMessage("Stopping telemetry...\n");
            break;

        default:
            radio.writeMessage("Invalid command, flushing uart buffer\n");
    }

    return ESP_OK;
}