#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

#include "modules/telemetry.h"
#include "modules/commandHandler.h"


CommandHandler::CommandHandler(RadioClass& radio, CJY901& imu, Telemetry& telemetry, bool TELEMETRY_ENABLED) 
    : radio(radio), imu(imu), telemetry(telemetry), telemetryEnabled(TELEMETRY_ENABLED){}

esp_err_t CommandHandler::executeCommand(int command, int option) {

    switch(command) {
        case 0:
            // calibrate IMU
            if(option == 0) imu.calibrateAcc();
            if(option == 1) imu.calibrateMag();
            radio.sendMessage("Received command 0...\n");
            printf("Laptop received command 0\n");
            break;

        case 1:
            // start telemetry
            //imu.stopCalibrating();
            //radio.sendMessage("Received command 1...\n");
            printf("Laptop received command 1\n");
            break;

        case 2:
            // stop telemetry
            //telemetryEnabled = option;
            radio.sendMessage("Received command 2...\n");
            printf("Laptop received command 2\n");

            break;

        case 3:
        	radio.sendMessage("Received command 3...\n");
            printf("Laptop received command 3\n");

            break;

        case 4:
            radio.sendMessage("help?\n");
            break;

        default:
            radio.sendMessage("Invalid command, flushing uart buffer\n");
            printf("invalid command\n");
    }

    return ESP_OK;
}