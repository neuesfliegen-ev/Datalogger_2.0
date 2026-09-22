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

void generateFileName(int option, char *buffer, size_t buffer_size){
    snprintf(
        buffer,
        buffer_size,
        "/flights/%d.bin",
        option
    );
}

CommandHandler::CommandHandler(RadioClass& radio, CJY901& imu, AirspeedClass& airspeed, GPSClass& gps, SDCard& sd, Telemetry& telemetry, bool& LOGGING_ENABLED, bool& TELEMETRY_ENABLED) 
    : radio(radio), imu(imu), airspeed(airspeed), gps(gps), sd(sd), telemetry(telemetry), loggingEnabled(LOGGING_ENABLED), telemetryEnabled(TELEMETRY_ENABLED){}

esp_err_t CommandHandler::executeCommand(int command, int option) {

    switch(command) {
        case 0: //START CALIBRATION (ALL SENSORS)
            radio.sendMessage("Received command 0: starting calibration, stop when ready...\n");
            if(option >= 0) {
                calibrate(option);
            }else{
                radio.sendMessage("Missing argument: calibrate what?\n");
            }
            break;

        case 1: //STOP CALIBRATION
            radio.sendMessage("Received command 1: stopping calibration...\n");
            imu.stopCalibrating();
            radio.sendMessage("IMU in normal mode!\n");
            break;  

        case 2: //START LOGGING ---- !problem could be naming a file name the same
            radio.sendMessage("Received command 2: generating file, starting logging...\n");
            loggingEnabled = true;
            char filename[64];
            generateFileName(option, filename, sizeof(filename));
            sd.openLogFile(filename);
            radio.sendMessage("Bin file writing!\n");
            break;

        case 3: //START TELEMETRY
        	radio.sendMessage("Received command 3: starting telemetry...\n");
            telemetryEnabled = true;
            break;

        case 4: //STOP LOGGING - loses last few seconds of data
            radio.sendMessage("Received command 4: stopping logging...\n");
            sd.flush();
            sd.end();
            loggingEnabled = false;
            radio.sendMessage("Flushed and saved!\n");
            break;

        case 5: //STOP TELEMETRY
            radio.sendMessage("Received command 5: stopping telemetry...\n");
            telemetryEnabled = false;
            break;

        case 6: //HELP - LIST AVAILABLE COMMANDS
            radio.sendMessage("Received command 6: getting help...\n");
            //radio.sendMessage("Type command + option e.g. cmd \"1\" and opt \"1\" as \"0 1\"\n");
            break;

        default:
            radio.sendMessage("Invalid command, flushing uart buffer\n");
    }

    return ESP_OK;
}

esp_err_t CommandHandler::calibrate(int option){
    switch(option){
        case 0: return imu.calibrateAcc(); break;
        case 1: return imu.calibrateMag(); break;
        default: radio.sendMessage("Invalid parameter: option\n");
    }
    return ESP_OK;
}