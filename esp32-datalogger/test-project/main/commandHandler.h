#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H


#include "commandHandler.h"
#include "radio.h"
#include "WT901_I2C.h"
#include "telemetry.h"

class CommandHandler {

public:
    CommandHandler(RadioClass& radio, CJY901& imu, Telemetry& telemetry, bool& TELEMETRY_ENABLED);

    esp_err_t executeCommand(int command, int option);
private:
    RadioClass& radio;
    CJY901& imu;
	Telemetry& telemetry;
	bool& telemetryEnabled;


};

#endif