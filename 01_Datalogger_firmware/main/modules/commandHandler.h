#pragma once

#include "modules/telemetry.h"
#include "hal/radio.h"
#include "hal/imu.h"

class CommandHandler {

public:
    CommandHandler(RadioClass& radio, CJY901& imu, Telemetry& telemetry, bool TELEMETRY_ENABLED);

    esp_err_t executeCommand(int command, int option);

private:
    RadioClass& radio;
    CJY901& imu;
	Telemetry& telemetry;
	bool telemetryEnabled;


};
