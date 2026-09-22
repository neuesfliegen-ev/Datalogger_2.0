#pragma once

#include "modules/telemetry.h"
#include "hal/radio.h"
#include "hal/imu.h"
#include "sd_card.h"
#include "hal/gps.h"


class CommandHandler {

public:
    CommandHandler(RadioClass& radio, CJY901& imu, AirspeedClass& airspeed, GPSClass& gps, SDCard& sd, Telemetry& telemetry, bool& LOGGING_ENABLED, bool& TELEMETRY_ENABLED);

    esp_err_t executeCommand(int command, int option);

private:
    RadioClass& radio;
    CJY901& imu;
    AirspeedClass& airspeed;
    GPSClass& gps;
    SDCard& sd;
	Telemetry& telemetry;
    bool& loggingEnabled;
	bool& telemetryEnabled;

    esp_err_t calibrate(int option);

};
