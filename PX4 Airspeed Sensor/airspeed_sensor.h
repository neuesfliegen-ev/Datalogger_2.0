#ifndef AIRSPEED_SENSOR_H
#define AIRSPEED_SENSOR_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "airspeed_data.h"

#include <stdint.h>
#include <math.h>

// MS4525DO I2C address
// Check your exact sensor model; common PX4 airspeed sensor uses 0x28
#define MS4525DO_I2C_ADDR 0x28

// Sensor sends fixed 4-byte data frame
#define MS4525DO_DATA_LENGTH 4

// Output type A: 10% to 90% counts
#define MS4525DO_PRESSURE_MIN_COUNTS 1638.0f
#define MS4525DO_PRESSURE_MAX_COUNTS 14745.0f

// 11-bit temperature max count
#define MS4525DO_TEMP_MAX_COUNTS 2047.0f

// Pressure conversion
#define PSI_TO_PA 6894.76f

// Change these depending on your exact sensor pressure range
// Example: ±1 psi differential sensor
#define MS4525DO_PRESSURE_MIN_PSI -1.0f
#define MS4525DO_PRESSURE_MAX_PSI 1.0f

// Standard sea-level air density
#define AIR_DENSITY_SEA_LEVEL 1.225f

class CAirspeedSensor
{
public:
    AirspeedData airspeedData;

    CAirspeedSensor() {}

    void StartIIC(i2c_master_dev_handle_t bus_handle);

    esp_err_t readRawData();

    esp_err_t updatePressure();
    esp_err_t updateTemperature();
    esp_err_t updateAirspeed();

    AirspeedData &updateAll();

    void printAllData();

private:
    i2c_master_dev_handle_t bus_handle;

    uint8_t rawBuffer[MS4525DO_DATA_LENGTH];

    unsigned short rawPressure;
    unsigned short rawTemperature;

    uint8_t statusBits;

    esp_err_t readSensorBytes(uint8_t *dest, uint8_t length);

    float convertPressureToPa(unsigned short rawPressure);
    float convertTemperatureToC(unsigned short rawTemperature);
    float calculateAirspeed(float pressurePa);
};

#endif
