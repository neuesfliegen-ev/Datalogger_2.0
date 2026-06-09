#pragma once

#include "esp_check.h"
#include "hal/imu.h"
#include "hal/gps.h"
#include "hal/airspeed.h"

/* Structs */
// Struct to save all three sensor data in one struct.
struct SDataset{
    uint32_t t;

    // IMU acceleration
    float ax, ay, az;

    // IMU gyroscope
    float gx, gy, gz;

    // IMU magnetometer
    float hx, hy, hz;

    // IMU attitude
    float roll, pitch, yaw;

    // IMU temperature / barometer
    float tmp;
    float hght;
    float press;

    // Pressure sensors
    float dpress;
    float airspeed_temp;

    // GPS
    int32_t lat; 			// degrees * 1e7
    int32_t lon;          	// degrees * 1e7
    uint16_t ground_speed; 	// km/h * 100
    int32_t gps_alt;      	// meters * 100

    // Other
    uint32_t s_count;

	uint8_t gps_hour;
	uint8_t gps_minute;
	uint8_t gps_second;
	uint16_t gps_millisecond;

	uint8_t gps_day;
	uint8_t gps_month;
	uint16_t gps_year;
};

// Struct to save all data, in short (register level)
struct SDataset_raw{
	short ax, ay, az;
	short gx, gy, gz;
	short hx, hy, hz;
	short roll, pitch, yaw;
	short tmp;
	short hght;
    uint16_t pressure_raw;
    uint16_t temp_raw;
};

/* Classes */
// Telemetry class to save all the data in one struct, and update the data from the sensors.
class Telemetry{
public:
	Telemetry(){};
	SDataset dataset;
	esp_err_t update_telemetry(uint32_t, CJY901&, GPSClass&, AirspeedClass&);

private:
};
