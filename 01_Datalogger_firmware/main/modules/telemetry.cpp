#include <cstdint>
#include "modules/telemetry.h"

/* Methods */

// update_telemetry
// This method takes in the data from the sensors, and updates the dataset struct with the new data.
esp_err_t Telemetry::update_telemetry(uint32_t now, CJY901 &imu, GPSClass &gps){
	
	dataset.t = now;
	// 1. IMU data
	dataset.ax = imu.imuData.acc.af[0];
	dataset.ay = imu.imuData.acc.af[1];
	dataset.az = imu.imuData.acc.af[2];

	dataset.gx = imu.imuData.gyr.wf[0];
	dataset.gy = imu.imuData.gyr.wf[1];
	dataset.gz = imu.imuData.gyr.wf[2];

	dataset.hx = imu.imuData.mag.h[0];
	dataset.hy = imu.imuData.mag.h[1];
	dataset.hz = imu.imuData.mag.h[2];

	dataset.roll  = imu.imuData.att.Attf[0];
	dataset.pitch = imu.imuData.att.Attf[1];
	dataset.yaw   = imu.imuData.att.Attf[2];

	dataset.tmp   = imu.imuData.tmp.tmpf;
	dataset.hght  = imu.imuData.bar.lHeight;
	dataset.press = imu.imuData.bar.lPressure;

    // GPS data
    dataset.lat          = gps.gps_.latitude;
    dataset.lon          = gps.gps_.longitude;
    dataset.ground_speed = gps.gps_.speed;
    dataset.gps_alt      = gps.gps_.altitude;

	dataset.gps_hour        = gps.dateTime_.time.hour;
	dataset.gps_minute      = gps.dateTime_.time.minute;
	dataset.gps_second      = gps.dateTime_.time.second;
	dataset.gps_millisecond = gps.dateTime_.time.millisecond;

	dataset.gps_day   = gps.dateTime_.date.day;
	dataset.gps_month = gps.dateTime_.date.month;
	dataset.gps_year  = gps.dateTime_.date.year;
	/* Other sensor data should be updated here */

	return ESP_OK;
}