#include <cstdint>
#include "telemetry.h"


esp_err_t Telemetry::update_telemetry(const IMUData& imuData){
	dataset.ax = imuData.acc.af[0]; dataset.ay = imuData.acc.af[1]; dataset.az = imuData.acc.af[2];
	dataset.gx = imuData.gyr.wf[0]; dataset.gy = imuData.gyr.wf[1]; dataset.gz = imuData.gyr.wf[2];  
	dataset.hx = imuData.mag.h[0]; dataset.hy = imuData.mag.h[1]; dataset.hz = imuData.mag.h[2];  
	dataset.roll = imuData.att.Attf[0]; dataset.pitch = imuData.att.Attf[1]; dataset.yaw = imuData.att.Attf[2];
	dataset.tmp = imuData.tmp.tmpf; 
	dataset.hght = imuData.bar.lHeight; dataset.press = imuData.bar.lPressure;

	

	return ESP_OK;
}