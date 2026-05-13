#ifndef TELEMETRY_H
#define TELEMETRY_H

#include "esp_check.h"
#include "imu_data.h"

struct SDataset {
	uint32_t t;
	float ax, ay, az;
	float gx, gy, gz;
	float hx, hy, hz;
	float roll, pitch, yaw;
	float tmp;
	float hght, press; 
	float psta, pdyn;
	float alt, gs, lat, lon;
};

struct SDataset_raw {
	short ax, ay, az;
	short gx, gy, gz;
	short hx, hy, hz;
	short roll, pitch, yaw;
	short tmp;
	short hght; 
	float psta, pdyn;
	float alt, gs, lat, lon;
};

class Telemetry{
public:
	Telemetry(){};
	struct SDataset dataset;
	struct SDataset rawDataset;
	esp_err_t update_telemetry(const IMUData& imuData);
private:

};


#endif