#include "esp_check.h"
#include "hal/imu_data.h"

/* Structs */

// Struct to save all three sensor data in one struct.
struct SDataset
{
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

// Struct to save all data, in short (resistor level)
// In resistor level, the data is in short, nd needs to be converted to float
// using the conversion factors provided in the datasheets of the sensors.
struct SDataset_raw
{
	short ax, ay, az;
	short gx, gy, gz;
	short hx, hy, hz;
	short roll, pitch, yaw;
	short tmp;
	short hght;
	float psta, pdyn;
	float alt, gs, lat, lon;
};

/* Classes */
// Telemetry class to save all the data in one struct, and update the data from the sensors.
class Telemetry
{
public:
	Telemetry() {};
	struct SDataset dataset;
	struct SDataset rawDataset;
	esp_err_t update_telemetry(const IMUData &imuData);

private:
};
