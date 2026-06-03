#pragma once

#include "driver/i2c_master.h"
#include "hal/imu_data.h"

/** Classes **/
class CJY901
{
public:
	IMUData imuData;

	CJY901(){};
	void StartIIC(i2c_master_dev_handle_t bus_handle);
	void CopeSerialData(unsigned char ucData);
	short ReadWord(unsigned char ucAddr);
	void WriteWord(unsigned char ucAddr, short sData);
	void ReadData(unsigned char ucAddr, unsigned char ucLength, char chrData[]);
	void updateTime();
	esp_err_t updateAcc();
	esp_err_t updateGyr();
	esp_err_t updateAtt();
	esp_err_t updateMag();
	esp_err_t updateBar();
	void updateDStatus();
	void updateLonLat();
	void updateGPSV();

	void Config();
	esp_err_t calibrateAcc();
	esp_err_t calibrateMag();
	esp_err_t stopCalibrating();
	// getOffsets();
	void checkOrientation();
	void checkAxis6Mode();
	IMUData &updateAll();
	void printAllData();
	esp_err_t unlock();
	esp_err_t lock();
	esp_err_t save();
	void reset();

private:
	// unsigned char ucDevAddr;
	i2c_master_dev_handle_t bus_handle;
	esp_err_t readRegisters(uint8_t addressToRead, uint8_t bytesToRead, uint8_t *dest);
	esp_err_t writeRegister(uint8_t *dataToWrite, uint8_t bytesToWrite);
};

