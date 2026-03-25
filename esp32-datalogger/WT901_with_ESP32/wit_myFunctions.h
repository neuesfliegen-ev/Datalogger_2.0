#pragma once
#include <Wire.h>
//---------ADD headers for wt_c_sdk_myFunctions-------------------------------------------------------------
//int32_t myWitI2cWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
//int32_t myWitI2cRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float hx, hy, hz;
  float roll, pitch, yaw;
  float temp;
  float height, pressure;
};

//get rid of acc and mag, just do all
int8_t wit_calibrate_acc();
int8_t wit_calibrate_mag();
int8_t wit_calibrate_all();
int8_t readEverythingFromIMU(TwoWire* imu, IMUData* t);