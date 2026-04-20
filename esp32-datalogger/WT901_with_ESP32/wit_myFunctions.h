#pragma once
#include <Wire.h>

constexpr int IMU_SDA = 21;//IMU
constexpr int IMU_SCL = 22;
constexpr uint8_t IMU_I2C_ADDR = 0x50;

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
  float hx, hy, hz;
  float roll, pitch, yaw;
  float temp;
  float height, pressure;
};

//get rid of acc and mag, just do all
int32_t setupIMU(TwoWire*);
int8_t wit_calibrate_all();
int8_t readEverythingFromIMU(IMUData* t);