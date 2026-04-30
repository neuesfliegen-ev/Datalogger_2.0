#pragma once
#include <stddef.h>
#include <stdint.h> 
#include <Wire.h>

struct DataLine{
  char buf[1000];
  size_t len;
};

struct Telemetry{
  int32_t timestamp;
  //IMU
  float ax; float ay; float az;
  float gx; float gy; float gz;
  float hx; float hy; float hz;
  float roll; float pitch; float yaw;
  float temp;//[C]
  float imuHeight;//[cm] 
  float imuPress;//[Pas]
  //GPS
  float lat; float lon;
  float gpsAlt; float gSpeed;
  //Pitot
  float statPress;
  float dynPress;
};

void updateDataLine(struct DataLine* dl, struct Telemetry* t);

void updateTelemetry(Telemetry* t);

