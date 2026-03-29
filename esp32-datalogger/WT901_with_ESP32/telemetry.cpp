#include <Arduino.h>
#include <stdint.h>
#include "telemetry.h"
#include "wit_myFunctions.h"
//updateTelemetry and then updateDataLine

void updateDataLine(struct DataLine* dl, struct Telemetry* t) {
  snprintf(
    dl->buf,
    sizeof(dl->buf),
    "%lu,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n", //into char array buf
    millis(),
    (float)t->ax, (float)t->ay, (float)t->az,
    (float)t->gx, (float)t->gy, (float)t->gz,
    (float)t->hx, (float)t->hy, (float)t->hz,
    (float)t->roll, (float)t->pitch, (float)t->yaw,
    (float)t->temp, (float)t->imuHeight, (float)t->imuPress
  );

}
//take imu i2c leitung and pass poitner to telemetry struct and fill data
void updateTelemetry(Telemetry* t){
  IMUData imuData;//make to store new imu data
  readEverythingFromIMU(&imuData);
  t->timestamp = millis();
  t->ax = imuData.ax; t->ay = imuData.ay; t->az = imuData.az; t->temp = imuData.temp; 
  t->gx = imuData.gx; t->gy = imuData.gy; t->gz = imuData.gz;
  t->hx = imuData.hx; t->hy = imuData.hy; t->hz = imuData.hz;
  t->roll = imuData.roll; t->pitch = imuData.pitch; t->yaw = imuData.yaw;
  t->imuHeight = imuData.height; t->imuPress = imuData.pressure;
}