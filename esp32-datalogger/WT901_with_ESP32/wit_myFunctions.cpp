#include <Arduino.h>
#include <stdint.h>
#include "wit_c_sdk.h"
#include "wit_myFunctions.h"
#include "REG.h"
#include "telemetry.h"

static uint32_t ACC_CAL_TIME = 3000;
static uint32_t MAG_CAL_TIME = 3000;

TwoWire* p_IMU = nullptr;

int32_t myWitI2cWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen) { 
  p_IMU->beginTransmission(ucAddr);
  p_IMU->write(ucReg);
  //p_IMU->write(p_ucVal, uiLen);
  for(uint32_t i = 0; i < uiLen; i++){
    p_IMU->write(*p_ucVal++);
  }
  byte error = p_IMU->endTransmission(false);
  if(error){Serial.println("IMU writing error: "); Serial.println(error); };
  return 1;
}

int32_t myWitI2cRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen) { 
  p_IMU->beginTransmission(ucAddr);
  p_IMU->write(ucReg);
  byte error = p_IMU->endTransmission(false);
  if( error ){
    Serial.println("IMU writing error"); Serial.println(error);
  };
  if (p_IMU->requestFrom(ucAddr, uiLen) != uiLen) {
    return 0;
    Serial.println("witRead: request was 0");
  }  
  for (uint32_t i = 0; i < uiLen; i++) {p_ucVal[i] = p_IMU->read(); Serial.println(p_ucVal[i]);}  
  return 1;
}

int32_t setupIMU(TwoWire* ptr){
  p_IMU = ptr;
  int32_t r = WitI2cFuncRegister(myWitI2cWrite, myWitI2cRead);
  int32_t witInit_ = WitInit(WIT_PROTOCOL_I2C, IMU_I2C_ADDR);
  return r && witInit_;
}

static bool readRegisters(TwoWire* imu, uint8_t startReg, uint8_t* buf, uint8_t len) {
  imu->beginTransmission(IMU_I2C_ADDR);
  imu->write(startReg);
  if (imu->endTransmission(false) != 0) return false;
  if (imu->requestFrom(IMU_I2C_ADDR, len) != len) return false;

  for (uint8_t i = 0; i < len; i++) {
    buf[i] = imu->read();
  }
  return true;
}

static int16_t toInt16(uint8_t lo, uint8_t hi) {
  return (int16_t)((hi << 8) | lo);
}

int8_t readAcc(IMUData* t) {
  uint8_t buf[8]; // Ax Ay Az Temp = 4 values * 2 bytes
  if (!readRegisters(p_IMU, WIT_ACC, buf, 8)) return -1;

  t->ax   = toInt16(buf[0], buf[1]) / 32768.0f * 16.0f;
  t->ay   = toInt16(buf[2], buf[3]) / 32768.0f * 16.0f;
  t->az   = toInt16(buf[4], buf[5]) / 32768.0f * 16.0f;
  t->temp = toInt16(buf[6], buf[7]) / 100.0f;
  return 1;
}

int8_t readGyr(IMUData* t) {
  uint8_t buf[8];
  if (!readRegisters(p_IMU, WIT_GYRO, buf, 8)) return -1;

  t->gx = toInt16(buf[0], buf[1]) / 32768.0f * 2000.0f;
  t->gy = toInt16(buf[2], buf[3]) / 32768.0f * 2000.0f;
  t->gz = toInt16(buf[4], buf[5]) / 32768.0f * 2000.0f;
  return 1;
}

int8_t readAng(IMUData* t) {
  uint8_t buf[6];
  if (!readRegisters(p_IMU, WIT_ANGLE, buf, 6)) return -1;

  t->roll  = toInt16(buf[0], buf[1]) / 32768.0f * 180.0f;
  t->pitch = toInt16(buf[2], buf[3]) / 32768.0f * 180.0f;
  t->yaw   = toInt16(buf[4], buf[5]) / 32768.0f * 180.0f;
  return 1;
}

int8_t readBar(IMUData* t) {
  uint8_t buf[6];
  if (!readRegisters(p_IMU, WIT_PRESS, buf, 6)) return -1;

  t->hx = toInt16(buf[0], buf[1]);
  t->hy = toInt16(buf[2], buf[3]);
  t->hz = toInt16(buf[4], buf[5]);
  return 1;
}

int8_t readMag(IMUData* t) {
  uint8_t buf[8];
  if (!readRegisters(p_IMU, WIT_MAGNETIC, buf, 8)) return -1;

  t->pressure = (int16_t)((buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)| buf[0]);//[Pa]
  t->height = (int16_t)((buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)| buf[4]);//[cm]
  return 1;
}

int8_t readEverythingFromIMU(IMUData* t) {
  if (readAcc(t) < 0) return -1;
  if (readGyr(t) < 0) return -1;
  if (readAng(t) < 0) return -1;
  if (readMag(t) < 0) return -1;
  if (readBar(t) < 0) return -1;
  return 1;
}


//-----CALIBRATION---------------------------------------------------------------------------------

int8_t wit_calibrate_acc(void){
  Serial.println("Calibrate Acc + Gyro: lay IMU flat");
  delay(7000);
  Serial.println("Starting Acc + Gyro calibration, keep it still!");
  if(WitStartAccCali() != WIT_HAL_OK){
    Serial.println("WitStartAccCali error");
    return -1;
  }else{
    Serial.println("WitStartAccCali OK");
    delay(ACC_CAL_TIME);
    if(WitStopAccCali() != WIT_HAL_OK){
      Serial.println("WitStopAccCali WIT_HAL_ERROR");
      return -1;
    }
  };

  Serial.println("Finished Acc calibration");
  return 1;
}

int8_t wit_calibrate_mag(void){
  Serial.println("Calibrate Mag: rotate IMU for the next 45 seconds");
  delay(5000);
  Serial.println("Starting Mag calibration");
  if(WitStartMagCali() != WIT_HAL_OK){
    Serial.println("WitStartMagCali error");
    return -1;
  }else{
    Serial.println("WitStartMagCali OK");
      delay(MAG_CAL_TIME); //rotate along all axes
    if(WitStopMagCali() != WIT_HAL_OK){
      Serial.println("WitStopMagCali error");
      return -1;
    }else{
      Serial.println("WitStopMagCali OK");
    };  
  };
  Serial.println("Finished Mag calibration");
  return 1;
}

int8_t wit_calibrate_all(void){
  if(wit_calibrate_acc() == 1){
    if(wit_calibrate_mag() == 1) {
      return 1;
    }
  };
  Serial.println("couldn't calibrate everything");
  return -1;
}
