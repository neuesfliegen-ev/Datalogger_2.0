#include <Arduino.h>
#include <Wire.h>
#include <stdint.h>
#include "wit_c_sdk.h"
#include "wit_myFunctions.h"
#include "REG.h"
#include "telemetry.h"

constexpr uint8_t IMU_I2C_ADDR = 0x50;

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

int8_t readAcc(TwoWire* imu, IMUData* t) {
  uint8_t buf[8]; // Ax Ay Az Temp = 4 values * 2 bytes
  if (!readRegisters(imu, WIT_ACC, buf, 8)) return -1;

  t->ax   = toInt16(buf[0], buf[1]) / 32768.0f * 16.0f;
  t->ay   = toInt16(buf[2], buf[3]) / 32768.0f * 16.0f;
  t->az   = toInt16(buf[4], buf[5]) / 32768.0f * 16.0f;
  t->temp = toInt16(buf[6], buf[7]) / 100.0f;
  return 1;
}

int8_t readGyr(TwoWire* imu, IMUData* t) {
  uint8_t buf[8];
  if (!readRegisters(imu, WIT_GYRO, buf, 8)) return -1;

  t->gx = toInt16(buf[0], buf[1]) / 32768.0f * 2000.0f;
  t->gy = toInt16(buf[2], buf[3]) / 32768.0f * 2000.0f;
  t->gz = toInt16(buf[4], buf[5]) / 32768.0f * 2000.0f;
  return 1;
}

int8_t readAng(TwoWire* imu, IMUData* t) {
  uint8_t buf[6];
  if (!readRegisters(imu, WIT_ANGLE, buf, 6)) return -1;

  t->roll  = toInt16(buf[0], buf[1]) / 32768.0f * 180.0f;
  t->pitch = toInt16(buf[2], buf[3]) / 32768.0f * 180.0f;
  t->yaw   = toInt16(buf[4], buf[5]) / 32768.0f * 180.0f;
  return 1;
}

int8_t readBar(TwoWire* imu, IMUData* t) {
  uint8_t buf[6];
  if (!readRegisters(imu, WIT_PRESS, buf, 6)) return -1;

  t->hx = toInt16(buf[0], buf[1]);
  t->hy = toInt16(buf[2], buf[3]);
  t->hz = toInt16(buf[4], buf[5]);
  return 1;
}

int8_t readMag(TwoWire* imu, IMUData* t) {
  uint8_t buf[8];
  if (!readRegisters(imu, WIT_MAGNETIC, buf, 8)) return -1;

  t->pressure = (int16_t)((buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)| buf[0]);//[Pa]
  t->height = (int16_t)((buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)| buf[4]);//[cm]
  return 1;
}

int8_t readEverythingFromIMU(TwoWire* imu, IMUData* t) {
  if (readAcc(imu, t) < 0) return -1;
  if (readGyr(imu, t) < 0) return -1;
  if (readAng(imu, t) < 0) return -1;
  if (readMag(imu, t) < 0) return -1;
  if (readBar(imu, t) < 0) return -1;
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
    delay(8000);
    if(WitStopAccCali() != WIT_HAL_OK){
      Serial.println("WitStopAccCali WIT_HAL_ERROR");
      return -1;
    }else{
      Serial.println("WitStopAccCali OK");
    };  
  };

  Serial.println("Finished Acc + Gyro calibration");
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
      delay(3000); //rotate along all axes
    if(WitStopMagCali() != WIT_HAL_OK){
      Serial.println("WitStopMagCali error");
      return -1;
    }else{
      Serial.println("WitStopMagCali OK");
    };  
  };
  Serial.println("Finished Gyro calibration");
  return 1;
}

int8_t wit_calibrate_all(void){
  if(wit_calibrate_acc() == 1){
    if(wit_calibrate_mag() == 1) {
      //if(wit_calibrate() == 1)
      return 1;
    }
  };
  Serial.println("couldn't calibrate everything");
  return -1;
}

/*
int8_t calibrate_acc(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen){
  
}

int32_t startAccCali(void){
	if(
    (KEY, KEY_UNLOCK) != WIT_HAL_OK)	    return  WIT_HAL_ERROR;// unlock reg
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
	else ;
	if(WitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}

int32_t stopAccCali(void){
	if(WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	if(s_uiProtoclo == WIT_PROTOCOL_MODBUS)	p_WitDelaymsFunc(20);
	else if(s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
	else ;
	if(WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK)	return  WIT_HAL_ERROR;
	return WIT_HAL_OK;
}
*/