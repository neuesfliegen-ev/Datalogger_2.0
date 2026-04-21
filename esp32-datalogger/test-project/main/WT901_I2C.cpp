
#include "WT901_I2C.h"
#include "string.h"



void CJY901::StartIIC(i2c_master_dev_handle_t h){
  bus_handle = h;
};


esp_err_t CJY901::readRegisters(uint8_t addressToRead, uint8_t bytesToRead, uint8_t *dest){
  esp_err_t result = i2c_master_transmit_receive(bus_handle, &addressToRead, 1, dest, bytesToRead, -1); return result;

};

esp_err_t CJY901::writeRegister(uint8_t *dataToWrite, uint8_t bytesToWrite){
  esp_err_t result = i2c_master_transmit(bus_handle, dataToWrite, 1+bytesToWrite, -1); return result;
};

//--UPDATE MEASUREMENTS-------------------------------
esp_err_t CJY901::updateAcc(){
  uint8_t buf[8];
  esp_err_t result = readRegisters(AX, 8, buf);

  int16_t ax_raw = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
  int16_t ay_raw = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
  int16_t az_raw = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);
  int16_t tmp_raw = (int16_t)(((int16_t)buf[7] << 8) | buf[6]);
  stcAcc.a[0] = ax_raw; stcAcc.a[1] = ay_raw; stcAcc.a[2] = az_raw; 
  stcAcc.af[0] = ((float)ax_raw / 32768.0f) * 16.0f; stcAcc.af[1] = ((float)ay_raw / 32768.0f) * 16.0f; stcAcc.af[2] = ((float)az_raw / 32768.0f) * 16.0f; 
  stcTmp.tmp = tmp_raw; stcTmp.tmpf = ((float)tmp_raw / 100.0f); 

  return result;
};

esp_err_t CJY901::updateGyr(){
    uint8_t buf[6];
    esp_err_t result = readRegisters(GX, 6, buf);

    int16_t gx_raw = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
    int16_t gy_raw = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
    int16_t gz_raw = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);

    stcGyr.w[0] = gx_raw;
    stcGyr.w[1] = gy_raw;
    stcGyr.w[2] = gz_raw;

    stcGyr.wf[0] = ((float)gx_raw / 32768.0f) * 2000.0f;
    stcGyr.wf[1] = ((float)gy_raw / 32768.0f) * 2000.0f;
    stcGyr.wf[2] = ((float)gz_raw / 32768.0f) * 2000.0f;

  return ESP_OK;
};

esp_err_t CJY901::updateMag() {
    uint8_t buf[8];
    esp_err_t result = readRegisters(HX, 8, buf);

    int16_t hx_raw = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
    int16_t hy_raw = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
    int16_t hz_raw = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);
    int16_t tmp_raw = (int16_t)(((int16_t)buf[7] << 8) | buf[6]);

    stcMag.h[0] = hx_raw; stcMag.h[1] = hy_raw; stcMag.h[2] = hz_raw;

    stcTmp.tmp = tmp_raw; stcTmp.tmpf = ((float)tmp_raw / 100.0f);

    return ESP_OK;
};

esp_err_t CJY901::updateBar() {
    uint8_t buf[8];
    esp_err_t result = readRegisters(PressureL, 8, buf);

    int32_t pressure_raw = ((int32_t)buf[3] << 24) | ((int32_t)buf[2] << 16) | ((int32_t)buf[1] << 8)  | ((int32_t)buf[0]);
    int32_t height_raw = ((int32_t)buf[7] << 24) | ((int32_t)buf[6] << 16) | ((int32_t)buf[5] << 8)  | ((int32_t)buf[4]);

    stcBar.lPressure = (float)pressure_raw;          // Pa
    stcBar.lHeight = ((float)height_raw) / 100.0f;   // cm -> m

    return ESP_OK;
};

esp_err_t CJY901::updateAtt() {
    uint8_t buf[6];
    esp_err_t result = readRegisters(Roll, 6, buf);
    if (result != ESP_OK) return result;

    int16_t roll_raw  = (int16_t)(((int16_t)buf[1] << 8) | buf[0]);
    int16_t pitch_raw = (int16_t)(((int16_t)buf[3] << 8) | buf[2]);
    int16_t yaw_raw   = (int16_t)(((int16_t)buf[5] << 8) | buf[4]);

    stcAtt.Att[0] = roll_raw;
    stcAtt.Att[1] = pitch_raw;
    stcAtt.Att[2] = yaw_raw;

    stcAtt.Attf[0] = ((float)roll_raw  / 32768.0f) * 180.0f;
    stcAtt.Attf[1] = ((float)pitch_raw / 32768.0f) * 180.0f;
    stcAtt.Attf[2] = ((float)yaw_raw   / 32768.0f) * 180.0f;

    return ESP_OK;
};

void CJY901::updateAll(){
  updateAcc();
  updateGyr();
  updateAtt();
  updateBar();
  updateMag();  
}

//--COMMANDS-----------------------------------------
esp_err_t CJY901::calibrateAcc() {
  esp_err_t err = unlock();
  if (err != ESP_OK) return err;
  
  uint8_t toWrite[] = {CALSW, CALGYROACC};
  err = writeRegister(toWrite, 2);
  if (err != ESP_OK) return err;
  
  return save();
}

esp_err_t CJY901::calibrateMag() {
  esp_err_t err = unlock();
  if (err != ESP_OK) return err;
  
  uint8_t toWrite[] = {CALSW, CALMAG};
  err = writeRegister(toWrite, 2);
  if (err != ESP_OK) return err;
  
  return save();
}

esp_err_t CJY901::stopCalibrating() {//unlock, set calsw normal, save
  esp_err_t err = unlock();
  if (err != ESP_OK) return err;
  
  uint8_t toWrite[] = {CALSW, NORMAL};
  err = writeRegister(toWrite, 2);
  if (err != ESP_OK) return err;
  
  return save();
}

esp_err_t CJY901::unlock() {
  uint8_t array[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};  
  esp_err_t r = writeRegister(array, 5);  
  return r;
}

esp_err_t CJY901::save() {
  uint8_t array[5] = {0XFF, 0XAA, 0X00, 0X00, 0X00};
  esp_err_t r = writeRegister(array, 5);
  return r;
}
