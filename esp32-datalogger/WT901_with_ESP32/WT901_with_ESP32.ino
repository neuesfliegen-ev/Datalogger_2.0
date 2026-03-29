#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "telemetry.h"
#include "REG.h"
#include "wit_c_sdk.h"
#include "wit_myFunctions.h"
#include "radio_com.h"
#include "Pitot.h"
//#include "gps.h"
//#include "sd.h"
constexpr int DEBUG = 1;


//Serial communication
HardwareSerial Radio(2); //UART 2
HardwareSerial GPS(1); //UART 1
TwoWire IMU(1); //I2C 1
//TwoWire Pitot(0); //I2C 0
//SPI SPI(0); //SD card

//static states
static uint8_t LOG_STATE = 0;
static uint8_t DISPLAY_DATA_ON_MONITOR = 1;

static struct DataLine dataline;//buf + size, csv
static struct Telemetry t;//all datapoints 

//Arduino IDE Serial functions -------------------
int8_t serial_poll(uint8_t cmd);
//------------------------------------------------

void setup() {//just do setup based on Serial on or off - 2 blocks
  Serial.begin(115200);
  delay(300);
  Serial.println("-- SETUP --");

  //start radio
  if(DEBUG){Serial.println("Radio.begin");}
  Radio.begin(9600, SERIAL_8N1, Radio_RX2, Radio_TX2); pinMode(LORA_M0, OUTPUT); pinMode(LORA_M1, OUTPUT); setupRadio(&Radio); 
  Serial.print("radio mode: "); Serial.println(get_radio_mode());
  

  //start IMU
  if(DEBUG){Serial.println("IMU.begin");}
  IMU.begin(IMU_SDA, IMU_SCL); setupIMU(&IMU);

  //start SD card reader
  //SPI.begin();

  //start GPS
  //GPS.begin(115200, SERIAL_8N1, GPS_RX1, GPS_TX1);
  //GPSsetup(GPS);

  //start Pitot
  //Pitot.begin();
}

void loop() {

  //Radio poll
  uint8_t buf[128] = {'h','i','!','\r','\n'};
  Radio.write(buf, sizeof(buf));
  uint8_t buf1[128] = {0x00, 0x01, 0x01, 's', 'e', 'n', 't', ' ', 'w', 'i', 't', 'h', ' ', 'h','1', '\r', '\n'};
  Radio.write(buf1, sizeof(buf1));
  uint8_t buf2[128] = {0x00, 0x02, 0x01, 's', 'e', 'n', 't', ' ', 'w', 'i', 't', 'h', ' ', 'h','2', '\r', '\n'};
  Radio.write(buf2, sizeof(buf2));
  
  int len = Radio.available(); 
  if (len > 0) {
    if (len > 127) len = 127;
    Radio.readBytes(buf, len);
    buf[len] = '\0';

    Serial.print("LEN: ");
    Serial.println(len);
    Serial.print("TXT: ");
    Serial.println((char*)buf);
  }
  
  //Serial.println("radio available check");
  
  if(Radio.available() > 0){
    Serial.println("sth in radio buffer");  
    uint8_t cmd = Radio.read();
    uint8_t arg = Radio.read();
    Serial.print(cmd); Serial.print(" "); Serial.println(arg);
    radio_poll(cmd, arg);
  }

  //Serial.println("serial available check");
  if(Serial.available() > 0){
    uint8_t cmd = Serial.read();
    uint8_t arg = Serial.read();
    serial_poll(cmd, arg);
    Serial.print(F("cmd was: "));
    Serial.println(cmd);
  }
  

  //Sensor poll
    //Read sensor values & generate Dataline
  //Serial.println("read imu");
  
  updateTelemetry(&t);

  //Serial.println("update dataline");
  updateDataLine(&dataline, &t);

  //Displaying data in monitors
  if(DISPLAY_DATA_ON_MONITOR == 1){
    Serial.write((uint8_t*)dataline.buf, strlen(dataline.buf));
    //sendDataLinetoDisplay(dataline, &Radio);
  }
  if(DISPLAY_DATA_IN_TERMINAL == 1){
    Radio.write((uint8_t*)dataline.buf, strlen(dataline.buf));
  }

  //SD poll
  //if(LOG_STATE == 1){Serial.println("logged line to SD");}

  delay(2000);
}

//Arduino IDE Serial Monitor functions
int8_t serial_poll(uint8_t cmd, uint8_t arg){
  switch(cmd){
    case 48: Serial.println("all the cmds"); break;
    //case 49: startLogging(); break;
    //case 50: stopLogging(); break;
    case 51: wit_calibrate_all(); Serial.println("Calibrating everything"); break;
    case 52: DISPLAY_DATA_ON_MONITOR = 1; break;
    case 53: DISPLAY_DATA_ON_MONITOR = 0; break;
    case 54: Serial.write((uint8_t*)dataline.buf, sizeof(dataline.buf)); break;
    case 55: set_radio_mode(arg); break;
    case 56: get_radio_mode(); break;
    default: Serial.println("Not acceptable cmd"); return -1;
  }
  return 1;
}

