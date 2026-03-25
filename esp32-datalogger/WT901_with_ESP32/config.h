#pragma once
#include <stdint.h>
#include "telemetry.h"

//Constants
constexpr int DEBUG = 1;

//constexpr int Radio_RX2 = 16; //Radio
//constexpr int Radio_TX2 = 17;
constexpr int GPS_RX1 = 3; //GPS
constexpr int GPS_TX1 = 4;
constexpr int IMU_SDA = 21;//IMU
constexpr int IMU_SCL = 22;
//constexpr int LORA_M0 = 4;//LoRa M0 pin
//constexpr int LORA_M1 = 0;//LoRa M1 pin -- transmission mode = (0,0); configuration mode = (0,1) 
constexpr uint8_t IMU_I2C_ADDR = 0x50; //I2C slave address for wit IMU 

//rates
constexpr uint8_t LORA_RATE = 9600;

