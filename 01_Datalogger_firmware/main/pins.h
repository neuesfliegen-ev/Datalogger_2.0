#pragma once

//LoRa module
const gpio_num_t RADIO_TX_PIN = GPIO_NUM_17;
const gpio_num_t RADIO_RX_PIN = GPIO_NUM_16;	
const gpio_num_t RADIO_M0_PIN = GPIO_NUM_34; //or ground directly	
const gpio_num_t RADIO_M1_PIN = GPIO_NUM_35; //or ground directly	

//GPS
const gpio_num_t GPS_TX_PIN = GPIO_NUM_33; //connect TX of gps
const gpio_num_t GPS_RX_PIN = GPIO_NUM_32; //connect RX of gps

//IMU
const gpio_num_t IMU_SDA_PIN = GPIO_NUM_21; //1k pullup
const gpio_num_t IMU_SCL_PIN = GPIO_NUM_22; //1k pullup

//Airspeed sensor
const gpio_num_t AIRSPEED_SDA_PIN = GPIO_NUM_27; //1k pull up --CHANGE
const gpio_num_t AIRSPEED_SCL_PIN = GPIO_NUM_14; //1k pull up --CHANGE

//SD card module
const gpio_num_t SD_MISO_PIN = GPIO_NUM_19; //VSPI
const gpio_num_t SD_CS_PIN = GPIO_NUM_5; //VPSI
const gpio_num_t SD_SCK_PIN = GPIO_NUM_18; //VSPI
const gpio_num_t SD_MOSI_PIN = GPIO_NUM_23; //VSPI 10k pullup

//LEDs
const gpio_num_t BLINK_LED = GPIO_NUM_2;

