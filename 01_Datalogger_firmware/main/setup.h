#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "pins.h"
#include "hal/radio.h"
#include "hal/imu.h"
#include "modules/telemetry.h"
#include "modules/commandHandler.h"


extern bool DEBUG;
extern bool TELEMETRY_ENABLED;

extern QueueHandle_t radioQueue;
extern int command, option; 

extern Telemetry telemetry;
extern CJY901 IMU;
extern RadioClass Radio;
//extern PitotClass Pitot;
//extern GPSClass GPS;
extern CommandHandler commandHandler;

const uart_port_t RADIO_UART_NUM = UART_NUM_2;
const uart_port_t GPS_UART_NUM = UART_NUM_1;

//BAUD RATES
const uint32_t RADIO_BAUD_RATE = 9600;
const uint32_t GPS_BAUD_RATE = 115200;

//COMMUNICATION PROTOCOLS
const i2c_port_num_t IMU_I2C_PORT = I2C_NUM_1;
const i2c_port_num_t PITOT_I2C_PORT = I2C_NUM_0;
const uint16_t WT901B_I2C_ADDR = 0x50;
const uint32_t wt901b_i2c_scl_speed_hz = 100000;
const uint32_t sleep_time_ms = 1000;

/*Constants for time keeping in milliseconds*/
const uint32_t GPS_UPDATE_PERIOD = 100;
const uint32_t PITOT_UPDATE_PERIOD = 200;
const uint32_t IMU_UPDATE_PERIOD = 100;

esp_err_t radio_uart_setup();

void gps_uart_setup();

void init_IMU_i2c();

void init_Pitot_i2c();

void serial_buses_setup();