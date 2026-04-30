#pragma once

constexpr int Radio_RX2 = 16; //Radio
constexpr int Radio_TX2 = 17;
constexpr int LORA_M0 = 33;//LoRa M0 pin
constexpr int LORA_M1 = 32;//LoRa M1 pin -- transmission mode = (0,0); configuration mode = (0,1) 
//constexpr uint8_t LORA_RATE = 9600;
//constexpr int8_t CHANNEL = 1;
//constexpr int8_t ADDRESS = {0x00, 0x01};

constexpr int NORMAL_MODE = 0; //Normal Mode (M0=0, M1=0)
constexpr int WOR_MODE = 1; //Wake On Radio Mode (M0=1, M1=0)
constexpr int CONFIG_MODE = 2; //Configuration Mode (M0=0, M1=1)
constexpr int SLEEP_MODE = 3; //Deep Sleep Mode (M0=1, M1=1)

static uint8_t DISPLAY_DATA_IN_TERMINAL = 1;

//int32_t sendDataLinetoDisplay();
void setupRadio(HardwareSerial*);
int32_t radio_poll(uint8_t, uint8_t);
int32_t set_radio_mode(uint8_t);
int32_t get_radio_mode();
int32_t listAllCmds();
