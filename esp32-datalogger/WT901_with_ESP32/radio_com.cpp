#include <Arduino.h>
#include <stdint.h>
#include "radio_com.h"
#include "telemetry.h"
#include "wit_myFunctions.h"
#include "wit_c_sdk.h"

//static uint8_t RADIO_MODE = 0;
HardwareSerial* p_Radio = nullptr;
static uint8_t radioWriteBuffer[256];

void setupRadio(HardwareSerial* ptr){
  p_Radio = ptr;
  set_radio_mode(0);
}

int32_t radio_poll(uint8_t cmd, uint8_t arg){
  switch(cmd){
    case 0: listAllCmds(); Serial.println("print all cmds on radio. e.g."); break;
    //case 1: startFlight(); break;
    //case 2: endFlight(); break;
    case 3: p_Radio->print("I: Calibrating everything\n"); wit_calibrate_all(); p_Radio->print("I: Calibrating done\n"); break;
    case 4: p_Radio->print("I: Now displaying data in terminal\n"); DISPLAY_DATA_IN_TERMINAL = 1; break;
    case 5: p_Radio->print("Stopping data display in terminal\n"); DISPLAY_DATA_IN_TERMINAL = 0; break;
    //case 6: p_Radio->write(dataline.buf, dataline.len); break;
    case 7: set_radio_mode(arg); break;
    case 8: get_radio_mode(); break;
    default: p_Radio->println("Not acceptable cmd: no spaces between cmd and arg"); return -1;
  }
  return 0;
}

int32_t set_radio_mode(uint8_t arg){
  switch(arg){
    case 0: digitalWrite(LORA_M0, LOW); digitalWrite(LORA_M1, LOW); p_Radio->println("Radio set to transmission mode"); break;
    case 1: digitalWrite(LORA_M0, HIGH); digitalWrite(LORA_M1, LOW); p_Radio->println("Radio set to WoR mode"); break;
    case 2: digitalWrite(LORA_M0, LOW); digitalWrite(LORA_M1, HIGH); p_Radio->println("Radio set to configuration mode"); break;
    case 3: /*digitalWrite(LORA_M0, HIGH); digitalWrite(LORA_M1, HIGH);*/p_Radio->println("Don't set Radio to deep sleep"); break;
    default: Serial.println("invalid radio mode: print cmd and arg without spaces"); return -1;
  }
  return 1;
}

int32_t get_radio_mode(){
  int mode = digitalRead(LORA_M0) + digitalRead(LORA_M1)*2;
  return mode;
}

int32_t listAllCmds(){//fix
  uint8_t buf[128] = {'l','i','s','t'};
  p_Radio->write(buf, 4);
  return 1;
}


