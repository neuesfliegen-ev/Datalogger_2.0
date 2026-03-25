#include <Arduino.h>
#include <stdint.h>
#include "radio_com.h"
#include "telemetry.h"
#include "wit_myFunctions.h"
#include "wit_c_sdk.h"

//static uint8_t RADIO_MODE = 0;
static uint8_t radioWriteBuffer[256];

int32_t radio_poll(uint8_t cmd, uint8_t arg, HardwareSerial* Radio){
  switch(cmd){
    case 0: listAllCmds(Radio); Serial.println("print all cmds on radio. e.g."); break;
    //case 1: startFlight(); break;
    //case 2: endFlight(); break;
    case 3: Radio->print("I: Calibrating everything\n"); wit_calibrate_all(); Radio->print("I: Calibrating done\n"); break;
    case 4: Radio->print("I: Calibrating acc\n"); wit_calibrate_acc(); Radio->print("I: Calibrating done\n"); break;
    case 5: Radio->print("I: Calibrating mag\n"); wit_calibrate_mag(); Radio->print("I: Calibrating done\n"); break;
    case 8: Radio->print("I: Now displaying data in terminal\n"); DISPLAY_DATA_IN_TERMINAL = 1; break;
    case 9: Radio->print("Stopping data display in terminal\n"); DISPLAY_DATA_IN_TERMINAL = 0; break;
    //case 10: Radio->write(dataline.buf, dataline.len); break;
    case 11: set_radio_mode(arg, Radio); break;
    case 12: get_radio_mode(Radio); break;
    default: Radio->println("Not acceptable cmd: no spaces between cmd and arg"); return -1;
  }
  return 0;
}

int32_t set_radio_mode(uint8_t arg, HardwareSerial* Radio){
  switch(arg){
    case 0x00: digitalWrite(LORA_M0, LOW); digitalWrite(LORA_M1, LOW); Radio->println("Radio set to transmission mode"); break;
    case 0x01: digitalWrite(LORA_M0, HIGH); digitalWrite(LORA_M1, LOW); Radio->println("Radio set to configuration mode"); break;
    case 0x02: digitalWrite(LORA_M0, LOW); digitalWrite(LORA_M1, HIGH); Radio->println("Radio set to configuration mode"); break;
    case 0x03: /*digitalWrite(LORA_M0, HIGH); digitalWrite(LORA_M1, HIGH);*/Radio->println("Don't set Radio to deep sleep"); break;
    default: Serial.println("invalid radio mode: print cmd and arg without spaces"); return -1;
  }
  return 1;
}

int32_t get_radio_mode(HardwareSerial* Radio){
  int mode = digitalRead(LORA_M0) + digitalRead(LORA_M1)*2;
  return mode;
}

int32_t listAllCmds(HardwareSerial* Radio){//fix
  uint8_t buf[128] = {'l','i','s','t','\r','\n'};
  Radio->write(buf, 6);
  return 1;
}


/*
int32_t write_to_radio(HardwareSerial* Radio){

  return 1;
}

int32_t get_radio_address(HardwareSerial* Radio){
  set_radio_mode(CONFIG_MODE, Radio);
  Serial.print("set mode to: "); Serial.println(get_radio_mode(Radio));
  radioWriteBuffer[] = {'h','e','r','e','\n'};
  Radio->write(radioWriteBuffer, 5);
  set_radio_mode(NORMAL_MODE, Radio);
  return mode;
}
*/

