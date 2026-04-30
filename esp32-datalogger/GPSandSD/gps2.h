#ifndef GPS2_H
#define GPS2_H

#include <TinyGPSPlus.h>

const int GPS_BAUD = 115200; 
const uint8_t MIN_SATS = 4;
const uint8_t RXD2 = 16, TXD2 = 17;
const unsigned long GPS_LOCK_TIMEOUT_MS = 60000;  // 60 seconds

void wiringCheck();
void GPSsetup();
void GPSLock();
void updateGPSData();
void printAll();
double getLat();
double getLong();
double getAlt();
float getSpeed();
uint8_t getSatCount();
uint8_t isGPSValid();
uint8_t getGPSmonth();
uint8_t getGPSday();
uint8_t getGPShour();
uint8_t getGPSminute();


#endif