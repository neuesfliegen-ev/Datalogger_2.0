#include <TinyGPSPlus.h>

extern TinyGPSPlus gps;

extern float latitude, longitude, gpsAltitude, Speed;  //&speed in m/s
extern uint8_t SatCount; 
const int GPS_BAUD = 115200; 
const uint8_t MIN_SATS = 4;
const uint8_t RXD2 = 16, TXD2 = 17;
const unsigned long GPS_LOCK_TIMEOUT_MS = 60000;  // 60 seconds

void wiringCheck();
void GPSsetup();
void GPSLock();
void updateGPSData();
void printAll();