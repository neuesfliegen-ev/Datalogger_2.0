#include "sd_read2.h"
#include "gps2.h"

// file config
File dataFile;
const int chipSelect = 5;
char filename[13]; // 1 for '/', 8 for name, 1 for '.', 3 for ext, 1 for null terminator

// define logstate
LogState logState = LogState::IDLE;

// Logging Parameters
unsigned long now = 0;
unsigned long lastCollection = 0;

unsigned long lastWriteTime = 0;
unsigned long lastFlushTime = 0;

// Create GPS parser
TinyGPSPlus gps;

float latitude = 0.0, longitude = 0.0, gpsAltitude = 0.0, Speed = 0.0;  //&speed in m/s
uint8_t SatCount = 0;

// Test variables - used only to test the code written
unsigned int count = 0;

void setup() {

  Serial.begin(115200);
  Serial.println("--- Booting ESP32 ---\n");
  GPSsetup();
  setupSD();

}

void loop() {
  // put your main code here, to run repeatedly:
  char lineToLog[170];
  updateGPSData();
  printAll();
  snprintf(lineToLog, sizeof(lineToLog), 
  "%lu,"
  "n/a,n/a,n/a," // Accel X,Y,Z
  "n/a,n/a,n/a," // Gyro X,Y,Z
  "n/a,n/a,n/a," // Mag X,Y,Z
  "n/a,n/a,n/a,n/a," // Roll, pitch, yaw, baro
  "%.6f,%.6f," // GPS lat & long
  "%.2f,%.2f," // GPS alt & speed
  "%u", now, latitude, longitude, gpsAltitude, Speed, SatCount);  
  log(lineToLog);
    
}
