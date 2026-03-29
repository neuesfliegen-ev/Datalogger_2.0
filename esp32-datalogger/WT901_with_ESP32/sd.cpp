#include <SD.h>
#include "sd_read.h"
#include "gps.h"
#include "telemetry.h"

//====Function Declarations====
// to be added
// void updateLogging();
// void startLog(); 
// void stopLog();

Telemetry* t = nullptr;

void writeToSD(const char* line);
void generateFile();
void generateFileName();

void setupSD() {
  if (!SD.begin(chipSelect)) {
    Serial.print(F("SD card missing\n"));
    logState = LogState::IDLE;
    while(1) {
      delay(10);
    };
  }
  generateFile();
  logState = LogState::ACTIVE;

  Serial.println(F("SD card initialised successfully.\n"));
}

void log(const char* line) {
  // this is a wrapper function for writeToSD()
  // use this in main
  now = millis();

  if (logState != LogState::ACTIVE) return;

  if (now - lastCollection >= LOGGING_PERIOD) {
    lastCollection = now;
    writeToSD(line);
    Serial.print("Logged at time stamp:");
    Serial.println(now);
  }
  
}

void writeToSD(const char* line) {
  // do not call this function directly!
  // call log() instead!!

  if (dataFile) {
    dataFile.println(line);
  }

  lastWriteTime = millis();

  // Flush every 300ms
  if (millis() - lastFlushTime >= FLUSH_INTERVAL) {
    if (dataFile) {
      dataFile.flush();
      Serial.println(">>> DATA SAFELY SAVED TO CARD <<<");
    }
    lastFlushTime = millis();
  }
}

void generateFile() {

  generateFileName();
  dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    // Write the CSV header so the file isn't 0 bytes
    dataFile.println("Timestamp,"
                    "accX,"
                    "accY,"
                    "accZ,"
                    "gyroX,"
                    "gyroY,"
                    "gyroZ,"
                    "magX,"
                    "magY,"
                    "magZ,"
                    "roll,"
                    "pitch,"
                    "yaw,"
                    "baro,"
                    "GPSlat,"
                    "GPSlong,"
                    "GPSalt,"
                    "GPSspeed,"
                    "sat_count,"
                    "GPS_valid,"
                    "pitot_static,"
                    "pitot_dynamic"
                    ); 
    
    
    // Crucial: flush() forces the SD card to update its directory
    dataFile.flush(); 
    
    Serial.print("File created and header written: ");
    Serial.println(filename);
  } else {
    Serial.println("Failed to create file!");
  }

 }

void generateFileName() { //will be updated to use GPS date and time to generate filename

  // snprintf builds the string safely into the 'filename' array
  // Format: /T[Team][TS].CSV -> /T120042.CSV (Total 11 chars)
  snprintf(filename, sizeof(filename), "/%02d%02d%02d%02d.CSV", getGPSmonth(), getGPSday(), getGPShour(), getGPSminute());
}
