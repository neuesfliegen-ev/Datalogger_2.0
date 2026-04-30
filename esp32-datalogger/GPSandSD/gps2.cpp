#include <TinyGPSPlus.h>
#include <math.h>
#include "gps2.h"

TinyGPSPlus gps;

double latitude = 0, longitude = 0, gpsAltitude = 0;
float Speed = 0;
uint8_t SatCount = 0;
uint8_t GPS_valid = 0;

/*
void wiringCheck() {
  while (gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
    updateGPSData();
  }
}
*/

void GPSsetup() {
  Serial1.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  GPSLock();
}

void GPSLock() {
  Serial.println("Beginning GPSLock");
  
  unsigned long startTime = millis();
  bool GPSLockState = false;

  while (millis() - startTime < GPS_LOCK_TIMEOUT_MS) {
    updateGPSData();
      

    // Display satellite count
    char satMsg[30];
    snprintf(satMsg, sizeof(satMsg), "Satellites: %d/%d", SatCount, MIN_SATS);
    Serial.println(satMsg);

    if (SatCount >= MIN_SATS && gps.date.isValid() && gps.time.isValid()) {
      GPSLockState = true;
      updateGPSData();
      break;  // Good GPS fix, break early
    }

  }

  if (!GPSLockState) {
  // Timeout reached without lock
    Serial.print("Could not lock minimum satellites. Current Satellites: ");
    Serial.println(SatCount);

    char errMsg1[] = "GPS Lock Failed!";
    char errMsg2[30];
    snprintf(errMsg2, sizeof(errMsg2), "Satellites: %d/%d", SatCount, MIN_SATS);

    Serial.println(errMsg1);
    Serial.println(errMsg2);


    delay(5000);
  } else {
    Serial.println("GPS lock acquired!");
    delay(2000);
  }

}

void updateGPSData() {
  while (Serial1.available()) {
    // Feed to TinyGPS parser
    gps.encode(Serial1.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
      latitude  = gps.location.lat();
      longitude = gps.location.lng();
  }
  if (gps.altitude.isUpdated() && gps.altitude.isValid()) {
      gpsAltitude = gps.altitude.meters();
  }
  if (gps.speed.isUpdated() && gps.speed.isValid()) {
      Speed = gps.speed.mps();
  }
  if (gps.satellites.isUpdated() && gps.satellites.isValid()) {
      SatCount = gps.satellites.value();
  }

  if (gps.location.isValid() && gps.altitude.isValid() && gps.speed.isValid() && gps.satellites.value() >= MIN_SATS) {
    GPS_valid = 1;
  } else{
    GPS_valid = 0;
  }
  
  

}

void printAll() { // only for serial print - won't be used later
  char gpsMsg[120];

  snprintf(gpsMsg, sizeof(gpsMsg), "Lat: %.6f, Long: %.6f, GPSAlt: %.2f, Speed: %.2f, SatCount: %u", latitude, longitude, gpsAltitude, Speed, SatCount);

  Serial.println(gpsMsg);
}

double getLat() {
  return round(latitude * 1e6) / 1e6;
}

double getLong() {
  return roundf(longitude * 1e6) / 1e6;
}

double getAlt() {
  return roundf(gpsAltitude * 1e2) / 1e2;
}

float getSpeed() {
  return roundf(Speed * 1e2) / 1e2;
}

uint8_t getSatCount() {
  return SatCount;
}

uint8_t isGPSValid() {
  // Returns true only if location, altitude, and speed are all currently valid
  return GPS_valid;

}

uint8_t getGPSmonth() {
  return gps.date.month();
}

uint8_t getGPSday() {
  return gps.date.day();
}

uint8_t getGPShour() {
  return gps.time.hour();
}

uint8_t getGPSminute() {
  return gps.time.minute();
}