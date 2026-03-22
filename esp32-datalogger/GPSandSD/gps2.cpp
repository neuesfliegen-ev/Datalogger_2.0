#include <TinyGPSPlus.h>
#include "gps2.h"

void wiringCheck();
void GPSsetup();
void GPSLock();
void updateGPSData();
void printAll();

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
      char c = Serial1.read();

      // Feed to TinyGPS parser
      gps.encode(c);

      if (gps.location.isUpdated()) {
      latitude     = gps.location.lat();
      longitude    = gps.location.lng();
      gpsAltitude  = gps.altitude.meters();
      Speed        = gps.speed.mps();
      SatCount     = gps.satellites.value();
      }
  }

}

void printAll() { // only for serial print - won't be used later
  char gpsMsg[120];

  snprintf(gpsMsg, sizeof(gpsMsg), "Lat: %.6f, Long: %.6f, GPSAlt: %.2f, Speed: %.2f, SatCount: %u", latitude, longitude, gpsAltitude, Speed, SatCount);

  Serial.println(gpsMsg);
}