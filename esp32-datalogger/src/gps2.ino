#include <TinyGPSPlus.h>

// Create GPS parser
TinyGPSPlus gps;

float latitude = 0.0, longitude = 0.0, gpsAltitude = 0.0, Speed = 0.0;  //&speed in m/s
uint8_t SatCount = 0; 
const int GPS_BAUD = 115200; 
const uint8_t MIN_SATS = 4;
const uint8_t RXD2 = 16, TXD2 = 17;
const unsigned long GPS_LOCK_TIMEOUT_MS = 60000;  // 60 seconds

// "public" functions - use these
void updateGPSData();
void GPSLock();
void wiringCheck();

// "private" functions - do NOT use these
void printAll();

void setup() {
  Serial.begin(115200);
  Serial1.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);

  Serial.println("--- Booting ESP32 ---\n");
  //wiringCheck();
  GPSLock();  

}

void loop() {
  // put your main code here, to run repeatedly:
  updateGPSData();
  printAll();
}

void wiringCheck() {
  while (gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
    updateGPSData();
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

void printAll() {
  char gpsMsg[120];

  snprintf(gpsMsg, sizeof(gpsMsg), "Lat: %.6f, Long: %.6f, GPSAlt: %.2f, Speed: %.2f, SatCount: %u", latitude, longitude, gpsAltitude, Speed, SatCount);

  Serial.println(gpsMsg);
}