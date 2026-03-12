#include <SD.h>

// file config - this should be in main.ino
File dataFile;
const int chipSelect = 5;
char filename[13]; // 1 for '/', 8 for name, 1 for '.', 3 for ext, 1 for null terminator

// define logstate - this should be in main.ino
enum class LogState { IDLE, ACTIVE };
LogState logState = LogState::IDLE;

//====Function Declarations====

// "public" functions - use these
void setupSD();
void log();

// void updateLogging();
// void startLog(); 
// void stopLog();

// "private" functions - do NOT use these
void writeToSD();
void generateFile();
void generateFileName();



// Logging Parameters
unsigned long now = 0;
unsigned long lastCollection = 0;
const unsigned long LOGGING_PERIOD = 100; // 100 milliseconds / 10Hz

unsigned long lastWriteTime = 0;
unsigned long lastFlushTime = 0;
const unsigned long FLUSH_INTERVAL = 200;  // Flush SD every 300ms

// Test variables - used only to test the code written
unsigned int count = 0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  Serial.println("--- Booting ESP32 ---\n");

  setupSD();

} 

void loop() {
  // put your main code here, to run repeatedly:

  now = millis();

  if (now - lastCollection >= LOGGING_PERIOD) {

    //test code
    lastCollection = now;
    char lineToLog[32];
    snprintf(lineToLog, sizeof(lineToLog), "%lu,%u", now, count);
    
    
    log(lineToLog);
  
    count++;
  }

}

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

  if (logState != LogState::ACTIVE) return;
  writeToSD(line);
  Serial.print("Logged at time stamp:");
  Serial.println(now);
  
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

  // this is just test code, the method in which the file name
  // is generated will be updated later once GPS is running
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
                    "baro,"
                    "GPSlat,"
                    "GPSlong,"
                    "GPSalt,"
                    "GPSspeed,"
                    "pitot_static"
                    "pitot_dynamic"); 
    
    
    // Crucial: flush() forces the SD card to update its directory
    dataFile.flush(); 
    
    Serial.print("File created and header written: ");
    Serial.println(filename);
  } else {
    Serial.println("Failed to create file!");
  }

 }


void generateFileName() {
  unsigned long ts = millis() % 10000; // Get the last 4 digits of uptime
  int teamNum = 12; // Example team number

  // snprintf builds the string safely into the 'filename' array
  // Format: /T[Team][TS].CSV -> /T120042.CSV (Total 11 chars)
  snprintf(filename, sizeof(filename), "/T%02d%04lu.CSV", teamNum, ts);
}









