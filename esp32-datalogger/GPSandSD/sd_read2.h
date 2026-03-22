#ifndef SD_READ2_H
#define SD_READ2_H

#include <SD.h>

// Logging Parameters
extern unsigned long now;
extern unsigned long lastCollection;
const unsigned long LOGGING_PERIOD = 100; // 100 milliseconds / 10Hz

extern unsigned long lastWriteTime;
extern unsigned long lastFlushTime;
const unsigned long FLUSH_INTERVAL = 200;  // Flush SD every 300ms

enum class LogState { IDLE, ACTIVE };

extern File dataFile;
extern const int chipSelect;
extern char filename[13]; // 1 for '/', 8 for name, 1 for '.', 3 for ext, 1 for null terminator
extern LogState logState;

void setupSD();
void log(const char* line);

#endif