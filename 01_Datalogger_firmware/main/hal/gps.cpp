#include <cstring>
#include <cstdlib>
#include "freertos/FreeRTOS.h"

#include "driver/uart.h"
#include "hal/gps.h"


void GPSClass::startUART(uart_port_t p){
	uart_num = p;
}

void GPSClass::update(){
    uint8_t c;

    while (uart_read_bytes(uart_num, &c, 1, 10 / portTICK_PERIOD_MS) > 0) {
        if (c == '\n') {
            line_[index_] = '\0';

            if (index_ > 6 && line_[0] == '$') {
                parseLine(line_);
            }

            index_ = 0;
        } else if (c != '\r') {
            if (index_ < sizeof(line_) - 1) {
                line_[index_++] = static_cast<char>(c);
            } else {
                index_ = 0;
            }
        }
    }
}

void GPSClass::parseLine(char* line) {
    if (std::strstr(line, "GGA")) {
        parseGGA(line);
    } else if (std::strstr(line, "RMC")) {
        parseRMC(line);
    } else if (std::strstr(line, "VTG")) {
        parseVTG(line);
    }
}

void GPSClass::parseGGA(char* line)
{
    char* f[16] = {};
    splitNMEA(line, f, 16);

    parseTime(f[1], dateTime_.time);

    if (f[2] && f[3])
        gps_.latitude = parseCoord(f[2], f[3][0]);

    if (f[4] && f[5])
        gps_.longitude = parseCoord(f[4], f[5][0]);

    if (f[9])
        gps_.altitude = parseAltitude(f[9]);
}

void GPSClass::parseRMC(char* line)
{
    char* f[16] = {};
    splitNMEA(line, f, 16);

    if (!f[2] || f[2][0] != 'A') return;

    parseTime(f[1], dateTime_.time);

    if (f[3] && f[4])
        gps_.latitude = parseCoord(f[3], f[4][0]);

    if (f[5] && f[6])
        gps_.longitude = parseCoord(f[5], f[6][0]);

    if (f[7])
        gps_.speed = parseSpeedKnots(f[7]);

    if (f[9])
        parseDate(f[9], dateTime_.date);
}

void GPSClass::parseVTG(char* line)
{
    char* f[16] = {};
    splitNMEA(line, f, 16);

    if (f[7])
        gps_.speed = static_cast<uint16_t>(std::atof(f[7]) * 100.0);
}

int GPSClass::splitNMEA(char* line, char* fields[], int maxFields)
{
    int count = 0;

    char* star = std::strchr(line, '*');
    if (star) *star = '\0';

    char* p = line;

    while (p && count < maxFields) {
        fields[count++] = p;

        p = std::strchr(p, ',');
        if (p) {
            *p = '\0';
            p++;
        }
    }

    return count;
}

int32_t GPSClass::parseCoord(const char* s, char hemi)
{
    if (!s || !*s) return 0;

    double raw = std::atof(s);
    int deg = static_cast<int>(raw / 100);
    double min = raw - deg * 100;
    double dec = deg + min / 60.0;

    if (hemi == 'S' || hemi == 'W') dec = -dec;

    return static_cast<int32_t>(dec * 10000000.0);
}

int32_t GPSClass::parseAltitude(const char* s)
{
    if (!s || !*s) return 0;
    return static_cast<int32_t>(std::atof(s) * 100.0);
}

uint16_t GPSClass::parseSpeedKnots(const char* s)
{
    if (!s || !*s) return 0;

    double knots = std::atof(s);
    double kmh = knots * 1.852;

    return static_cast<uint16_t>(kmh * 100.0);
}

void GPSClass::parseTime(const char* s, SGPSTime& time)
{
    if (!s || !*s) return;

    double val = std::atof(s);
    uint32_t whole = static_cast<uint32_t>(val);

    time.hour = whole / 10000;
    time.minute = (whole / 100) % 100;
    time.second = whole % 100;
    time.millisecond = static_cast<uint16_t>((val - whole) * 1000.0);
}

void GPSClass::parseDate(const char* s, SGPSDate& date)
{
    if (!s || !*s) return;

    uint32_t val = static_cast<uint32_t>(std::atoi(s));

    date.day = val / 10000;
    date.month = (val / 100) % 100;
    date.year = 2000 + (val % 100);
}