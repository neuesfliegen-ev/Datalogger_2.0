#pragma once

#include "driver/uart.h"

#include "hal/gps_data.h"

class GPSClass {
public:
	GPSClass(){};
	void startUART(uart_port_t p);

	void update();

	SGPS gps_;
	SGPSDateTime dateTime_;

private: 
	uart_port_t uart_num;

    char line_[128]{};
    uint16_t index_ = 0;

    void parseLine(char* line);
    void parseGGA(char* line);
    void parseRMC(char* line);
    void parseVTG(char* line);

    static int splitNMEA(char* line, char* fields[], int maxFields);
    static int32_t parseCoord(const char* s, char hemi);
    static int32_t parseAltitude(const char* s);
    static uint16_t parseSpeedKnots(const char* s);
    static void parseTime(const char* s, SGPSTime& time);
    static void parseDate(const char* s, SGPSDate& date);

};