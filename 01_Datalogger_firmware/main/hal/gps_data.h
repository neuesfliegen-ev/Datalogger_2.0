#pragma once

typedef struct {
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t millisecond;
} SGPSTime;

typedef struct {
    int32_t latitude;    // degrees * 1e7
    int32_t longitude;   // degrees * 1e7
    uint16_t speed;      // km/h * 100
    int32_t altitude;    // meters * 100
} SGPS;

typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
} SGPSDate;

typedef struct {
    SGPSTime time;
    SGPSDate date;
} SGPSDateTime;




/*NMEA sentences*/

// $GPGGA,hhmmss.sss,lat,N,lon,E,fix,sats,hdop,alt,M,...
// gives: time, latitude, longitude, altitude

// $GPRMC,hhmmss.sss,status,lat,N,lon,E,speed_knots,course,ddmmyy,...
// gives: time, date, latitude, longitude, ground speed

// $GPVTG,...,speed_knots,N,speed_kmh,K
// gives: ground speed only