#ifndef AIRSPEED_DATA_H
#define AIRSPEED_DATA_H

struct SAirPressure
{
    // Raw pressure counts from sensor
    unsigned short rawPressure;

    // Differential pressure in Pascal
    float pressurePa;

};

struct SAirTemp
{
    // Raw temperature counts
    unsigned short rawTemp;

    // Converted temperature
    float tempC;
};

struct SAirSpeed
{
    // Calculated airspeed
    float speedMS;     // meters/sec

    // Optional
    float speedKPH;    // km/h
};

struct SAirDensity
{
    // Air density (optional)
    float rho;
};

struct SAirStatus
{
    bool sensorOK;
    bool dataReady;
    bool communicationError;
};

struct AirspeedData
{
    SAirPressure pressure;
    SAirTemp temperature;
    SAirSpeed speed;
    SAirDensity density;
    SAirStatus status;
};

#endif

