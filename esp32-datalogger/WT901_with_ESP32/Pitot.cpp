#include <Arduino.h>
#include <Wire.h>
#include "Pitot.h"

#define I2C_ADDR 0x28

// Sensor transfer function limits
#define OUTPUT_MIN 1638.0
#define OUTPUT_MAX 14745.0

// 1 PSI = 6894.76 Pa
#define P_MIN (-6894.76)
#define P_MAX ( 6894.76)

#define RHO 1.225 // Air density at sea level

float pressOffset = 0.0;
float diffPressFiltered = 0.0;

const float alpha = 0.2;   // filter strength

uint16_t read_dp_counts() {
    uint8_t buf[4];

    Wire.beginTransmission(I2C_ADDR);
    Wire.endTransmission();
    Wire.requestFrom(I2C_ADDR, 4);

    if (Wire.available() < 4) { // not enough data
        return 0;
    }

    for (int i = 0; i < 4; i++) {
        buf[i] = Wire.read();
    }

    uint16_t dp_counts = ((buf[0] & 0x3F) << 8) | buf[1];
    return dp_counts;
}

float counts_to_pa(uint16_t rawCounts) {
    float diffPress = ((rawCounts - OUTPUT_MIN) / (OUTPUT_MAX - OUTPUT_MIN)) * (P_MAX - P_MIN) + P_MIN;

    return diffPress;  // in Pascals
}

void calibrate_offset() {
    const int N = 100;
    float sum = 0;

    for (int i = 0; i < N; i++) {
        uint16_t c = read_dp_counts();
        float diffPress = counts_to_pa(c);
        sum += diffPress;
        delay(10);
    }

    pressOffset = sum / N;
}

float compute_indicatedAirspeed(float dp_pa) {
    if (dp_pa < 0) dp_pa = 0;

    float v = sqrt((2.0 * dp_pa) / RHO);
    return v;  // in m/s
}

float update_airspeed() {
    uint16_t rawCounts = read_dp_counts();

    if (rawCounts == 0) {
        return 0;  // or return last valid airspeed
    }

    float diffPress = counts_to_pa(rawCounts);

    float diffPressCalibrated = diffPress - pressOffset;

    diffPressFiltered = alpha * diffPressCalibrated + (1 - alpha) * diffPressFiltered;

    if (diffPressFiltered < 0) diffPressFiltered = 0;

    float indicatedAirspeed = compute_indicatedAirspeed(diffPressFiltered);

    return indicatedAirspeed; // in m/s
}