#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#define I2C_ADDRESS_MS4525 (0x28U)

#define MIN_PRESSURE_PSI   (-1.0f)
#define MAX_PRESSURE_PSI   (1.0f)
#define PRESSURE_SPAN_PSI  (2.0f)

#define OUTPUT_MIN         (0.10f)
#define OUTPUT_SPAN        (0.80f)

typedef struct {
    uint8_t raw[4];
    uint8_t status;
    uint16_t pressure_raw;
    uint16_t temp_raw;
    float pressure_pa;
    float temp_c;
    float speed_ms;
    //float speed_kmh;
    //float speed_kt;
    float offset;
} ms4525_data;

class AirspeedClass {
private:
    i2c_master_dev_handle_t sensor;

    esp_err_t readRaw(uint8_t raw_data[4]);

public:
    ms4525_data data;

    void setup(i2c_master_dev_handle_t sensor_handle);

    esp_err_t read();
    esp_err_t offset(uint16_t offset_loop_amount);
    void offsetAdd(float offset);
};

