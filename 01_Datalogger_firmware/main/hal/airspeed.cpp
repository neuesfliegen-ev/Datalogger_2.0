#include "freertos/FreeRTOS.h"
#include "hal/airspeed.h"

#include <math.h>
#include <string.h>

#define PSI_TO_PA      (6894.76f)
#define AIR_DENSITY    (1.225f) //USES STANDARD ATMOSPHERIC PRESSURE AT 25C AIR DENSITY
#define FILTER_SIZE    (10U)

#define P_CNT          (16383.0f)
#define T_CNT          (2047.0f)

#define T_MIN          (-50.0f)
#define T_MAX          (150.0f)

#define MS4525_TIMEOUT_MS 100

static float pressureBuffer[FILTER_SIZE];
static uint8_t bufferIndex = 0;
static float pressureSum = 0.0f;

void AirspeedClass::setup(i2c_master_dev_handle_t sensor_handle) {
    sensor = sensor_handle;
}

esp_err_t AirspeedClass::readRaw(uint8_t raw_data[4]) {
    return i2c_master_receive(sensor, raw_data, 4, pdMS_TO_TICKS(MS4525_TIMEOUT_MS));
}

static float updatePressureFilter(float newPressurePa)
{
    pressureSum -= pressureBuffer[bufferIndex];
    pressureBuffer[bufferIndex] = newPressurePa;
    pressureSum += newPressurePa;

    bufferIndex++;
    if (bufferIndex >= FILTER_SIZE) {
        bufferIndex = 0;
    }

    return pressureSum / FILTER_SIZE;
}

esp_err_t AirspeedClass::read() {
    uint8_t raw_data[4];

    esp_err_t err = readRaw(raw_data);

    if (err != ESP_OK) {
        return err;
    }

    memcpy(data.raw, raw_data, 4);

    data.status = (raw_data[0] >> 6) & 0x03;
    data.pressure_raw = ((uint16_t)(raw_data[0] & 0x3F) << 8) | raw_data[1];
    data.temp_raw = ((uint16_t)raw_data[2] << 3) | ((raw_data[3] >> 5) & 0x07);

    switch(data.status)
    {
        case 0:
            break;

        case 1:
            return ESP_FAIL;

        case 2:
            return ESP_ERR_INVALID_STATE;

        case 3:
            return ESP_FAIL;
    }

    float pressure_psi = ((data.pressure_raw - (OUTPUT_MIN * P_CNT)) *
                         (PRESSURE_SPAN_PSI / (OUTPUT_SPAN * P_CNT))) + MIN_PRESSURE_PSI;

    data.temp_c = ((float)data.temp_raw * (T_MAX - T_MIN) / T_CNT) + T_MIN;

    data.pressure_pa = updatePressureFilter((pressure_psi * PSI_TO_PA) - data.offset);

    if (data.pressure_pa > 0.0f) {
        data.speed_ms = sqrtf((2.0f * data.pressure_pa) / AIR_DENSITY);
    } else {
        data.speed_ms = 0.0f;
    }

    //data.speed_kmh = data.speed_ms * 3.6f;
    //data.speed_kt = data.speed_ms * 1.94384f;

    return ESP_OK;
}

esp_err_t AirspeedClass::offset(uint16_t offset_loop_amount) {
    float offset_sum = 0.0f;
    esp_err_t err;

    if (offset_loop_amount == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    for(uint16_t i = 0; i < offset_loop_amount; i++) {
        err = read();

        if(err != ESP_OK) {
            return err;
        }

        offset_sum += data.pressure_pa;
    }

    data.offset = offset_sum / (float)offset_loop_amount;

    return ESP_OK;
}

void AirspeedClass::offsetAdd(float offset) {
    data.offset = offset;
}