#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "unity.h"
#include <cstring>
#include <cstdio>
#include "pins.h"


namespace
{

    constexpr uart_port_t GPS_UART_PORT = UART_NUM_1;
    constexpr uint32_t GPS_BAUD_RATE = 115200;
    constexpr int GPS_RX_BUFFER_SIZE = 2048;
    constexpr int GPS_TEST_TIMEOUT_MS = 3000;

}

TEST_CASE("[GPS] UART data diagnostic", "[gps]")
{
    /* Step 1: Does the GPS UART initialize successfully? */
     uart_config_t uart_config{};

    uart_config.baud_rate = GPS_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    // Install the UART driver with an RX buffer.
    esp_err_t result = uart_driver_install(
        GPS_UART_PORT,
        GPS_RX_BUFFER_SIZE,
        0,
        0,
        nullptr,
        0);

    if (result != ESP_OK)
    {
        TEST_FAIL_MESSAGE(
            "Failed to install the GPS UART driver.");
    }

    // Apply the UART communication settings.
    result = uart_param_config(
        GPS_UART_PORT,
        &uart_config);

    if (result != ESP_OK)
    {
        uart_driver_delete(GPS_UART_PORT);

        TEST_FAIL_MESSAGE(
            "Failed to configure GPS UART parameters.");
    }

    // Connect the UART controller to the configured GPS GPIO pins.
    result = uart_set_pin(
        GPS_UART_PORT,
        GPS_TX_PIN,
        GPS_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE);

    if (result != ESP_OK)
    {
        uart_driver_delete(GPS_UART_PORT);

        TEST_FAIL_MESSAGE(
            "Failed to configure GPS UART pins.");
    }

    TEST_MESSAGE("[GPS] UART initialization passed.");

    /* Step 2: Receive GPS data through UART. */

    uart_flush_input(GPS_UART_PORT);

    char received_data[512]{};

    const int received_length = uart_read_bytes(
        GPS_UART_PORT,
        received_data,
        sizeof(received_data) - 1,
        pdMS_TO_TICKS(GPS_TEST_TIMEOUT_MS));

    if (received_length <= 0)
    {
        uart_driver_delete(GPS_UART_PORT);

        TEST_FAIL_MESSAGE(
            "[GPS] No UART data received. Check power, wiring, "
            "RX pin, and baud rate.");
    }

    // Add a string terminator after the received data.
    received_data[received_length] = '\0';

    printf(
        "\n[GPS] Received %d bytes:\n%s\n",
        received_length,
        received_data);

    /* Step 3: Check for a supported NMEA sentence. */

    const bool has_nmea_start =
        std::strchr(received_data, '$') != nullptr;

    const bool has_supported_sentence =
        std::strstr(received_data, "GGA") != nullptr ||
        std::strstr(received_data, "RMC") != nullptr ||
        std::strstr(received_data, "VTG") != nullptr;

    if (!has_nmea_start || !has_supported_sentence)
    {
        uart_driver_delete(GPS_UART_PORT);

        TEST_FAIL_MESSAGE(
            "[GPS] UART data was received, but no supported "
            "NMEA sentence was found.");
    }

    TEST_MESSAGE("[GPS] NMEA sentence validation passed.");
    TEST_MESSAGE(
        "[GPS] Manually verify the latitude, longitude, and fix status "
        "in the NMEA output against the board's current location.");

    /* Step 4: Release the UART resources. */

    uart_driver_delete(GPS_UART_PORT);

    TEST_MESSAGE("[GPS] UART cleanup passed.");
}