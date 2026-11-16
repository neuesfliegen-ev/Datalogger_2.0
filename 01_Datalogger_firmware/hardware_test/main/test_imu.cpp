#include <cmath>
#include "driver/i2c_master.h"
#include "unity.h"
#include "hal/imu.h"
#include "pins.h"

namespace
{

    constexpr i2c_port_num_t IMU_I2C_PORT = I2C_NUM_1;
    constexpr uint16_t IMU_ADDRESS = 0x50;
    constexpr uint32_t IMU_CLOCK_HZ = 100000;
    constexpr int PROBE_TIMEOUT_MS = 100;

}

/*
    TEST 1: IMU responds and acceleration registers can be read
*/

TEST_CASE(
    "IMU responds and acceleration registers can be read",
    "[imu]")
{
    /* --- Configure the IMU I2C master bus --- */
    // Configure the ESP32 I2C master bus used by the IMU.
    i2c_master_bus_config_t bus_config{};

    bus_config.i2c_port = IMU_I2C_PORT;
    bus_config.sda_io_num = IMU_SDA_PIN;
    bus_config.scl_io_num = IMU_SCL_PIN;
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = true;

    // Create the I2C bus and store its handle in 'bus_handle'.
    i2c_master_bus_handle_t bus_handle = nullptr;

    esp_err_t result = i2c_new_master_bus(
        &bus_config,
        &bus_handle);

    // Stop the test if the I2C bus could not be created.
    TEST_ASSERT_EQUAL(ESP_OK, result);

    /* --- Check IMU Address ACK Response --- */
    // Check whether the IMU responds at its configured I2C address.
    result = i2c_master_probe(
        bus_handle,
        IMU_ADDRESS,
        PROBE_TIMEOUT_MS);

    // Release the I2C bus before failing the test.
    if (result != ESP_OK)
    {
        // Release the I2C bus before failing the test.
        i2c_del_master_bus(bus_handle);

        TEST_FAIL_MESSAGE(
            "IMU did not respond at address 0x50. "
            "Check power, GND, SDA GPIO_21, SCL GPIO_22, "
            "I2C address, and pull-up resistors.");
    }

    /* --- Configure IMU Device --- */
    // Configure the IMU as a device on the I2C bus.
    i2c_device_config_t device_config{};

    device_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    device_config.device_address = IMU_ADDRESS;
    device_config.scl_speed_hz = IMU_CLOCK_HZ;

    // Add the IMU device to the bus and store its handle.
    i2c_master_dev_handle_t device_handle = nullptr;

    result = i2c_master_bus_add_device(
        bus_handle,
        &device_config,
        &device_handle);

    // Release the bus before failing if device registration fails.
    if (result != ESP_OK)
    {
        i2c_del_master_bus(bus_handle);
        TEST_FAIL_MESSAGE(
            "Failed to add the IMU device to the I2C bus");
    }

    /* --- Read from IMU Device --- */
    // Connect the existing IMU driver to the I2C device.
    CJY901 imu;
    imu.StartIIC(device_handle);

    // Read the acceleration registers from the IMU.
    result = imu.updateAcc();

    if (result != ESP_OK)
    {
        i2c_master_bus_rm_device(device_handle);
        i2c_del_master_bus(bus_handle);

        TEST_FAIL_MESSAGE(
            "Failed to read IMU acceleration registers. "
            "Check IMU register communication and driver configuration.");
    }


    // Release the I2C resources after the test completes.
    i2c_master_bus_rm_device(device_handle);
    i2c_del_master_bus(bus_handle);
}
