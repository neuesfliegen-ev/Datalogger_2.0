/* C Libraries */
#include <stdio.h>
#include <string.h>

/* ESP-IDF */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

/* Local */
#include "pins.h"
#include "hal/imu.h"
#include "hal/radio.h"
#include "modules/telemetry.h"
#include "modules/commandHandler.h"
#include "setup.h"

esp_err_t radio_uart_setup(){
	const int uart_buffer_size = (1024 * 2);	// Setup UART buffered IO with event queue
	QueueHandle_t uart_queue;	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(RADIO_UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
			ESP_LOGI("radio", "install radio uart driver", "1");
	uart_config_t uart_config{};   // zero-initialize everything
		uart_config.baud_rate = RADIO_BAUD_RATE;
		uart_config.data_bits = UART_DATA_8_BITS;
		uart_config.parity    = UART_PARITY_DISABLE;
		uart_config.stop_bits = UART_STOP_BITS_1;
		uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uart_config.source_clk = UART_SCLK_DEFAULT;	
	ESP_ERROR_CHECK(uart_param_config(RADIO_UART_NUM, &uart_config));
		ESP_LOGI("radio", "config radio uart");
	ESP_ERROR_CHECK(uart_set_pin(RADIO_UART_NUM, RADIO_TX_PIN, RADIO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
		ESP_LOGI("radio", "set radio pins");

	//gpio_set_pull_mode(RADIO_TX_PIN, GPIO_PULLUP_ONLY);
	Radio.startUART(RADIO_UART_NUM);
		ESP_LOGI("radio","RadioClass start");
	return ESP_OK;
}

void gps_uart_setup(){
	const int uart_buffer_size = (1024 * 2);	// Setup UART buffered IO with event queue
	QueueHandle_t uart_queue;	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
		ESP_LOGI("gps", "gps uart install");

	uart_config_t uart_config{};   // zero-initialize everything
		uart_config.baud_rate = GPS_BAUD_RATE;
		uart_config.data_bits = UART_DATA_8_BITS;
		uart_config.parity    = UART_PARITY_DISABLE;
		uart_config.stop_bits = UART_STOP_BITS_1;
		uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uart_config.source_clk = UART_SCLK_DEFAULT;	
	ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
		ESP_LOGI("gps", "gps param config");

	ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
		ESP_LOGI("gps", "gps pin set");

	GPS.startUART(GPS_UART_NUM);	
}

void init_IMU_i2c(){
	i2c_master_bus_config_t imu_bus_config = {};
		imu_bus_config.i2c_port = IMU_I2C_PORT;
		imu_bus_config.sda_io_num = IMU_SDA_PIN;
		imu_bus_config.scl_io_num = IMU_SCL_PIN;
		imu_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
		imu_bus_config.glitch_ignore_cnt = 7;
		imu_bus_config.intr_priority = 0;
		imu_bus_config.trans_queue_depth = 0;
		imu_bus_config.flags.enable_internal_pullup = true;
	
	i2c_master_bus_handle_t imu_bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&imu_bus_config, &imu_bus_handle));

	i2c_device_config_t imu_dev_cfg = {};
	    imu_dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    	imu_dev_cfg.device_address = WT901B_I2C_ADDR;
    	imu_dev_cfg.scl_speed_hz = 100000;

	i2c_master_dev_handle_t imu_dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(imu_bus_handle, &imu_dev_cfg, &imu_dev_handle));
	IMU.StartIIC(imu_dev_handle);
}

void init_Airspeed_i2c(){
	i2c_master_bus_config_t airspeed_bus_config = {};
		airspeed_bus_config.i2c_port = AIRSPEED_I2C_PORT;
		airspeed_bus_config.sda_io_num = AIRSPEED_SDA_PIN;
		airspeed_bus_config.scl_io_num = AIRSPEED_SCL_PIN;
		airspeed_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
		airspeed_bus_config.glitch_ignore_cnt = 7;
		airspeed_bus_config.intr_priority = 0;
		airspeed_bus_config.trans_queue_depth = 0;
		airspeed_bus_config.flags.enable_internal_pullup = true;
	
	i2c_master_bus_handle_t airspeed_bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&airspeed_bus_config, &airspeed_bus_handle));

	i2c_device_config_t airspeed_dev_cfg = {};
	    airspeed_dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    	airspeed_dev_cfg.device_address = AIRSPEED_I2C_ADDR;
    	airspeed_dev_cfg.scl_speed_hz = 100000;

	i2c_master_dev_handle_t airspeed_dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(airspeed_bus_handle, &airspeed_dev_cfg, &airspeed_dev_handle));
	Airspeed.setup(airspeed_dev_handle);
}

void serial_buses_setup(){
	init_IMU_i2c();
	init_Airspeed_i2c();

    radio_uart_setup();
    gps_uart_setup();

}
