/*DO NOT RUN BUS SETUP FUNCTIONS WITHOUT HARDWARE CONNECTION: comment out in serial_buses_setup()*/

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"


#include "driver/spi_common.h"
#include "driver/sdspi_host.h"

#include "sdmmc_cmd.h"


#include "radio.h"
#include "WT901_I2C.h"
#include "telemetry.h"

static const bool DEBUG = true;

const gpio_num_t RADIO_TX_PIN = GPIO_NUM_17;
const gpio_num_t RADIO_RX_PIN = GPIO_NUM_16;
const gpio_num_t GPS_TX_PIN = GPIO_NUM_34;
const gpio_num_t GPS_RX_PIN = GPIO_NUM_35;
const gpio_num_t IMU_SDA_PIN = GPIO_NUM_21; //1k pullup
const gpio_num_t IMU_SCL_PIN = GPIO_NUM_22; //1k pullup
const gpio_num_t PITOT_SDA_PIN = GPIO_NUM_8; //1k pull up --CHANGE
const gpio_num_t PITOT_SCL_PIN = GPIO_NUM_9; //1k pull up --CHANGE
const gpio_num_t SD_MISO_PIN = GPIO_NUM_2;
const gpio_num_t SD_CS_PIN = GPIO_NUM_13;
const gpio_num_t SD_SCK_PIN = GPIO_NUM_14;
const gpio_num_t SD_MOSI_PIN = GPIO_NUM_15; //10k pullup

const gpio_num_t BLINK_LED = GPIO_NUM_2;

const uart_port_t RADIO_UART_NUM = UART_NUM_2;
const uart_port_t GPS_UART_NUM = UART_NUM_1; //CHECK?

/*Constants for time keeping in milliseconds*/
const uint32_t GPS_UPDATE_PERIOD = 1000;
const uint32_t PITOT_UPDATE_PERIOD = 200;
const uint32_t IMU_UPDATE_PERIOD = 100;

static const i2c_port_num_t IMU_I2C_PORT = I2C_NUM_1;
static const i2c_port_num_t PITOT_I2C_PORT = I2C_NUM_0;
static const uint16_t WT901B_I2C_ADDR = 0x50;
static const uint32_t wt901b_i2c_scl_speed_hz = 100000;
static const uint32_t sleep_time_ms = 1000;

static const char *TAG = "SD_INIT";

Telemetry telemetry;
CJY901 IMU;
RadioClass radio;

static uint32_t last_gps_update = 0;
static uint32_t last_imu_update = 0;
static uint32_t last_pitot_update = 0;

esp_err_t radio_uart_setup(){
	const int uart_buffer_size = (1024 * 2);	// Setup UART buffered IO with event queue
	QueueHandle_t uart_queue;	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(RADIO_UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

	uart_config_t uart_config{};   // zero-initialize everything
		uart_config.baud_rate = 9600;
		uart_config.data_bits = UART_DATA_8_BITS;
		uart_config.parity    = UART_PARITY_DISABLE;
		uart_config.stop_bits = UART_STOP_BITS_1;
		uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uart_config.source_clk = UART_SCLK_DEFAULT;	
	ESP_ERROR_CHECK(uart_param_config(RADIO_UART_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(RADIO_UART_NUM, RADIO_TX_PIN, RADIO_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	radio.startUART(RADIO_UART_NUM);
	return ESP_OK;
}

void gps_uart_setup(){
	const int uart_buffer_size = (1024 * 2);	// Setup UART buffered IO with event queue
	QueueHandle_t uart_queue;	// Install UART driver using an event queue here
	ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));

	uart_config_t uart_config{};   // zero-initialize everything
		uart_config.baud_rate = 9600;
		uart_config.data_bits = UART_DATA_8_BITS;
		uart_config.parity    = UART_PARITY_DISABLE;
		uart_config.stop_bits = UART_STOP_BITS_1;
		uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
		uart_config.source_clk = UART_SCLK_DEFAULT;	
	ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	radio.startUART(GPS_UART_NUM);	
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
    	imu_dev_cfg.device_address = 0x50;
    	imu_dev_cfg.scl_speed_hz = 100000;

	i2c_master_dev_handle_t imu_dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(imu_bus_handle, &imu_dev_cfg, &imu_dev_handle));
	IMU.StartIIC(imu_dev_handle);
}

void init_Pitot_i2c(){
	i2c_master_bus_config_t imu_bus_config = {};
		imu_bus_config.i2c_port = PITOT_I2C_PORT;
		imu_bus_config.sda_io_num = PITOT_SDA_PIN;
		imu_bus_config.scl_io_num = PITOT_SCL_PIN;
		imu_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
		imu_bus_config.glitch_ignore_cnt = 7;
		imu_bus_config.intr_priority = 0;
		imu_bus_config.trans_queue_depth = 0;
		imu_bus_config.flags.enable_internal_pullup = true;
	
	i2c_master_bus_handle_t imu_bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&imu_bus_config, &imu_bus_handle));

	i2c_device_config_t imu_dev_cfg = {};
	    imu_dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    	imu_dev_cfg.device_address = 0x50;
    	imu_dev_cfg.scl_speed_hz = 100000;

	i2c_master_dev_handle_t imu_dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(imu_bus_handle, &imu_dev_cfg, &imu_dev_handle));
	//Pitot.StartIIC(imu_dev_handle);
}


void serial_buses_setup(){
	init_IMU_i2c();
	//init_Pitot_i2c();

	//if(init_spi_bus() != ESP_OK){return;}
    //if(init_sdcard() != ESP_OK){return;}
    //radio_uart_setup();
    //gps_uart_setup();

}

void update_all_sensor_data(){
	uint32_t now = esp_timer_get_time() / 1000;
	/* **update all the data structs**
	if (now - last_gps_update >= GPS_UPDATE_PERIOD) {
    	gps_data = GPS.updateAll();
    	last_gps_update = now;
	}
	if (now - last_pitot_update >= PITOT_UPDATE_PERIOD) {
    	pitot_data = Pitot.updateAll();
    	last_pitot_update = now;
	}
	if (now - last_imu_update >= IMU_UPDATE_PERIOD){
		imu_data = IMU.updateAll();	
		last_imu_update = now;
	}
	*/
	const IMUData& imu_data = IMU.updateAll();
	telemetry.update_telemetry(imu_data/*, pitot_data, gps_data*/); 

	if(DEBUG)printf("updated telemetry\n");
}


extern "C" void app_main(){
	serial_buses_setup();

	char* ourTaskName = pcTaskGetName(NULL);
	ESP_LOGI(ourTaskName, "Hello, starting up..");
	
	gpio_reset_pin(BLINK_LED);
	gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);


	while(1){
		gpio_set_level(BLINK_LED, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		
		update_all_sensor_data();

		printf("Attitude: %.2f, %.2f, %.2f\n", IMU.imuData.att.Attf[0], IMU.imuData.att.Attf[1],IMU.imuData.att.Attf[2]);

		gpio_set_level(BLINK_LED, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}


