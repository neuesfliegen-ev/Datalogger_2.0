/*DO NOT RUN BUS SETUP FUNCTIONS WITHOUT HARDWARE CONNECTION: comment out in serial_buses_setup()*/

#include <stdio.h>
#include <string.h>

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
#include "commandHandler.h"

static bool DEBUG = false;
static bool TELEMETRY_ENABLED = true;

const gpio_num_t RADIO_TX_PIN = GPIO_NUM_17;
const gpio_num_t RADIO_RX_PIN = GPIO_NUM_16;	
const gpio_num_t RADIO_M0_PIN = GPIO_NUM_34;	
const gpio_num_t RADIO_M1_PIN = GPIO_NUM_35;	
const gpio_num_t GPS_TX_PIN = GPIO_NUM_33; //connect TX of gps
const gpio_num_t GPS_RX_PIN = GPIO_NUM_32; //connect RX of gps
const gpio_num_t IMU_SDA_PIN = GPIO_NUM_21; //1k pullup
const gpio_num_t IMU_SCL_PIN = GPIO_NUM_22; //1k pullup
const gpio_num_t PITOT_SDA_PIN = GPIO_NUM_8; //1k pull up --CHANGE
const gpio_num_t PITOT_SCL_PIN = GPIO_NUM_9; //1k pull up --CHANGE
//const gpio_num_t SD_MISO_PIN = GPIO_NUM_2;
const gpio_num_t SD_CS_PIN = GPIO_NUM_13;
const gpio_num_t SD_SCK_PIN = GPIO_NUM_14;
const gpio_num_t SD_MOSI_PIN = GPIO_NUM_15; //10k pullup

const gpio_num_t BLINK_LED = GPIO_NUM_2;

const uart_port_t RADIO_UART_NUM = UART_NUM_2;
const uart_port_t GPS_UART_NUM = UART_NUM_1;

/*Constants for time keeping in milliseconds*/
const uint32_t GPS_UPDATE_PERIOD = 100;
const uint32_t PITOT_UPDATE_PERIOD = 200;
const uint32_t IMU_UPDATE_PERIOD = 100;

const uint32_t RADIO_BAUD_RATE = 9600;
const uint32_t GPS_BAUD_RATE = 115200;


static const i2c_port_num_t IMU_I2C_PORT = I2C_NUM_1;
static const i2c_port_num_t PITOT_I2C_PORT = I2C_NUM_0;
static const uint16_t WT901B_I2C_ADDR = 0x50;
static const uint32_t wt901b_i2c_scl_speed_hz = 100000;
static const uint32_t sleep_time_ms = 1000;

static const char *TAG = "SD_INIT";

QueueHandle_t radioQueue;
int command, option; 

Telemetry telemetry;
CJY901 IMU;
RadioClass Radio(RADIO_M0_PIN, RADIO_M1_PIN);
//PitotClass Pitot;
//GPSClass GPS;
CommandHandler commandHandler(Radio, IMU, telemetry, TELEMETRY_ENABLED);

static uint32_t last_gps_update = 0;
static uint32_t last_imu_update = 0;
static uint32_t last_pitot_update = 0;

void queue_radio_message(const char *text, size_t len) {
    RadioMessage msg;

    if (len > sizeof(msg.text)) {
        len = sizeof(msg.text);
    }

    memcpy(msg.text, text, len);
    msg.len = len;

    xQueueSend(radioQueue, &msg, 0);
}

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

	//GPS.startUART(GPS_UART_NUM);	
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
    radio_uart_setup();
    gps_uart_setup();
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
	telemetry.update_telemetry(now, imu_data/*, pitot_data, gps_data*/); 

	if(DEBUG) printf("updated telemetry\n");
}

void work_task(void *pvParameters) {
    while (1) {
		if (Radio.readCommand(command, option)) {
    		commandHandler.executeCommand(command, option);
		}        

		if(TELEMETRY_ENABLED){
			update_all_sensor_data(); 
			Radio.sendDataset(&telemetry);

			//GPS exp start ----------------------------------------------------------
			
			uint8_t data[1024];
			size_t length = 0;
			ESP_ERROR_CHECK(uart_get_buffered_data_len(GPS_UART_NUM, &length));
			
			if (length > sizeof(data)) {
			    length = sizeof(data);
			}

			int read_len = uart_read_bytes(GPS_UART_NUM, data, length, pdMS_TO_TICKS(10));

			if (read_len > 0) {
			    RadioMessage msg;
			    size_t pos = 0;

			    memcpy(msg.text + pos, "G: ", 3);
			    pos += 3;

			    if (read_len > sizeof(msg.text) - pos - 2) {
			        read_len = sizeof(msg.text) - pos - 2;
			    }

			    memcpy(msg.text + pos, data, read_len);
			    pos += read_len;

			    memcpy(msg.text + pos, "\r\n", 2);
			    pos += 2;

			    msg.len = pos;

			    xQueueSend(radioQueue, &msg, 0);
			}			
		}

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void radio_task(void* arg) {
    RadioMessage msg;

    while (true) {
        if (xQueueReceive(radioQueue, &msg, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(RADIO_UART_NUM0, msg.text, msg.len);
        }

        vTaskDelay(pdMS_TO_TICKS(1)); // lets watchdog/idle task breathe
    }
}

extern "C" void app_main(){
	radioQueue = xQueueCreate(10, sizeof(RadioMessage));
	Radio.setQueue(radioQueue);
	serial_buses_setup();

	xTaskCreatePinnedToCore(work_task,  "work_task",  8192, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(radio_task, "radio_task", 4096, NULL, 6, NULL, 1);
}

/*
extern "C" void app_main(){
	char* ourTaskName = pcTaskGetName(NULL);
	ESP_LOGI(ourTaskName, "Hello, starting up..");

	SemaphoreHandle_t radioMutex = xSemaphoreCreateMutex();

	serial_buses_setup();
		

	gpio_reset_pin(BLINK_LED);
	gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);
	gpio_set_direction(RADIO_M0_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(RADIO_M0_PIN, 0);
	gpio_set_level(RADIO_M1_PIN, 0);

	while(1){

		vTaskDelay(100 / portTICK_PERIOD_MS);

		if (Radio.readCommand(command, option)) {
    		commandHandler.executeCommand(command, option);
		}


		if(TELEMETRY_ENABLED){
			update_all_sensor_data(); 
			//if(DEBUG)printf("Attitude: %.2f, %.2f, %.2f\n", IMU.imuData.att.Attf[0], IMU.imuData.att.Attf[1], IMU.imuData.att.Attf[2]);
			//if(DEBUG)printf("Attitude from telemetry: %.2f, %.2f, %.2f\n", telemetry.dataset.roll, telemetry.dataset.pitch, telemetry.dataset.yaw);
			xSemaphoreTake(radioMutex, portMAX_DELAY);
			Radio.sendDataset(&telemetry);
			xSemaphoreGive(radioMutex);
			
			vTaskDelay(10 / portTICK_PERIOD_MS);

			//GPS exp start ----------------------------------------------------------
			
			uint8_t data[1024];
			size_t length = 0;
			ESP_ERROR_CHECK(uart_get_buffered_data_len(GPS_UART_NUM, &length));
			
			if (length > sizeof(data)) {
			    length = sizeof(data);
			}

			int read_len = uart_read_bytes(GPS_UART_NUM, data, length, pdMS_TO_TICKS(10));

			if (read_len > 0) {
				if (read_len > 0) {
					xSemaphoreTake(radioMutex, portMAX_DELAY);

	   		 		uart_write_bytes(RADIO_UART_NUM, "G: ", 3);
	    			uart_write_bytes(RADIO_UART_NUM, (const char*)data, read_len);
	    			uart_write_bytes(RADIO_UART_NUM, "\r\n", 2);
	    			xSemaphoreGive(radioMutex);

				}		
			}			
		}


		//GPS exp end ----------------------------------------------------------
	}
}

*/

