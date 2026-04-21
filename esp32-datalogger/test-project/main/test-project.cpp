#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "WT901_I2C.h"


const gpio_num_t RADIO_TX_PIN = GPIO_NUM_17;
const gpio_num_t RADIO_RX_PIN = GPIO_NUM_18;
const gpio_num_t IMU_SDA_PIN = GPIO_NUM_21;
const gpio_num_t IMU_SCL_PIN = GPIO_NUM_22;

static const i2c_port_num_t IMU_I2C_PORT = I2C_NUM_1;
static const uint16_t WT901B_I2C_ADDR = 0x50;
static const uint32_t wt901b_i2c_scl_speed_hz = 100000;
static const uint32_t sleep_time_ms = 1000;

static const gpio_num_t BLINK_LED = GPIO_NUM_2;

CJY901 IMU;

void serial_buses_setup(){
	i2c_master_bus_config_t imu_bus_config = {};
		imu_bus_config.i2c_port = I2C_NUM_1;
		imu_bus_config.sda_io_num = GPIO_NUM_21;
		imu_bus_config.scl_io_num = GPIO_NUM_22;
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

extern "C" void app_main(){
	serial_buses_setup();


	char* ourTaskName = pcTaskGetName(NULL);
	ESP_LOGI(ourTaskName, "Hello, starting up..");
	
	gpio_reset_pin(BLINK_LED);
	gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);

	while(1){
		gpio_set_level(BLINK_LED, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		IMU.updateAll();
		printf("Attitude: %.2f, %.2f, %.2f\n", IMU.stcAtt.Attf[0], IMU.stcAtt.Attf[1],IMU.stcAtt.Attf[2]);
		
		gpio_set_level(BLINK_LED, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}


