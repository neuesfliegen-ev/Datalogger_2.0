/*DO NOT RUN BUS SETUP FUNCTIONS WITHOUT HARDWARE CONNECTION: comment out in serial_buses_setup()*/

/* C Libraries */
#include <stdio.h>
#include <string.h>

/* ESP-IDF */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "driver/spi_common.h"
#include "driver/sdspi_host.h"
#include "sdmmc_cmd.h"

/* Local */
#include "pins.h"
#include "setup.h"
#include "hal/radio.h"
#include "hal/imu.h"
#include "modules/telemetry.h"
#include "modules/commandHandler.h"
#include "sd_card.h"

bool DEBUG = false;
bool TELEMETRY_ENABLED = true;
bool LOGGING = false;

QueueHandle_t radioQueue;
int command, option; 

SDCard sd;
Telemetry telemetry;
CJY901 IMU;
RadioClass Radio(RADIO_M0_PIN, RADIO_M1_PIN);
//PitotClass Pitot;
//GPSClass GPS;
CommandHandler commandHandler(Radio, IMU, telemetry, TELEMETRY_ENABLED);

static uint32_t last_gps_update = 0;
static uint32_t last_imu_update = 0;
static uint32_t last_pitot_update = 0;

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

//Read sensors, read radio, send over radio 
void polling_task(void *pvParameters) {
	RadioMessage msg;
    
    for(;;) {
		if (Radio.readCommand(command, option)) {
    		commandHandler.executeCommand(command, option);
		}        

		if(TELEMETRY_ENABLED){
			update_all_sensor_data(); 
			Radio.sendDataset(&telemetry);		
		}

		if (xQueueReceive(radioQueue, &msg, portMAX_DELAY) == pdTRUE) {
            uart_write_bytes(RADIO_UART_NUM, msg.text, msg.len);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

//Log to SD
void logging_task(void* arg) {
    for (;;) {

    }
}

extern "C" void app_main(){
	//esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
	sd.begin();
	serial_buses_setup();

	radioQueue = xQueueCreate(10, sizeof(RadioMessage));
	Radio.setQueue(radioQueue);

	xTaskCreatePinnedToCore(polling_task, "polling_task", 8192, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(logging_task, "logging_task", 4096, NULL, 6, NULL, 1);
}

