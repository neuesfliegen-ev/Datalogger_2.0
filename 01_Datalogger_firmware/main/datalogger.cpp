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
#include "hal/gps.h"
#include "hal/airspeed.h"
#include "modules/telemetry.h"
#include "modules/commandHandler.h"
#include "hal/sd_card.h"

/* Settings */
bool DEBUG = false;
bool TELEMETRY_ENABLED = true;
bool LOGGING = false;

QueueHandle_t datasetQueue;
QueueHandle_t radioQueue;
int command, option; 

//Module objects
SDCard sd;
Telemetry telemetry;
CJY901 IMU;
AirspeedClass Airspeed;
GPSClass GPS;
RadioClass Radio(RADIO_M0_PIN, RADIO_M1_PIN);
CommandHandler commandHandler(Radio, IMU, telemetry, TELEMETRY_ENABLED);

static uint32_t last_gps_update = 0;
static uint32_t last_imu_update = 0;
static uint32_t last_airspeed_update = 0;

void generateFileName(char *buffer, size_t buffer_size){
    int64_t ms = esp_timer_get_time() / 1000;

    snprintf(
        buffer,
        buffer_size,
        "/sdcard/%lld_test_file.bin",
        ms
    );
}

void update_all_sensor_data(){
	uint32_t now = esp_timer_get_time() / 1000;
	/* **update all the data structs**/
	/*5ms?*/
	if (now - last_airspeed_update >= AIRSPEED_UPDATE_PERIOD) {
    	Airspeed.read();
    	last_airspeed_update = now;
	}
	/*10ms max.*/
	if (now - last_gps_update >= GPS_UPDATE_PERIOD) {
    	GPS.update();
    	last_gps_update = now;
	}	
	/*3ms?*/
	IMU.updateAll(); last_imu_update = now;
	/*1ms?*/
	telemetry.update_telemetry(now, IMU, GPS, Airspeed); 

	if(DEBUG) ESP_LOGI("TELEMETRY", "updated telemetry\n");
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
	SDataset writeBuffer[32];
	size_t count = 0;
	uint32_t flush_counter = 0;

	for (;;) {
	    SDataset sample;

	    if (xQueueReceive(datasetQueue, &sample, portMAX_DELAY) == pdTRUE) {
	        writeBuffer[count++] = sample;

	        if (count >= 32) {
	            sd.writeDatasets(writeBuffer, count);
	            count = 0;

	            flush_counter++;

	            if (flush_counter >= 10) {
	                sd.flush();
	                flush_counter = 0;
	            }
	            if(DEBUG) ESP_LOGI("SD CARD", "flushed file");	        
	        }
	    }
	}
}

extern "C" void app_main(){
	//esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
	sd.begin();
	serial_buses_setup();

	//Create binary file (test phase)
	char filename[64];
	generateFileName(filename, sizeof(filename));
	sd.openLogFile(filename);


	radioQueue = xQueueCreate(10, sizeof(RadioMessage));
	Radio.setQueue(radioQueue);

	datasetQueue = xQueueCreate(32, sizeof(SDataset));

	xTaskCreatePinnedToCore(polling_task, "polling_task", 4096, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(logging_task, "logging_task", 6144, NULL, 6, NULL, 1);
	//xTaskCreatePinnedToCore(gps_task, "gps_task", 1024, NULL, 7, NULL, 0);
}

