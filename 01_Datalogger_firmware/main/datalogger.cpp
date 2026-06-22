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
bool TELEMETRY_ENABLED = false;
bool LOGGING = true;

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
        "/sdcard/21062026_%lld_test_file.bin",
        ms
    );
}

void update_all_sensor_data(){
	uint32_t now = esp_timer_get_time() / 1000;
	/* **update all the data structs**/
	/*0.7 ms*/
	if (now - last_airspeed_update >= AIRSPEED_UPDATE_PERIOD) {
	int64_t t0 = esp_timer_get_time();
	esp_err_t err = Airspeed.read();
		//int64_t t1 = esp_timer_get_time();
		//printf("airspeed read time = %.2f ms\n", (t1 - t0) / 1000.0);    	
	last_airspeed_update = now;
	}	

	/*$=ms max.*/
	if (now - last_gps_update >= GPS_UPDATE_PERIOD) {
		int64_t t2 = esp_timer_get_time();	
    	GPS.update();
    	int64_t t3 = esp_timer_get_time();
    	//printf("gps read time = %.2f ms\n", (t3 - t2) / 1000.0);    	
    	last_gps_update = now;
	}	
	/*6ms?*/
	int64_t t4 = esp_timer_get_time();
	IMU.updateAll(); 
		int64_t t5 = esp_timer_get_time();
    	//printf("IMU read time = %.2f ms\n", (t5 - t4) / 1000.0);    		
	last_imu_update = now;
	/*1ms?*/
	telemetry.update_telemetry(now, IMU, GPS, Airspeed); 

	if(DEBUG) ESP_LOGI("TELEMETRY", "updated telemetry\n");
}

//Read sensors, read radio, send over radio 
void polling_task(void *pvParameters) {
	RadioMessage msg;
    
    for(;;) {

    	update_all_sensor_data(); 

    	//Queue to the sd card 
  		if (xQueueSend(datasetQueue, &telemetry.dataset, 0) != pdTRUE) {
    		ESP_LOGW("SD QUEUE", "datasetQueue full, sample dropped");
		}     

		if(TELEMETRY_ENABLED){
			if (Radio.readCommand(command, option)) commandHandler.executeCommand(command, option);
			Radio.sendDataset(&telemetry);	
			if (xQueueReceive(radioQueue, &msg, portMAX_DELAY) == pdTRUE) uart_write_bytes(RADIO_UART_NUM, msg.text, msg.len);
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
	    	/*
	    	if (xQueuePeek(datasetQueue, &sample, portMAX_DELAY) == pdTRUE) {
					    ESP_LOGI("QUEUE",
					        "t=%u "
					        "ax=%.3f ay=%.3f az=%.3f "
					        "gx=%.3f gy=%.3f gz=%.3f "
					        "hx=%.3f hy=%.3f hz=%.3f "
					        "roll=%.3f pitch=%.3f yaw=%.3f "
					        "tmp=%.3f hght=%.3f press=%.3f "
					        "dpress=%.3f airspeed_temp=%.3f "
					        "lat=%ld lon=%ld gs=%u gps_alt=%ld "
					        "s_count=%u "
					        "gps=%02u:%02u:%02u.%03u "
					        "%02u/%02u/%04u",
					        sample.t,
					        sample.ax, sample.ay, sample.az,
					        sample.gx, sample.gy, sample.gz,
					        sample.hx, sample.hy, sample.hz,
					        sample.roll, sample.pitch, sample.yaw,
					        sample.tmp, sample.hght, sample.press,
					        sample.dpress, sample.airspeed_temp,
					        (long)sample.lat,
					        (long)sample.lon,
					        sample.ground_speed,
					        (long)sample.gps_alt,
					        sample.s_count,
					        sample.gps_hour,
					        sample.gps_minute,
					        sample.gps_second,
					        sample.gps_millisecond,
					        sample.gps_day,
					        sample.gps_month,
					        sample.gps_year
					    );
					} else {
					    ESP_LOGI("QUEUE", "Queue empty");
					}*/
			

	        writeBuffer[count++] = sample;
	        //ESP_LOGW("SD QUEUE", "received to datasetQueue");

	        if (count >= 32) {
	            esp_err_t err = sd.writeDatasets(writeBuffer, count);
	            if (err != ESP_OK) {
    				ESP_LOGE("SD", "write failed: %s", esp_err_to_name(err));
				}else{
					ESP_LOGW("SD QUEUE", "wrote to sd card");					
				}
	            count = 0;

	            flush_counter++;

	            if (flush_counter >= 4) {
	                esp_err_t errFlush = sd.flush();
	                if(errFlush != ESP_OK) ESP_LOGE("SD QUEUE", "flush failed");
					
					if (err != ESP_OK) {
					    ESP_LOGE("SD QUEUE", "SD end failed: %s", esp_err_to_name(err));
					}	                
					flush_counter = 0;
					int time = esp_timer_get_time();
					if(time >= 300000000) sd.end();
	            
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

