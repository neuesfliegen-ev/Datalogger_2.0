#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "timers.h"

#include "esp_log.h"

#include "driver/gpio.h"

#define RADIO_TX_PIN (GPIO_NUM_17)
#define RADIO_RX_PIN (GPIO_NUM_16)
#define BLINK_LED 2

void task1(){

}

void app_main(void)
{
	char* ourTaskName = pcTaskGetName(NULL);
	ESP_LOGI(ourTaskName, "Hello, starting up..");
	
	gpio_reset_pin(BLINK_LED);
	gpio_set_direction(BLINK_LED, GPIO_MODE_OUTPUT);

	while(1){
		gpio_set_level(BLINK_LED, 1);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
		gpio_set_level(BLINK_LED, 0);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

