#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "core.h"
#include "utils.h"
#include "network.h"
#include "storage.h"
#include "webserver.h"
#include "settings.h"

#include "driver/gpio.h"
// #define CONFIG_LOG_MAXIMUM_LEVEL INFO

static const char *TAG = "MAIN";
static bool inited = false;
static SemaphoreHandle_t sem;

void networkHandler(uint8_t event, uint32_t address) {
	ESP_LOGI(TAG, "event %d, address %d.%d.%d.%d", event, 
		     address & 0xFF, (address & 0xFFFF) >> 8, 
		     (address & 0xFFFFFF) >> 16, address >> 24);
	if (!inited && (event == WIFI_CONNECTED || event == ETH_CONNECTED)) {
		inited = true;
		runWebServer();
	    initWS();
	    initMQTT();
	    initFTP(address);
	} else if (!inited && (event == WIFI_EVENT_AP_STACONNECTED)) {
		inited = true;
		ESP_LOGI(TAG, "staconnected");
		initWebServer(0);
	} else if (event == 100) {
		// sntp event
		sntpEvent();
	}
}

void app_main(void)
{
    ESP_LOGI(TAG, "--= RelayController =--");
    sem = xSemaphoreCreateMutex();
    initStorage(sem);
    loadSettings();
    initCore(sem);
    initNetwork(&networkHandler);     
}

// Chip is ESP32-D0WD-V3 (revision v3.0)   on new
// Chip is ESP32-D0WD-V3 (revision v3.0)   on old