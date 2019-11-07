#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "leds.h"

static const char *TAG = "us-leds";

esp_err_t event_handler(void *ctx, system_event_t *event)
{
    return ESP_OK;
}

static void us_timing_isr(void *args)
{
	xQueueHandle q = (xQueueHandle)args;
	static int tstart;
	struct timeval now;
	int pin = CONFIG_US_PIN_ECHO;
	if (gpio_get_level(pin)) {
		gettimeofday(&now, NULL);
		tstart = now.tv_usec;
	} else {
		gettimeofday(&now, NULL);
		int diff = now.tv_usec - tstart;
		/* post back to queue */
		xQueueSendFromISR(q, &diff, NULL);
		esp_err_t err = gpio_set_intr_type(pin, GPIO_INTR_DISABLE);
		if (err) {
			ESP_LOGW(TAG, "Failed to turn off echo irq: %d", err);
		}
	}
}


void task_ultrasonic(void *pvParameters)
{
	int ptrig = CONFIG_US_PIN_TRIGGER;
	int pecho = CONFIG_US_PIN_ECHO;
	gpio_config_t iotrig = {
		.pin_bit_mask = (1<<CONFIG_US_PIN_TRIGGER),
		.mode = GPIO_MODE_OUTPUT,
	};
	gpio_config_t ioecho = {
		.pin_bit_mask = (1<<CONFIG_US_PIN_ECHO),
		.mode = GPIO_MODE_INPUT,
	};

	esp_err_t err;
	err = gpio_install_isr_service(0);
	if (err) {
		ESP_LOGW(TAG, "Failed to setup isr service: %d", err);
	}
	/* even 3 elements is way bigger than we need really */
	xQueueHandle us_evt_queue = xQueueCreate(3, sizeof(uint32_t));
	err = gpio_isr_handler_add(pecho, us_timing_isr, us_evt_queue);
	if (err) {
		ESP_LOGW(TAG, "Failed to install private isr: %d", err);
	}

	gpio_config(&iotrig);
	gpio_config(&ioecho);

	gpio_set_level(ptrig, 0);
	ESP_LOGI(TAG, "Starting ultrasonic task, trig is: %d and echo is %d", ptrig, pecho);
	
	while(true) {
		gpio_set_level(ptrig, 1);
		/* allegedly requires 10us high pulse.  LA shows that 10us delay here results in ~20us */
		os_delay_us(5);
		gpio_set_level(ptrig, 0);
		err = gpio_set_intr_type(pecho, GPIO_INTR_ANYEDGE);
		if (err) {
			ESP_LOGW(TAG, "Failed to set irq on echo: %d", err);
		}
		
		/* 50ms is _wayyyy_ longer than it should take to get an echo */
		uint32_t tv;
		if (xQueueReceive(us_evt_queue, &tv, 50 / portTICK_PERIOD_MS)) {
			/* process distance and post data to .... someone */
			/* s = ut + ½at² */
			/* s = ut, div 2 for echo */
			float t = 20.0f;
			float v = 331.4 + (0.6*t);
			float distance = tv * 1e-6 * v / 2;
			ESP_LOGI(TAG, "time: %dus, %f m", tv, distance);
		} else {
			/* didn't get an irq back! */
			ESP_LOGI(TAG, "no data :(");
		}
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
	
}

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t sta_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &sta_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_ERROR_CHECK( esp_wifi_connect() );

    xTaskCreate(task_ultrasonic, "ultrasonci", 2048, NULL, 10, NULL);
    xTaskCreate(task_leds, "leds", 2048, NULL, 10, NULL);

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_4, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

