/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "led_strip.h"
#include "sdkconfig.h"

static const char *TAG = "example";

/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/
#define BLINK_GPIO CONFIG_BLINK_GPIO

//static uint8_t s_led_state = 0;

//#ifdef CONFIG_BLINK_LED_RMT
#if 0
static led_strip_t *pStrip_a;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        pStrip_a->set_pixel(pStrip_a, 0, 16, 16, 16);
        /* Refresh the strip to send data */
        pStrip_a->refresh(pStrip_a, 100);
    } else {
        /* Set all LED off to clear all pixels */
        pStrip_a->clear(pStrip_a, 50);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    pStrip_a = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, 1);
    /* Set all LED off to clear all pixels */
    pStrip_a->clear(pStrip_a, 50);
}

#endif



static void IRAM_ATTR us_timing_isr(void *args)
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


void task_leds(void *pvParameters)
{
        xQueueHandle qin = pvParameters;
	esp_err_t err;
	const char *ttag = "leds";
	const int pixn = CONFIG_LEDS_STRIP_LENGTH;

	led_strip_t *strip;
	strip = led_strip_init(CONFIG_BLINK_LED_RMT_CHANNEL, BLINK_GPIO, pixn);
	/* Set all LED off to clear all pixels */
	strip->clear(strip, 50);
	int iter = 0;

	while (true) {
		float d;
		if (xQueueReceive(qin, &d, 50)) {
			err = strip->clear(strip, 10);
                        const float maxd = 2.0f;
			if (d > maxd) {
				ESP_LOGD(ttag, "ignoring out of range: %f", d);
				continue;
			}
                        float ratio = (maxd - d) / maxd;
			uint32_t lit = ratio * pixn;
                        ESP_LOGI(ttag, "for d: %f, ratio of %f => %d leds determined", d, ratio, lit);
			
			if (err) {
				ESP_LOGW(ttag, "failed to clear: %d", err);
			}
			for (int i = 0; i < lit; i++) {
				int r = (ratio * 255);
				int g = 0; //255 - (ratio * 255);
				int b = 255 - (ratio * 255);
				err = strip->set_pixel(strip, i, r/4,g/4,b/4); // make it dimmer...
				if (err) {
					ESP_LOGW(ttag, "failed to set pixel: %d %d", i, err);
				}
			}
			err = strip->refresh(strip, 20);
			if (err) {
				ESP_LOGW(ttag, "failed to refresh: %d", err);
			}
		} else {
			ESP_LOGW(ttag, "failed to get a distance reading");
			/* blink a "warning" pattern?! */
		}
	}
	while (false) {
		ESP_LOGI(ttag, "iter: %d", iter);
		strip->clear(strip, 50);
		strip->set_pixel(strip, iter % pixn, 60, 20, 10);
		strip->refresh(strip, 100);
		iter++;
		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}



void task_ultrasonic(void *pvParameters)
{
        xQueueHandle qout = pvParameters;
        int ptrig = CONFIG_US_PIN_TRIGGER;
        int pecho = CONFIG_US_PIN_ECHO;
        gpio_config_t iotrig = {
                .pin_bit_mask = (1<<CONFIG_US_PIN_TRIGGER),
                .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config_t ioecho = {
		.intr_type = GPIO_INTR_ANYEDGE,
                .pin_bit_mask = (1<<CONFIG_US_PIN_ECHO),
                .mode = GPIO_MODE_INPUT,
		//.pull_up_en = 1,
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
                //os_delay_us(5);
		esp_rom_delay_us(5);
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
#if 1
                        if (xQueueSend(qout, &distance, 20)) {
                                /* good */
                        } else {
                                ESP_LOGW(TAG, "failed to post distance to consumers?");
                        }
#endif
                } else {
                        /* didn't get an irq back! */
                        ESP_LOGI(TAG, "no data :(");
                }
                vTaskDelay(300 / portTICK_PERIOD_MS);
        }

}


void app_main(void)
{

    /* Configure the peripheral according to the LED type */
//    configure_led();

	xQueueHandle distance_queue = xQueueCreate(3, sizeof(float));
	xTaskCreate(task_ultrasonic, "ultrasonic", 2048, distance_queue, 10, NULL);
	xTaskCreate(task_leds, "leds", 2048, distance_queue, 10, NULL);


    while (1) {
        //ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        //blink_led();
        /* Toggle the LED state */
        //s_led_state = !s_led_state;
        //vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
	vTaskDelay(500/portTICK_PERIOD_MS);
    }
}
