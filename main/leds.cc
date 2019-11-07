
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <NeoPixelBus.h>
#include "leds.h"

extern "C" void task_leds(void *pvParameters)
{
	xQueueHandle qin = pvParameters;
	static const char *ttag = "leds";
	const int colorSaturation = 64;
	const uint16_t PixelCount = CONFIG_LEDS_STRIP_LENGTH;
	const uint8_t PixelPin = 2;  /* ignored on esp8266 */
	NeoPixelBus<NeoGrbFeature, NeoEsp8266Dma800KbpsMethod> strip(PixelCount, PixelPin);
	RgbColor red(colorSaturation, 0, 0);
	RgbColor green(0, colorSaturation, 0);
	RgbColor blue(0, 0, colorSaturation);
	RgbColor white(colorSaturation);
	RgbColor black(0);

	/* this resets all the neopixels to an off state */
	strip.Begin();
	strip.Show();

	int i = 0;
	while(true) {
		float d;
		if (xQueueReceive(qin, &d, 50)) {
			/* got valid data, update leds... */

			/*
			 * loose plan, far is red, near is blue....
			 */
			const float maxd = 2.0f;
			float ratio = (maxd - d) / maxd;
			ESP_LOGI(ttag, "for d: %f, ratio of %f determined", d, ratio);
			HslColor far(0.0f, 0.9f, 0.5f);
			HslColor near(200.0f/360.0f, 0.9f, 0.5f);
			HslColor indicator = HslColor::LinearBlend<NeoHueBlendShortestDistance>(far, near, ratio);
			for (int k = 0; k < PixelCount; k++) {
				/* I guess, once you've got to this, do some blending along the strip too... */
				strip.SetPixelColor(k, indicator);
			}
			strip.Show();
		} else {
			ESP_LOGW(ttag, "failed to get a distance reading...");
			/* in a magic world we fire off a task that magically keeps the leds blinking some warning pattern */
		}
	}
}

