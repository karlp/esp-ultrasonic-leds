
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

#include <NeoPixelBus.h>
#include "leds.h"

extern "C" void task_leds(void *args)
{
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
		i++;
		for (int k = 0; k < PixelCount; k++) {
			switch((k + i)%4) {
				case 0: strip.SetPixelColor(k, red); break;
				case 1: strip.SetPixelColor(k, blue); break;
				case 2: strip.SetPixelColor(k, green); break;
				case 3: strip.SetPixelColor(k, white); break;
			}
		}
    		strip.Show();
		vTaskDelay(500 / portTICK_PERIOD_MS);
	}
}

