// FreeRTOS example for ESP32-C6 with a one-wire RGB LED (WS2812/NeoPixel)
// Blinks the LED in red using the ESP-IDF led_strip (RMT) driver on GPIO 8.

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "esp_log.h"

// =============================
// Configuration
// =============================
// On many ESP32-C6 dev boards, the addressable RGB LED is connected to GPIO 8.
// Adjust if your board uses a different pin or more LEDs in the strip.
#ifndef LED_STRIP_GPIO
#define LED_STRIP_GPIO 8
#endif

#ifndef LED_STRIP_LED_NUM
#define LED_STRIP_LED_NUM 1
#endif

static const char *TAG = "RGB_BLINK";
static led_strip_handle_t s_led_strip;

static void rgb_blink_task(void *arg)
{
	ESP_LOGI(TAG, "Blinking WS2812 on GPIO %d", LED_STRIP_GPIO);
	while (1) {
		// Set pixel 0 to RED (R,G,B)
		led_strip_set_pixel(s_led_strip, 0, 10, 0, 0);
		led_strip_refresh(s_led_strip);
		vTaskDelay(pdMS_TO_TICKS(500));

		// Turn all pixels off
		led_strip_clear(s_led_strip);
		led_strip_refresh(s_led_strip);
		vTaskDelay(pdMS_TO_TICKS(500));
	}
}

void app_main(void)
{
	// Create and initialize the LED strip driver
	led_strip_config_t strip_config = {
		.strip_gpio_num = LED_STRIP_GPIO,
		.max_leds = LED_STRIP_LED_NUM,
		// Other fields default to 0 (RGB format, WS2812 model in most IDF versions)
	};

	led_strip_rmt_config_t rmt_config = {
		.resolution_hz = 10 * 1000 * 1000, // 10 MHz resolution
		// Other fields default to sensible values
	};

	ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
	ESP_ERROR_CHECK(led_strip_clear(s_led_strip)); // Make sure LED is off at start

	// Create the blink task
	xTaskCreate(rgb_blink_task, "rgb_blink", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
}