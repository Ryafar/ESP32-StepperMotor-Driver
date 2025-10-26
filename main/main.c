// FreeRTOS example for ESP32-C6 with RGB LED and Stepper Motor control
// - Blinks the LED in red using the ESP-IDF led_strip (RMT) driver on GPIO 8
// - Controls a stepper motor via boot button (GPIO9)

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "led_strip.h"
#include "driver/gpio.h"
#include "esp_log.h"

// =============================
// LED Configuration
// =============================
// On many ESP32-C6 dev boards, the addressable RGB LED is connected to GPIO 8.
#ifndef LED_STRIP_GPIO
#define LED_STRIP_GPIO 8
#endif

#ifndef LED_STRIP_LED_NUM
#define LED_STRIP_LED_NUM 1
#endif

// =============================
// Stepper Motor Configuration
// =============================
// Stepper motor driver pins
#ifndef STEPPER_DIR_GPIO
#define STEPPER_DIR_GPIO GPIO_NUM_0    // Direction control
#endif

#ifndef STEPPER_ENABLE_GPIO
#define STEPPER_ENABLE_GPIO GPIO_NUM_1 // Enable (active low for most drivers)
#endif

#ifndef STEPPER_STEP_GPIO
#define STEPPER_STEP_GPIO GPIO_NUM_2   // Step pulse
#endif

#ifndef BUTTON_GPIO
#define BUTTON_GPIO GPIO_NUM_9         // Boot button
#endif

// Stepper motor parameters
#define STEPS_PER_REVOLUTION 200       // Typical for 1.8° stepper
#define STEP_DELAY_US 1000             // Delay between steps (1ms = 1000 steps/sec)

static const char *TAG = "MAIN";
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

static void stepper_motor_task(void *arg)
{
	ESP_LOGI(TAG, "Stepper motor task started. Press BOOT button (GPIO%d) to turn motor.", BUTTON_GPIO);
	
	// Set initial state: motor disabled, direction forward
	gpio_set_level(STEPPER_ENABLE_GPIO, 1); // Disable motor (active low)
	gpio_set_level(STEPPER_DIR_GPIO, 0);    // Direction = forward
	
	bool motor_running = false;
	
	while (1) {
		// Read button state (active low - pressed = 0)
		int button_state = gpio_get_level(BUTTON_GPIO);
		
		if (button_state == 0) { // Button pressed
			if (!motor_running) {
				ESP_LOGI(TAG, "Button pressed - enabling motor");
				gpio_set_level(STEPPER_ENABLE_GPIO, 0); // Enable motor
				motor_running = true;
			}
			
			// Generate step pulse
			gpio_set_level(STEPPER_STEP_GPIO, 1);
			esp_rom_delay_us(10); // Short pulse width (10µs)
			gpio_set_level(STEPPER_STEP_GPIO, 0);
			esp_rom_delay_us(STEP_DELAY_US); // Delay between steps
			
		} else { // Button released
			if (motor_running) {
				ESP_LOGI(TAG, "Button released - disabling motor");
				gpio_set_level(STEPPER_ENABLE_GPIO, 1); // Disable motor
				motor_running = false;
			}
			vTaskDelay(pdMS_TO_TICKS(10)); // Check button every 10ms when not pressed
		}
	}
}

void app_main(void)
{
	// =============================
	// Initialize LED Strip
	// =============================
	led_strip_config_t strip_config = {
		.strip_gpio_num = LED_STRIP_GPIO,
		.max_leds = LED_STRIP_LED_NUM,
	};

	led_strip_rmt_config_t rmt_config = {
		.resolution_hz = 10 * 1000 * 1000, // 10 MHz resolution
	};

	ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &s_led_strip));
	ESP_ERROR_CHECK(led_strip_clear(s_led_strip)); // Make sure LED is off at start

	// =============================
	// Initialize Stepper Motor GPIOs
	// =============================
	// Configure output pins for stepper motor
	gpio_config_t stepper_outputs = {
		.pin_bit_mask = (1ULL << STEPPER_DIR_GPIO) | 
		                (1ULL << STEPPER_ENABLE_GPIO) | 
		                (1ULL << STEPPER_STEP_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&stepper_outputs));

	// Configure boot button as input with pull-up
	gpio_config_t button_input = {
		.pin_bit_mask = (1ULL << BUTTON_GPIO),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_ENABLE,  // Internal pull-up
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	ESP_ERROR_CHECK(gpio_config(&button_input));

	ESP_LOGI(TAG, "GPIO configured:");
	ESP_LOGI(TAG, "  Stepper DIR: GPIO%d", STEPPER_DIR_GPIO);
	ESP_LOGI(TAG, "  Stepper ENABLE: GPIO%d (active low)", STEPPER_ENABLE_GPIO);
	ESP_LOGI(TAG, "  Stepper STEP: GPIO%d", STEPPER_STEP_GPIO);
	ESP_LOGI(TAG, "  Button: GPIO%d (active low)", BUTTON_GPIO);

	// =============================
	// Create FreeRTOS tasks
	// =============================
	xTaskCreate(rgb_blink_task, "rgb_blink", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(stepper_motor_task, "stepper_motor", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
	
	ESP_LOGI(TAG, "All tasks started");
}