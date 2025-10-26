// FreeRTOS example for ESP32-C6 with RGB LED and Stepper Motor control
// - Blinks the LED in red using the ESP-IDF led_strip (RMT) driver on GPIO 8
// - Controls a TMC2209 stepper motor driver via boot button (GPIO9)
//
// TMC2209 Configuration (standalone mode):
// - Enable: Active LOW (0 = enabled)
// - MS1/MS2: Configure microstepping (see datasheet)
// - VREF: Adjust potentiometer for motor current
//   Formula: I_RMS = VREF / (8 × 0.15) ≈ VREF / 1.2
//   For 4.1V motor at typical 1-2A: VREF ≈ 1.2V - 2.4V
// - Spread Cycle mode (default) - smoother operation

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
// Stepper Motor Configuration - TMC2209
// =============================
// TMC2209 driver pins (standalone mode)
#ifndef STEPPER_DIR_GPIO
#define STEPPER_DIR_GPIO GPIO_NUM_0    // Direction control
#endif

#ifndef STEPPER_ENABLE_GPIO
#define STEPPER_ENABLE_GPIO GPIO_NUM_1 // Enable (active LOW for TMC2209)
#endif

#ifndef STEPPER_STEP_GPIO
#define STEPPER_STEP_GPIO GPIO_NUM_2   // Step pulse
#endif

#ifndef BUTTON_GPIO
#define BUTTON_GPIO GPIO_NUM_9         // Boot button
#endif

// Stepper motor parameters
// TMC2209 microstepping configuration via MS1/MS2 pins:
//
// | MS1  | MS2  | Microstepping | Steps/Rev | Smoothness | Torque |
// |------|------|---------------|-----------|------------|--------|
// | Open | Open | 1/8           | 1600      | Very smooth| Medium | <-- CURRENT SETTING
// | GND  | Open | 1/2           | 400       | Rougher    | High   |
// | VIO  | Open | 1/4           | 800       | Smooth     | Good   |
// | GND  | GND  | 1/16          | 3200      | Ultra smooth| Low   |
// | VIO  | VIO  | Full step     | 200       | Rough      | Maximum|
// | Open | VIO  | 1/4           | 800       | Smooth     | Good   |
// | GND  | VIO  | 1/16          | 3200      | Ultra smooth| Low   |
// | VIO  | GND  | 1/32          | 6400      | Extreme smooth| Very low|
//
// Note: "Open" means pin not connected (floating). "VIO" = 3.3V or 5V logic voltage.
//
// Current configuration: 1/8 microstepping (MS1=Open, MS2=Open) - DEFAULT
#define STEPS_PER_REVOLUTION 1600      // For 1/8 microstepping - very smooth motion
#define STEP_DELAY_US 1250             // 1000µs = ~60 RPM (1 rev/sec)
#define STEP_DELAY_START_US 5000       // Start even slower for smooth acceleration
#define PULSE_WIDTH_US 3               // Minimum 100ns for TMC2209, using 3µs for safety
#define ACCELERATION_STEPS 800         // Ramp up over 0.5 revolutions (800 steps)

// Motor control mode
#define USE_TOGGLE_MODE 1              // 1 = toggle on/off with button press, 0 = hold to run
#define USE_ACCELERATION 1             // 1 = smooth acceleration/deceleration, 0 = instant speed

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
	ESP_LOGI(TAG, "TMC2209 stepper motor task started. Press BOOT button (GPIO%d) to toggle motor.", BUTTON_GPIO);
	ESP_LOGI(TAG, "Microstepping: 1/8 step (1600 steps/rev) - very smooth motion");
	ESP_LOGI(TAG, "Target speed: 30 RPM (0.5 rev/sec)");
	ESP_LOGI(TAG, "Control mode: %s", USE_TOGGLE_MODE ? "TOGGLE (press once to start/stop)" : "HOLD (hold button to run)");
	ESP_LOGI(TAG, "Acceleration: %s", USE_ACCELERATION ? "ENABLED (smooth ramp)" : "DISABLED (instant speed)");
	ESP_LOGW(TAG, "IMPORTANT: For 1/8 microstepping, leave MS1 and MS2 OPEN (not connected)!");
	ESP_LOGW(TAG, "IMPORTANT: If motor stutters, increase VREF current (turn pot clockwise)!");
	
	// TMC2209 initial state: motor disabled, direction forward
	gpio_set_level(STEPPER_ENABLE_GPIO, 1); // Disable motor (active LOW)
	gpio_set_level(STEPPER_DIR_GPIO, 0);    // Direction = forward
	gpio_set_level(STEPPER_STEP_GPIO, 0);   // Step starts low
	
	// Give TMC2209 time to initialize
	vTaskDelay(pdMS_TO_TICKS(100));
	
	bool motor_running = false;
	bool last_button_state = 1; // Released (active low)
	uint32_t step_count = 0;
	uint32_t current_delay_us = STEP_DELAY_START_US; // Current step delay for acceleration
	
	while (1) {
		// Read button state (active low - pressed = 0)
		int button_state = gpio_get_level(BUTTON_GPIO);
		
#if USE_TOGGLE_MODE
		// Toggle mode: press button to start, press again to stop
		// Detect falling edge (button press)
		if (last_button_state == 1 && button_state == 0) {
			// Button just pressed - toggle motor state
			vTaskDelay(pdMS_TO_TICKS(50)); // Debounce
			if (gpio_get_level(BUTTON_GPIO) == 0) { // Still pressed after debounce
				motor_running = !motor_running;
				if (motor_running) {
					ESP_LOGI(TAG, "Motor ON - press button again to stop");
					gpio_set_level(STEPPER_ENABLE_GPIO, 0); // Enable motor
					step_count = 0;
					current_delay_us = STEP_DELAY_START_US; // Reset to slow start
				} else {
					ESP_LOGI(TAG, "Motor OFF (total steps: %lu, %.1f revolutions)", 
					         step_count, (float)step_count / STEPS_PER_REVOLUTION);
					gpio_set_level(STEPPER_ENABLE_GPIO, 1); // Disable motor
					gpio_set_level(STEPPER_STEP_GPIO, 0);
				}
			}
		}
		last_button_state = button_state;
		
		// Generate steps if motor is running
		if (motor_running) {
#if USE_ACCELERATION
			// Smooth acceleration: gradually decrease delay from START to target
			if (step_count < ACCELERATION_STEPS) {
				// Linear ramp: interpolate from slow to fast
				uint32_t ramp_progress = step_count;
				current_delay_us = STEP_DELAY_START_US - 
				                   ((STEP_DELAY_START_US - STEP_DELAY_US) * ramp_progress / ACCELERATION_STEPS);
			} else {
				current_delay_us = STEP_DELAY_US; // Full speed
			}
#else
			current_delay_us = STEP_DELAY_US; // No acceleration
#endif
			
			// Generate step pulse
			gpio_set_level(STEPPER_STEP_GPIO, 1);
			esp_rom_delay_us(PULSE_WIDTH_US);
			gpio_set_level(STEPPER_STEP_GPIO, 0);
			esp_rom_delay_us(current_delay_us - PULSE_WIDTH_US);
			
			step_count++;
			
			// Log every 1600 steps (1 revolution)
			if (step_count % 1600 == 0) {
				ESP_LOGI(TAG, "Steps: %lu (%.1f rev) @ %lu µs/step", step_count, 
				         (float)step_count / STEPS_PER_REVOLUTION, current_delay_us);
			}
		} else {
			vTaskDelay(pdMS_TO_TICKS(10)); // No stepping, just check button
		}
#else
		// Hold mode: hold button to run motor
		if (button_state == 0) { // Button pressed
			if (!motor_running) {
				ESP_LOGI(TAG, "Button pressed - enabling motor");
				gpio_set_level(STEPPER_ENABLE_GPIO, 0); // Enable motor (active LOW)
				vTaskDelay(pdMS_TO_TICKS(1)); // Give driver 1ms to wake up
				motor_running = true;
				step_count = 0;
				current_delay_us = STEP_DELAY_START_US;
			}
			
#if USE_ACCELERATION
			// Smooth acceleration
			if (step_count < ACCELERATION_STEPS) {
				uint32_t ramp_progress = step_count;
				current_delay_us = STEP_DELAY_START_US - 
				                   ((STEP_DELAY_START_US - STEP_DELAY_US) * ramp_progress / ACCELERATION_STEPS);
			} else {
				current_delay_us = STEP_DELAY_US;
			}
#else
			current_delay_us = STEP_DELAY_US;
#endif
			
			// Generate step pulse
			gpio_set_level(STEPPER_STEP_GPIO, 1);
			esp_rom_delay_us(PULSE_WIDTH_US);
			gpio_set_level(STEPPER_STEP_GPIO, 0);
			esp_rom_delay_us(current_delay_us - PULSE_WIDTH_US);
			
			step_count++;
			
			// Log progress every 1600 steps (1 revolution)
			if (step_count % 1600 == 0) {
				ESP_LOGI(TAG, "Steps: %lu (%.1f rev) @ %lu µs/step", step_count, 
				         (float)step_count / STEPS_PER_REVOLUTION, current_delay_us);
			}
			
		} else { // Button released
			if (motor_running) {
				ESP_LOGI(TAG, "Button released - disabling motor (total steps: %lu)", step_count);
				gpio_set_level(STEPPER_ENABLE_GPIO, 1); // Disable motor
				gpio_set_level(STEPPER_STEP_GPIO, 0);   // Ensure step is low
				motor_running = false;
				step_count = 0;
			}
			vTaskDelay(pdMS_TO_TICKS(10)); // Check button every 10ms when not pressed
		}
#endif
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
	ESP_LOGI(TAG, "  Stepper ENABLE: GPIO%d (active LOW - TMC2209)", STEPPER_ENABLE_GPIO);
	ESP_LOGI(TAG, "  Stepper STEP: GPIO%d", STEPPER_STEP_GPIO);
	ESP_LOGI(TAG, "  Button: GPIO%d (active low)", BUTTON_GPIO);
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "TMC2209 Configuration:");
	ESP_LOGI(TAG, "  - Microstepping: 1/8 step (1600 steps/rev) - VERY SMOOTH");
	ESP_LOGI(TAG, "  - Hardware: Leave MS1 and MS2 OPEN (not connected)");
	ESP_LOGI(TAG, "  - Target speed: 30 RPM (0.5 rev/sec) with acceleration ramp");
	ESP_LOGI(TAG, "  - Control: %s", USE_TOGGLE_MODE ? "Toggle mode (tap to start/stop)" : "Hold mode");
	ESP_LOGI(TAG, "  - Adjust VREF potentiometer for motor current");
	ESP_LOGI(TAG, "  - Motor: 4.1V rated, typical 1-2A");
	ESP_LOGI(TAG, "  - Recommended VREF: 1.2V - 1.8V for smooth low-speed operation");
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "Microstepping options (see comments in code for full table):");
	ESP_LOGI(TAG, "  - MS1=Open, MS2=Open: 1/8 step (1600 steps/rev) <- CURRENT");
	ESP_LOGI(TAG, "  - MS1=VIO,  MS2=VIO:  Full step (200 steps/rev) - fastest");
	ESP_LOGI(TAG, "  - MS1=VIO,  MS2=Open: 1/4 step (800 steps/rev) - balanced");
	ESP_LOGI(TAG, "");
	ESP_LOGI(TAG, "Speed tuning: Change STEP_DELAY_US in code");
	ESP_LOGI(TAG, "  - 3333µs = 30 RPM (current - very smooth)");
	ESP_LOGI(TAG, "  - 2000µs = 50 RPM (faster)");
	ESP_LOGI(TAG, "  - 1250µs = 80 RPM (fast)");

	// =============================
	// Create FreeRTOS tasks
	// =============================
	xTaskCreate(rgb_blink_task, "rgb_blink", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
	xTaskCreate(stepper_motor_task, "stepper_motor", 2048, NULL, tskIDLE_PRIORITY + 2, NULL);
	
	ESP_LOGI(TAG, "All tasks started");
}