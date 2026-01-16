#include "gnarl.h"

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <nvs_flash.h>
#include "adc.h"
#include "led.h"
#include "rfm95.h"

#include "soc/soc.h"
#ifndef CONFIG_IDF_TARGET_ESP32C6
#include "soc/rtc_cntl_reg.h"
#endif
#include <driver/gpio.h>
#include "pump_config.h"
// #define PUMP_FREQUENCY 868250000

// Uncomment to run Sniffer mode instead of normal application
// #define RUN_SNIFFER

#ifdef RUN_SNIFFER
#define SNIFFER_TIMEOUT 600
static uint8_t sniffer_rx_buf[256];
#endif

void app_main(void) {
#ifndef CONFIG_IDF_TARGET_ESP32C6
	// Brownout detector disabled to prevent infinite reboot loops when battery is low.
	// Instead, we use proactive battery monitoring in adc.c:
	// - Battery voltage checked every 60 seconds
	// - At critical level (3300 mV), device enters deep sleep
	// - This prevents battery damage from repeated brownout-triggered reboots
	WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif

	// Increase NimBLE verbosity to help diagnose disconnects
	esp_log_level_set("NimBLE", ESP_LOG_ERROR);
	ESP_LOGI(TAG, "%s", SUBG_RFSPY_VERSION);
	
	// Initialize NVS flash first (required for LED mode and BLE)
	ESP_ERROR_CHECK(nvs_flash_init());

	// Initialize GPIO ISR service (required for radio interrupt handling)
	// Must be called once before rfm95_init()
	ESP_ERROR_CHECK(gpio_install_isr_service(0));

#ifdef RUN_SNIFFER
    ESP_LOGI(TAG, "Starting Sniffer Mode");
    rfm95_init();
	uint8_t v = read_version();
	printf("radio version %d.%d\n", version_major(v), version_minor(v));
	set_frequency(PUMP_FREQUENCY);
	printf("frequency set to %lu Hz\n", read_frequency());
	for (;;) {
		int n = sleep_receive(sniffer_rx_buf, sizeof(sniffer_rx_buf), SNIFFER_TIMEOUT);
		if (n == 0) {
			// printf("[timeout]\n");
			continue;
		}
		for (int i = 0; i < n; i++) {
			printf("%02X ", sniffer_rx_buf[i]);
		}
		printf("(RSSI = %d, # bytes = %d, count = %d)\n", read_rssi(), n, rx_packet_count());
		vTaskDelay(pdMS_TO_TICKS(10)); // Yield to allow IDLE task to reset WDT
	}
#else
	
	// Initialize LED GPIO and task
	led_init();
	// Load saved LED mode from NVS (or use default AUTO)
	led_load_mode();
	// Apply the loaded mode (no need to call led_set_mode - it would re-save)
	// current_led_mode already set by led_load_mode, just apply hardware state
	if (current_led_mode == LED_MODE_OFF) {
		led_off();
	} else if (current_led_mode == LED_MODE_ON) {
		led_on();
	} else {
		led_off(); // AUTO mode starts with LED off
	}
	
	rfm95_init();
	uint8_t v = read_version();
	ESP_LOGD(TAG, "radio version %d.%d", version_major(v), version_minor(v));
	
	// Load saved frequency from NVS (or use default PUMP_FREQUENCY)
	uint32_t startup_freq = gnarl_load_frequency();
	set_frequency(startup_freq);
	ESP_LOGD(TAG, "frequency set to %lu Hz", (unsigned long)read_frequency());
	
	adc_init();
	gnarl_init();
#endif
}
