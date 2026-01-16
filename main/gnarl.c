#include "gnarl.h"
#include "module.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#ifdef BLE_SLEEP
#include <esp_sleep.h>
#include <driver/uart.h>
#endif

#include "4b6b.h"
#include "adc.h"
#include "commands.h"
#include "led.h"
#include "rfm95.h"

#include <nvs_flash.h>
#include <nvs.h>

#ifdef LED_IS_WS2812
#include "led_strip.h"
static led_strip_handle_t led_strip = NULL;
#endif

#include "pump_config.h"

#define MAX_PARAM_LEN	32
#define MAX_PACKET_LEN	256

#define QUEUE_LENGTH		40
static QueueHandle_t request_queue = NULL;

// NVS storage
#define NVS_NAMESPACE "pickle"
#define NVS_LED_MODE_KEY "led_mode"
#define NVS_FREQ_KEY "radio_freq"

// LED mode management
uint8_t current_led_mode = LED_MODE_OFF; // Default to OFF mode
static uint8_t fr[3]; // Virtual registers for frequency

// Helper to update fr[] from frequency in Hz (inverse of check_frequency logic)
static void update_virtual_registers(uint32_t freq_hz) {
	// freq = (f * 24MHz) >> 16
	// f = (freq << 16) / 24MHz
	uint64_t f = ((uint64_t)freq_hz << 16) / (24 * MHz);
	fr[0] = (f >> 16) & 0xFF;
	fr[1] = (f >> 8) & 0xFF;
	fr[2] = f & 0xFF;
}

// Frequency saving logic
static uint32_t current_frequency = PUMP_FREQUENCY;
static uint32_t last_saved_frequency = PUMP_FREQUENCY;
static uint64_t last_freq_change_time = 0;

// Save frequency to NVS
// WARNING: NVS operations can block - only call when radio/BLE are idle
static void gnarl_save_frequency(uint32_t freq) {
	nvs_handle_t nvs_handle;
	esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err == ESP_OK) {
		err = nvs_set_u32(nvs_handle, NVS_FREQ_KEY, freq);
		if (err != ESP_OK) {
			ESP_LOGE(TAG, "nvs_set_u32 failed: %s", esp_err_to_name(err));
			nvs_close(nvs_handle);
			return;
		}
		err = nvs_commit(nvs_handle);
		nvs_close(nvs_handle);
		if (err == ESP_OK) {
			ESP_LOGI(TAG, "Saved frequency %lu Hz to NVS", (unsigned long)freq);
			last_saved_frequency = freq;
		} else {
			ESP_LOGE(TAG, "nvs_commit failed: %s", esp_err_to_name(err));
		}
	} else {
		ESP_LOGE(TAG, "gnarl_save_frequency: nvs_open failed: %s", esp_err_to_name(err));
	}
}

// Check if we need to save frequency (called periodically from loop)
static void check_save_frequency(void) {
	if (current_frequency != last_saved_frequency) {
		uint64_t now = esp_timer_get_time();
		// Save 1 minute (60,000,000 us) after last change
		// Must verify queue is empty to avoid blocking during active communication
		// Also ensure BLE is NOT connected to avoid NVS/radio conflicts that can hang the stack
		if ((now - last_freq_change_time > 60000000) &&
			(request_queue != NULL && uxQueueMessagesWaiting(request_queue) == 0) &&
			!ble_is_connected()) {
			gnarl_save_frequency(current_frequency);
		}
	}
}

#ifdef BLE_SLEEP
// Light sleep when BLE is disconnected to save power (~0.8mA vs ~20-40mA)
// Wakeup sources: timer (5 sec) or any GPIO/BLE activity
#define LIGHT_SLEEP_DURATION_US (5 * 1000 * 1000)  // 5 seconds

static void enter_light_sleep_if_idle(void) {
	// Only sleep if: BLE fully initialized, BLE disconnected, and no pending commands
	if (ble_is_ready() && !ble_is_connected() &&
		(request_queue == NULL || uxQueueMessagesWaiting(request_queue) == 0)) {

		// Wait for UART TX to complete before sleep
		uart_wait_tx_idle_polling(UART_NUM_0);

		// Configure timer wakeup
		esp_sleep_enable_timer_wakeup(LIGHT_SLEEP_DURATION_US);

		// Enter light sleep - CPU stops, RAM retained
		// Wakes up on: timer, BLE activity (NimBLE handles this internally)
		esp_light_sleep_start();

		// Disable timer wakeup after waking
		esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
	}
}
#endif // BLE_SLEEP

// Load frequency from NVS (or return default), updates virtual registers
uint32_t gnarl_load_frequency(void) {
	nvs_handle_t nvs_handle;
	uint32_t freq = PUMP_FREQUENCY;
	esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
	if (err == ESP_OK) {
		if (nvs_get_u32(nvs_handle, NVS_FREQ_KEY, &freq) == ESP_OK) {
			ESP_LOGI(TAG, "Loaded frequency %lu Hz from NVS", (unsigned long)freq);
		} else {
			ESP_LOGI(TAG, "No saved frequency found, using default %lu Hz", (unsigned long)freq);
		}
		nvs_close(nvs_handle);
	} else {
		ESP_LOGW(TAG, "gnarl_load_frequency: nvs_open failed: %s, using default", esp_err_to_name(err));
	}
	// Update state variables
	current_frequency = freq;
	last_saved_frequency = freq;
	last_freq_change_time = esp_timer_get_time();
	// Ensure virtual registers are in sync with what we loaded
	update_virtual_registers(freq);
	return freq;
}

// Load LED mode from NVS
void led_load_mode(void) {
	// Default to OFF mode
	current_led_mode = LED_MODE_OFF;

	nvs_handle_t nvs_handle;
	esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
	if (err != ESP_OK) {
		ESP_LOGW(TAG, "led_load_mode: nvs_open failed: %s (using default OFF)", esp_err_to_name(err));
		return;
	}

	uint8_t saved_mode;
	err = nvs_get_u8(nvs_handle, NVS_LED_MODE_KEY, &saved_mode);
	nvs_close(nvs_handle);

	if (err == ESP_OK) {
		// Validate saved mode - only OFF or ON allowed
		if (saved_mode == LED_MODE_OFF || saved_mode == LED_MODE_ON) {
			current_led_mode = saved_mode;
			ESP_LOGI(TAG, "LED mode LOADED from NVS: 0x%02X (%s)",
					 saved_mode,
					 saved_mode == LED_MODE_OFF ? "OFF" : "ON");
		} else {
			ESP_LOGW(TAG, "Invalid LED mode in NVS: 0x%02X, using default OFF", saved_mode);
		}
	} else if (err == ESP_ERR_NVS_NOT_FOUND) {
		ESP_LOGI(TAG, "No saved LED mode found, using default OFF");
	} else {
		ESP_LOGW(TAG, "led_load_mode: nvs_get_u8 failed: %s (using default OFF)", esp_err_to_name(err));
	}
}

// Save LED mode to NVS
static void led_save_mode(uint8_t mode) {
	nvs_handle_t nvs_handle;
	esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "led_save_mode: nvs_open failed: %s", esp_err_to_name(err));
		return;
	}
	
	err = nvs_set_u8(nvs_handle, NVS_LED_MODE_KEY, mode);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "led_save_mode: nvs_set_u8 failed: %s", esp_err_to_name(err));
	}
	
	err = nvs_commit(nvs_handle);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "led_save_mode: nvs_commit failed: %s", esp_err_to_name(err));
	}
	
	nvs_close(nvs_handle);
	ESP_LOGI(TAG, "LED mode saved to NVS: 0x%02X", mode);
}

#ifdef LED_IS_WS2812
void led_on(void) {
	if (!led_strip) return;
	// Low-power white pulse
	led_strip_set_pixel(led_strip, 0, 16, 16, 16);
	led_strip_refresh(led_strip);
}

void led_off(void) {
	if (!led_strip) return;
	led_strip_clear(led_strip);
	led_strip_refresh(led_strip);
}
#endif

// Apply LED mode without saving to NVS
static void led_apply_mode(uint8_t mode) {
	current_led_mode = mode;
	// OFF = LED always off, ON = LED blinks on activity
	led_off();
}

// Set LED mode and save to NVS (called from CmdLED)
void led_set_mode(uint8_t mode) {
	led_apply_mode(mode);
	led_save_mode(mode); // Save to NVS
}

// LED task queue and handle
#define LED_QUEUE_LENGTH 10
static QueueHandle_t led_queue = NULL;

// Activity indicator functions - send events to LED task (non-blocking)
void led_indicate_radio_tx(void) {
	if (led_queue != NULL && current_led_mode == LED_MODE_ON) {
		led_event_t event = LED_EVENT_RADIO_TX;
		xQueueSend(led_queue, &event, 0); // Don't block if queue full
	}
}

void led_indicate_radio_rx(void) {
	if (led_queue != NULL && current_led_mode == LED_MODE_ON) {
		led_event_t event = LED_EVENT_RADIO_RX;
		xQueueSend(led_queue, &event, 0);
	}
}

void led_indicate_ble_activity(void) {
	if (led_queue != NULL && current_led_mode == LED_MODE_ON) {
		led_event_t event = LED_EVENT_BLE;
		xQueueSend(led_queue, &event, 0);
	}
}

// LED task - handles blink events with proper delays in separate task
static void led_task(void *unused) {
	led_event_t event;
	
	for (;;) {
		if (xQueueReceive(led_queue, &event, portMAX_DELAY)) {
			// Blink in ON mode
			if (current_led_mode == LED_MODE_ON) {
				led_on();
				vTaskDelay(pdMS_TO_TICKS(30)); // 30ms pulse - safe in separate task
				led_off();
			}
		}
	}
}

// Initialize LED GPIO and task
void led_init(void) {
#ifdef LED_IS_WS2812
	// Initialize WS2812 LED strip (single pixel)
	led_strip_config_t strip_config = {
		.strip_gpio_num = LED,
		.max_leds = 1,
		.led_model = LED_MODEL_WS2812,
		.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
		.flags = {.invert_out = false},
	};
	led_strip_rmt_config_t rmt_config = {
		.resolution_hz = 10 * 1000 * 1000, // 10MHz
		.mem_block_symbols = 0,
		.flags = {.with_dma = 0},
	};
	if (led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip) != ESP_OK) {
		ESP_LOGE(TAG, "Failed to create WS2812 LED strip object");
		led_strip = NULL;
	} else {
		led_strip_clear(led_strip);
		led_strip_refresh(led_strip);
	}
#else
	gpio_set_direction(LED, GPIO_MODE_OUTPUT);
	gpio_set_level(LED, 0);
#endif
	
	// Create LED event queue
	led_queue = xQueueCreate(LED_QUEUE_LENGTH, sizeof(led_event_t));
	if (led_queue == NULL) {
		ESP_LOGE(TAG, "Failed to create LED queue");
		return;
	}
	
	// Create LED task with low priority
	xTaskCreate(led_task, "LED", 2048, NULL, tskIDLE_PRIORITY + 1, NULL);
	ESP_LOGI(TAG, "LED task started");
} 

typedef enum {
	ENCODING_NONE = 0,
	ENCODING_4B6B = 2,
} encoding_type_t;

static encoding_type_t encoding_type = ENCODING_NONE;

typedef enum CommandCode rfspy_cmd_t;

typedef struct {
	rfspy_cmd_t command;
	int length;
	int rssi;
	uint8_t data[MAX_PARAM_LEN + MAX_PACKET_LEN];
} rfspy_request_t;


typedef struct __attribute__((packed)) {
	uint8_t listen_channel;
	uint32_t timeout_ms;
} get_packet_cmd_t;

typedef struct __attribute__((packed)) {
	uint8_t send_channel;
	uint8_t repeat_count;
	uint16_t delay_ms;
	uint8_t packet[];
} send_packet_cmd_t;

typedef struct __attribute__((packed)) {
	uint8_t send_channel;
	uint8_t repeat_count;
	uint16_t delay_ms;
	uint8_t listen_channel;
	uint32_t timeout_ms;
	uint8_t retry_count;
	uint16_t preamble_ms;
	uint8_t packet[];
} send_and_listen_cmd_t;

typedef struct __attribute__((packed)) {
	uint8_t rssi;
	uint8_t packet_count;
	uint8_t packet[MAX_PACKET_LEN];
} response_packet_t;

typedef struct __attribute__((packed)) {
	uint32_t uptime;
	uint16_t rx_overflow;
	uint16_t rx_fifo_overflow;
	uint16_t packet_rx_count;
	uint16_t packet_tx_count;
	uint16_t crc_failure_count;
	uint16_t spi_sync_failure_count;
	uint16_t placeholder0;
	uint16_t placeholder1;
} statistics_cmd_t;

static statistics_cmd_t statistics;
static response_packet_t rx_buf;

static inline void swap_bytes(uint8_t *p, uint8_t *q) {
	uint8_t t = *p;
	*p = *q;
	*q = t;
}

// Safe byte-swap for potentially unaligned packed struct members
static inline void reverse_two_bytes(void *ptr) {
	uint8_t tmp[2];
	memcpy(tmp, ptr, 2);
	uint8_t swap = tmp[0];
	tmp[0] = tmp[1];
	tmp[1] = swap;
	memcpy(ptr, tmp, 2);
}

static inline void reverse_four_bytes(void *ptr) {
	uint8_t tmp[4];
	memcpy(tmp, ptr, 4);
	uint8_t swap = tmp[0];
	tmp[0] = tmp[3];
	tmp[3] = swap;
	swap = tmp[1];
	tmp[1] = tmp[2];
	tmp[2] = swap;
	memcpy(ptr, tmp, 4);
}

// Buffer for 4b6b encoding/decoding
// Encoding: 2 input bytes → 3 output bytes, so N bytes → (N * 3 / 2) + 1 bytes
// Decoding: 3 input bytes → 2 output bytes, so N bytes → (N * 2 / 3) + 1 bytes
// MAX_PACKET_LEN (256) bytes encoded → 385 bytes (worst case)
// MAX_PACKET_LEN (256) bytes decoded → 171 bytes (worst case)
// Use the larger size to handle both operations safely
#define PKT_BUF_SIZE ((MAX_PACKET_LEN * 3 / 2) + 2)  // 386 bytes
static uint8_t pkt_buf[PKT_BUF_SIZE];

static void send(uint8_t *data, int len, int repeat_count, int delay_ms) {
	if (len > 0 && data[len-1] == 0) {
		len--;
	}
	// Validate input length to prevent buffer overflow
	if (len > MAX_PACKET_LEN) {
		ESP_LOGE(TAG, "send: packet too large (%d > %d)", len, MAX_PACKET_LEN);
		send_code(RESPONSE_CODE_PARAM_ERROR);
		return;
	}
	switch (encoding_type) {
	case ENCODING_NONE:
		break;
	case ENCODING_4B6B:
		len = encode_4b6b(data, pkt_buf, len);
		data = pkt_buf;
		break;
	default:
		ESP_LOGE(TAG, "send: unknown encoding type %d", encoding_type);
		send_code(RESPONSE_CODE_PARAM_ERROR);
		return;
	}
	print_bytes("TX: sending %d bytes:", data, len);
	led_indicate_radio_tx();
	{
		int64_t t0 = esp_timer_get_time();
		ESP_LOGI(TAG, "radio TX start: %d bytes at %lld us", len, (long long)t0);
		transmit(data, len);
		int64_t t1 = esp_timer_get_time();
		ESP_LOGI(TAG, "radio TX done: %d bytes duration %lld ms", len, (long long)((t1 - t0) / 1000));
		// Yield briefly after TX to give NimBLE and other tasks CPU time
		vTaskDelay(1);
	}
	while (repeat_count > 0) {
		vTaskDelay(pdMS_TO_TICKS(delay_ms));
		led_indicate_radio_tx();
		transmit(data, len);
		// Yield to other tasks to avoid WDT starvation if repeat_count is large
		vTaskDelay(1);
		repeat_count--;
	}
}

// Transform an RSSI value back into the raw encoding that the TI CC111x radios use.
// See section 13.10.3 of the CC1110 data sheet.
static uint8_t raw_rssi(int rssi) {
	const int rssi_offset = 73;
	return (rssi + rssi_offset) * 2;
}

static void rx_common(int n, int rssi) {
	if (n == 0) {
		ESP_LOGD(TAG, "RX: timeout");
		send_code(RESPONSE_CODE_RX_TIMEOUT);
		return;
	}
	// Validate received length
	if (n < 0 || n > MAX_PACKET_LEN) {
		ESP_LOGE(TAG, "RX: invalid packet length %d", n);
		send_code(RESPONSE_CODE_RX_TIMEOUT);
		return;
	}
	// Truncate oversized packets to prevent BLE buffer overflow
	if (n > 256) {
		ESP_LOGW(TAG, "RX: truncating packet from %d to 256 bytes", n);
		n = 256;
	}
	// Filter out noise/idle patterns: packets consisting mostly of 0x55 (RF idle state)
	if (n > 10) {
		int noise_count = 0;
		for (int i = 0; i < n; i++) {
			if (rx_buf.packet[i] == 0x55) noise_count++;
		}
		// If > 90% of packet is 0x55, it's noise
		if (noise_count > (n * 9 / 10)) {
			ESP_LOGW(TAG, "RX: discarding noise packet (%d/%d bytes are 0x55)", noise_count, n);
			send_code(RESPONSE_CODE_RX_TIMEOUT);
			return;
		}
	}
	// Minimum valid Medtronic packet is ~10 bytes (preamble + sync + data)
	if (n < 10) {
		ESP_LOGD(TAG, "RX: packet too short (%d bytes)", n);
		send_code(RESPONSE_CODE_RX_TIMEOUT);
		return;
	}
	rx_buf.rssi = raw_rssi(rssi);
	if (rx_buf.rssi == 0) {
		rx_buf.rssi = 1;
	}
	rx_buf.packet_count = rx_packet_count();
	if (rx_buf.packet_count == 0) {
		rx_buf.packet_count = 1;
	}
	int d;
	switch (encoding_type) {
	case ENCODING_NONE:
		break;
	case ENCODING_4B6B:
		d = decode_4b6b(rx_buf.packet, pkt_buf, n);
		if (d != -1) {
			memcpy(rx_buf.packet, pkt_buf, d);
			n = d;
		}
		break;
	default:
		ESP_LOGE(TAG, "RX: unknown encoding type %d", encoding_type);
		send_code(RESPONSE_CODE_PARAM_ERROR);
		return;
	}
	print_bytes("RX: received %d bytes:", rx_buf.packet, n);
	led_indicate_radio_rx();
	send_bytes((uint8_t *)&rx_buf, 2 + n);
}

static void get_packet(const uint8_t *buf, int len) {
	get_packet_cmd_t *p = (get_packet_cmd_t *)buf;
	reverse_four_bytes(&p->timeout_ms);
	ESP_LOGD(TAG, "get_packet: listen_channel %d timeout_ms %lu",
		 p->listen_channel, (unsigned long)p->timeout_ms);
	int n = receive(rx_buf.packet, sizeof(rx_buf.packet), p->timeout_ms);
	rx_common(n, read_rssi());
}

static void send_packet(const uint8_t *buf, int len) {
	send_packet_cmd_t *p = (send_packet_cmd_t *)buf;
	reverse_two_bytes(&p->delay_ms);
	ESP_LOGD(TAG, "send_packet: len %d send_channel %d repeat_count %d delay_ms %d",
		 len, p->send_channel, p->repeat_count, p->delay_ms);
	len -= (p->packet - (uint8_t *)p);
	send(p->packet, len, p->repeat_count, p->delay_ms);
	send_code(RESPONSE_CODE_SUCCESS);
}

static void send_and_listen(const uint8_t *buf, int len) {
	send_and_listen_cmd_t *p = (send_and_listen_cmd_t *)buf;
	reverse_two_bytes(&p->delay_ms);
	reverse_four_bytes(&p->timeout_ms);
	reverse_two_bytes(&p->preamble_ms);
	ESP_LOGD(TAG, "send_and_listen: len %d send_channel %d repeat_count %d delay_ms %d",
		 len, p->send_channel, p->repeat_count, p->delay_ms);
	ESP_LOGD(TAG, "send_and_listen: listen_channel %d timeout_ms %lu retry_count %d",
		 p->listen_channel, (unsigned long)p->timeout_ms, p->retry_count);
	
	// Fix timeout for wakeup commands - iAPS sometimes sends 1ms timeout
	// DISABLED: Extending timeout causes iAPS client-side timeout error
	// iAPS expects quick response and has its own retry logic with longer timeouts
	/*
	len -= (p->packet - (uint8_t *)p);
	if (len >= 8 && p->timeout_ms < 100) {
		// Decode command byte after preamble (6 bytes)
		// Packet format: [preamble 6 bytes][encoded_cmd][params]
		uint8_t decoded[2];
		int dec_len = decode_4b6b(&p->packet[6], decoded, 3);  // Decode from offset 6 (after preamble)
		if (dec_len > 0 && decoded[0] == 0x5D) {  // CMD_WAKEUP
			ESP_LOGW(TAG, "Wakeup detected with short timeout (%lu ms), extending to 500 ms", 
				(unsigned long)p->timeout_ms);
			p->timeout_ms = 500;
		}
	}
	*/
	
	len -= (p->packet - (uint8_t *)p);


	send(p->packet, len, p->repeat_count, p->delay_ms);
	int64_t r0 = esp_timer_get_time();
	int n = receive(rx_buf.packet, sizeof(rx_buf.packet), p->timeout_ms);
	int64_t r1 = esp_timer_get_time();
	ESP_LOGI(TAG, "radio RX: got %d bytes in %lld ms (timeout %lu ms)", n, (long long)((r1 - r0) / 1000), (unsigned long)p->timeout_ms);
	int rssi = read_rssi();
	for (int retries = p->retry_count; retries > 0; retries--) {
		if (n != 0) {
			break;
		}
		send(p->packet, len, p->repeat_count, p->delay_ms);
		n = receive(rx_buf.packet, sizeof(rx_buf.packet), p->timeout_ms);
		rssi = read_rssi();
	}
	rx_common(n, rssi);
}


// static uint8_t fr[3]; // Moved to top of file

static inline bool valid_frequency(uint32_t f) {
	if (863*MHz <= f && f <= 870*MHz) {
		return true;
	}
	if (910*MHz <= f && f <= 920*MHz) {
		return true;
	}
	return false;
}

// Change the radio frequency if the current register values make sense.
static void check_frequency(void) {
	uint32_t f = ((uint32_t)fr[0] << 16) + ((uint32_t)fr[1] << 8) + ((uint32_t)fr[2]);
	uint32_t freq = (uint32_t)(((uint64_t)f * 24*MHz) >> 16);
	if (valid_frequency(freq)) {
		ESP_LOGI(TAG, "setting frequency to %lu Hz", (unsigned long)freq);
		set_frequency(freq);
		
		if (current_frequency != freq) {
			current_frequency = freq;
			last_freq_change_time = esp_timer_get_time();
		}
	} else {
		ESP_LOGD(TAG, "invalid frequency (%lu Hz)", (unsigned long)freq);
	}
}

static void update_register(const uint8_t *buf, int len) {
	// AAPS sends 2 bytes, Loop sends 10
	if (len < 2) {
		ESP_LOGE(TAG, "update_register: len = %d", len);
		return;
	}
	uint8_t addr = buf[0];
	uint8_t value = buf[1];
	ESP_LOGD(TAG, "update_register: addr %02X value %02X", addr, value);
	switch (addr) {
	case 0x09 ... 0x0B:
		fr[addr - 0x09] = value;
		check_frequency();
		break;
	default:
		ESP_LOGD(TAG, "update_register: addr %02X ignored", addr);
		break;
	}
	send_code(RESPONSE_CODE_SUCCESS);
}

static void read_register(const uint8_t *buf, int len) {
	uint8_t addr = buf[0];
	uint8_t value = 0;
	switch (addr) {
	case 0x09 ... 0x0B:
		value = fr[addr - 0x09];
		break;
	}
	ESP_LOGD(TAG, "read_register: addr %02X value %02X", addr, value);
	send_bytes(&value, sizeof(value));
}

static void led_mode(const uint8_t *buf, int len) {
	if (len < 2) {
		ESP_LOGE(TAG, "led_mode: len = %d", len);
		return;
	}
	uint8_t led = buf[0];
	uint8_t mode = buf[1];
	
	// Normalize mode: 0x00 = OFF, any non-zero = ON
	uint8_t normalized_mode = (mode == 0x00) ? LED_MODE_OFF : LED_MODE_ON;
	
	ESP_LOGI(TAG, "led_mode: led %02X mode %02X -> %s", 
			 led, mode,
			 normalized_mode == LED_MODE_OFF ? "OFF" : "ON");
	
	// CmdLED sets mode: 0x00 = OFF, anything else = ON
	if (led == 0 || led == 1) {
		led_set_mode(normalized_mode);
	}
	
	send_code(RESPONSE_CODE_SUCCESS);
}

static void send_stats(void) {
	statistics.uptime = xTaskGetTickCount();
	// From rfm95:
	statistics.packet_rx_count = rx_packet_count();
	statistics.packet_tx_count = tx_packet_count();
	// Use placeholders for battery info (keeps protocol size stable).
	// placeholder0: battery voltage in mV, placeholder1: battery percent (0..100).
	statistics.placeholder0 = (uint16_t)get_battery_voltage();
	statistics.placeholder1 = (uint16_t)battery_percent(get_battery_voltage());
	ESP_LOGD(TAG, "send_stats len %d uptime %lu rx %d tx %d batt %d mV (%d%%)",
		 sizeof(statistics), (unsigned long)statistics.uptime,
		 statistics.packet_rx_count, statistics.packet_tx_count,
		 statistics.placeholder0, statistics.placeholder1);
	
	// Use local copies to avoid taking address of packed struct members
	uint32_t uptime = statistics.uptime;
	uint16_t rx_count = statistics.packet_rx_count;
	uint16_t tx_count = statistics.packet_tx_count;
	uint16_t ph0 = statistics.placeholder0;
	uint16_t ph1 = statistics.placeholder1;
	reverse_four_bytes(&uptime);
	reverse_two_bytes(&rx_count);
	reverse_two_bytes(&tx_count);
	reverse_two_bytes(&ph0);
	reverse_two_bytes(&ph1);
	statistics.uptime = uptime;
	statistics.packet_rx_count = rx_count;
	statistics.packet_tx_count = tx_count;
	statistics.placeholder0 = ph0;
	statistics.placeholder1 = ph1;
	
	send_bytes((const uint8_t *)&statistics, sizeof(statistics));
}

static void set_sw_encoding(const uint8_t *buf, int len) {
	ESP_LOGD(TAG, "encoding mode %02X", buf[0]);
	switch (buf[0]) {
	case ENCODING_NONE:
	case ENCODING_4B6B:
		encoding_type = buf[0];
		break;
	default:
		send_code(RESPONSE_CODE_PARAM_ERROR);
		return;
	}
	send_code(RESPONSE_CODE_SUCCESS);
}

void rfspy_command(const uint8_t *buf, int count, int rssi) {
	if (count == 0) {
		ESP_LOGE(TAG, "rfspy_command: count == 0");
		return;
	}
	if (buf[0] != count - 1 || count == 1) {
		ESP_LOGE(TAG, "rfspy_command: length = %d, byte 0 == %d", count, buf[0]);
		return;
	}
	// Validate buffer size to prevent overflow
	if (count - 2 > MAX_PARAM_LEN + MAX_PACKET_LEN) {
		ESP_LOGE(TAG, "rfspy_command: payload too large (%d > %d)", 
			count - 2, MAX_PARAM_LEN + MAX_PACKET_LEN);
		return;
	}
	rfspy_cmd_t cmd = buf[1];
	
	// All commands go through the queue to ensure proper task synchronization
	// (especially important for SPI operations like CmdUpdateRegister)
	rfspy_request_t req = {
		.command = cmd,
		.length = count - 2,
		.rssi = rssi,
	};
	memcpy(req.data, buf + 2, req.length);
	if (!xQueueSend(request_queue, &req, 0)) {
		ESP_LOGE(TAG, "rfspy_command: cannot queue request for command %d", cmd);
		return;
	}
	ESP_LOGD(TAG, "rfspy_command %d, queue length %d", cmd, uxQueueMessagesWaiting(request_queue));
}

static void gnarl_loop(void *unused) {
	ESP_LOGD(TAG, "starting gnarl_loop");
	const int timeout_ms = 30*MILLISECONDS;
	for (;;) {
		// Periodically check if we need to save new frequency
		check_save_frequency();

#ifdef BLE_SLEEP
		// Enter light sleep when BLE disconnected to save power
		enter_light_sleep_if_idle();
#endif

		rfspy_request_t req;
		if (!xQueueReceive(request_queue, &req, pdMS_TO_TICKS(timeout_ms))) {
			continue;
		}
		switch (req.command) {
		case CmdGetState:
			ESP_LOGI(TAG, "CmdGetState");
			send_bytes((const uint8_t *)STATE_OK, strlen(STATE_OK));
			break;
		case CmdGetVersion:
			ESP_LOGI(TAG, "CmdGetVersion");
			send_bytes((const uint8_t *)SUBG_RFSPY_VERSION, strlen(SUBG_RFSPY_VERSION));
			break;
		case CmdGetPacket:
			ESP_LOGI(TAG, "CmdGetPacket");
			get_packet(req.data, req.length);
			break;
		case CmdSendPacket:
			ESP_LOGI(TAG, "CmdSendPacket");
			send_packet(req.data, req.length);
			break;
		case CmdSendAndListen:
			ESP_LOGI(TAG, "CmdSendAndListen");
			send_and_listen(req.data, req.length);
			break;
		case CmdUpdateRegister:
			ESP_LOGI(TAG, "CmdUpdateRegister");
			update_register(req.data, req.length);
			break;
		case CmdReadRegister:
			ESP_LOGI(TAG, "CmdReadRegister");
			read_register(req.data, req.length);
			break;
		case CmdReset:
			ESP_LOGI(TAG, "CmdReset - saving state and restarting device");
			// Сохранить частоту в NVS (если изменилась)
			if (current_frequency != last_saved_frequency) {
				gnarl_save_frequency(current_frequency);
			}
			// Сохранить LED mode в NVS
			led_save_mode(current_led_mode);
			// Отправить SUCCESS перед рестартом
			send_code(RESPONSE_CODE_SUCCESS);
			// Дать время на отправку ответа по BLE (~100ms)
			vTaskDelay(pdMS_TO_TICKS(100));
			// Программный рестарт
			esp_restart();
			break;
		case CmdLED:
			ESP_LOGI(TAG, "CmdLED");
			led_mode(req.data, req.length);
			break;
		case CmdSetSWEncoding:
			ESP_LOGI(TAG, "CmdSetSWEncoding");
			set_sw_encoding(req.data, req.length);
			break;
		case CmdResetRadioConfig:
			ESP_LOGI(TAG, "CmdResetRadioConfig");
			send_code(RESPONSE_CODE_SUCCESS);
			break;
		case CmdGetStatistics:
			ESP_LOGI(TAG, "CmdGetStatistics");
			send_stats();
			break;
		default:
			ESP_LOGE(TAG, "unimplemented rfspy command %d", req.command);
			break;
		}
		// Yield to IDLE task to prevent WDT starvation if queue is flooded
		vTaskDelay(1);
	}
}

void start_gnarl_task(void) {
	request_queue = xQueueCreate(QUEUE_LENGTH, sizeof(rfspy_request_t));
	// Start radio task with moderate priority. Too high priority can starve NimBLE host and trigger disconnects.
	// Lower priority to avoid starving BLE host task.
	xTaskCreate(gnarl_loop, "PICKL", 8192, 0, tskIDLE_PRIORITY + 1, 0);
	ESP_LOGI(TAG, "gnarl task started with priority %d", tskIDLE_PRIORITY + 1);
}
