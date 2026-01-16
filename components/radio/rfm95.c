#include <unistd.h>

#define TAG		"rfm95"

#include <driver/gpio.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "module.h"
#include "rfm95.h"
#include "spi.h"

// Polling mode для RX: опрашиваем регистр SyncMatch вместо ожидания прерывания DIO2.
// На ESP32 Heltec V2 GPIO32 (DIO2) - input-only pin, прерывания работают нестабильно.
// Polling надёжнее и даёт тот же результат (~100-120ms на приём пакета).
#define USE_POLLING

#ifndef USE_POLLING
#include <driver/uart.h>  // Только для sleep mode (uart_wait_tx_idle_polling)
#include <esp_sleep.h>
#include <stdatomic.h>
#endif

#define MILLISECOND	1000

// The FIFO_THRESHOLD value should allow a maximum-sized packet to be
// written in two bursts, but be large enough to avoid fifo underflow.
#define FIFO_THRESHOLD	20

static inline uint8_t read_mode(void) {
	return read_register(REG_OP_MODE) & OP_MODE_MASK;
}

#define MAX_WAIT	1000

static void set_mode(uint8_t mode) {
	uint8_t cur_mode = read_mode();
	if (mode == cur_mode) {
		return;
	}
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | mode);
	ESP_LOGD(TAG, "set_mode %d -> %d", cur_mode, mode);
	if (cur_mode == MODE_SLEEP) {
		usleep(100);
	}
	for (int w = 0; w < MAX_WAIT; w++) {
		cur_mode = read_mode();
		if (cur_mode == mode) {
			return;
		}
		vTaskDelay(1);  // Yield to other tasks
	}
	ESP_LOGI(TAG, "set_mode(%d) timeout in mode %d", mode, cur_mode);

	// Diagnostic dump
	{
		uint8_t op = read_register(REG_OP_MODE);
		uint8_t irq1 = read_register(REG_IRQ_FLAGS_1);
		uint8_t irq2 = read_register(REG_IRQ_FLAGS_2);
		ESP_LOGW(TAG, "Diagnostic: REG_OP_MODE=0x%02X, IRQ1=0x%02X, IRQ2=0x%02X", op, irq1, irq2);
	}

	// Attempt one recovery: hardware reset and retry the mode set
	ESP_LOGW(TAG, "Attempting recovery: reset and retry set_mode(%d)", mode);
	rfm95_reset();
	write_register(REG_OP_MODE, FSK_OOK_MODE | MODULATION_OOK | mode);
	vTaskDelay(pdMS_TO_TICKS(10));
	for (int w = 0; w < (MAX_WAIT/2); w++) {
		cur_mode = read_mode();
		if (cur_mode == mode) {
			ESP_LOGI(TAG, "set_mode(%d) succeeded after recovery", mode);
			return;
		}
		vTaskDelay(1);
	}
	ESP_LOGE(TAG, "set_mode(%d) failed after recovery; current mode=%d", mode, cur_mode);
}

static inline void set_mode_sleep(void) {
	set_mode(MODE_SLEEP);
}

static inline void set_mode_standby(void) {
	set_mode(MODE_STDBY);
}

static inline void set_mode_receive(void) {
	set_mode(MODE_RX);
}

static inline void set_mode_transmit(void) {
	set_mode(MODE_TX);
}

static inline void sequencer_stop(void) {
	write_register(REG_SEQ_CONFIG_1, SEQUENCER_STOP);
}

// Reset the radio device.  See section 7.2.2 of data sheet.
// NOTE: the RFM95 requires the reset pin to be in input mode
// except while resetting the chip, unlike the RFM69 for example.

void rfm95_reset(void) {
	ESP_LOGD(TAG, "reset");
	gpio_set_direction(LORA_RST, GPIO_MODE_OUTPUT);
	gpio_set_level(LORA_RST, 0);
	usleep(100);  // 100us - too short for vTaskDelay, usleep OK
	gpio_set_direction(LORA_RST, GPIO_MODE_INPUT);
	vTaskDelay(pdMS_TO_TICKS(5));  // 5ms delay for reset completion
}

static volatile int rx_packets;

int rx_packet_count(void) {
	return rx_packets;
}

static volatile int tx_packets;

int tx_packet_count(void) {
	return tx_packets;
}

#ifndef USE_POLLING
// Interrupt mode: ISR для пробуждения задачи при SyncMatch на DIO2
static _Atomic TaskHandle_t rx_waiting_task = NULL;

static void IRAM_ATTR rx_interrupt(void *unused) {
	TaskHandle_t task = atomic_load_explicit(&rx_waiting_task, memory_order_acquire);
	if (task != NULL) {
		vTaskNotifyGiveFromISR(task, 0);
	}
}
#endif

void rfm95_init(void) {
	spi_init();

	// Hardware reset and quick diagnostic: verify REG_VERSION to confirm SPI comms
	rfm95_reset();
	ESP_LOGI(TAG, "Using CS pin: %d", LORA_CS);
	uint8_t ver = read_register(REG_VERSION);
	ESP_LOGI(TAG, "RFM95 REG_VERSION = 0x%02X", ver);
	if (ver == 0x00 || ver == 0xFF) {
		ESP_LOGW(TAG, "RFM95 appears not responding (REG_VERSION=0x%02X). Trying another reset...", ver);
		rfm95_reset();
		vTaskDelay(pdMS_TO_TICKS(100));
		ver = read_register(REG_VERSION);
		ESP_LOGI(TAG, "RFM95 REG_VERSION after reset = 0x%02X", ver);
		if (ver == 0x00 || ver == 0xFF) {
			ESP_LOGE(TAG, "RFM95 still unresponsive - check wiring, VCC, GND, CS and RST pins and ensure LORA_CS is not a flash pin (GPIO6)");
		}
	}

	// Enable PA_BOOST for TTGO LoRa32 boards (usually required for TX)
	write_register(REG_PA_CONFIG, 0x8F);

	// Enable LNA with max gain and boost
	write_register(REG_LNA, 0x23);

#ifndef USE_POLLING
	// Interrupt mode: настройка DIO2 для прерывания при SyncMatch
	// Note: gpio_install_isr_service() must be called once in app_main() before rfm95_init()
	gpio_set_direction(LORA_DIO2, GPIO_MODE_INPUT);
	gpio_set_intr_type(LORA_DIO2, GPIO_INTR_POSEDGE);
	gpio_isr_handler_add(LORA_DIO2, rx_interrupt, 0);
#endif
	// DIO2 mapping = 11 = SyncAddress в FSK/OOK Packet mode (Table 28, SX1276 datasheet)
	write_register(REG_DIO_MAPPING_1, 3 << DIO2_MAPPING_SHIFT);

	// Must be in Sleep mode first before the second call can change to FSK/OOK mode.
	set_mode_sleep();
	set_mode_sleep();

	// Ideal bit rate is 16384 bps; this works out to 16385 bps.
	write_register(REG_BITRATE_MSB, 0x07);
	write_register(REG_BITRATE_LSB, 0xA1);

	// Use 64 samples for RSSI.
	write_register(REG_RSSI_CONFIG, 5);

	// 200 kHz channel bandwidth (mantissa = 20, exp = 1)
	write_register(REG_RX_BW, (1 << RX_BW_MANT_SHIFT) | 1);

	// Make sure enough preamble bytes are sent.
	write_register(REG_PREAMBLE_MSB, 0x00);
	write_register(REG_PREAMBLE_LSB, 0x18);

	// Use 4 bytes for Sync word.
	write_register(REG_SYNC_CONFIG, SYNC_ON | 3);

	// Sync word.
	write_register(REG_SYNC_VALUE_1, 0xFF);
	write_register(REG_SYNC_VALUE_2, 0x00);
	write_register(REG_SYNC_VALUE_3, 0xFF);
	write_register(REG_SYNC_VALUE_4, 0x00);

	// Use unlimited length packet format (data sheet section 4.2.13.2).
	write_register(REG_PACKET_CONFIG_1, PACKET_FORMAT_FIXED);
	write_register(REG_PAYLOAD_LENGTH, 0);
	write_register(REG_PACKET_CONFIG_2, PACKET_MODE| 0);
}

static inline bool fifo_empty(void) {
	return (read_register(REG_IRQ_FLAGS_2) & FIFO_EMPTY) != 0;
}

static inline bool fifo_full(void) {
	return (read_register(REG_IRQ_FLAGS_2) & FIFO_FULL) != 0;
}

static inline bool fifo_threshold_exceeded(void) {
	return (read_register(REG_IRQ_FLAGS_2) & FIFO_LEVEL) != 0;
}

static inline void clear_fifo(void) {
	write_register(REG_IRQ_FLAGS_2, FIFO_OVERRUN);
}

static inline uint8_t read_fifo_flags(void) {
	return read_register(REG_IRQ_FLAGS_2);
}

static inline void xmit_byte(uint8_t b) {
	write_register(REG_FIFO, b);
}

static inline void xmit(uint8_t* data, int len) {
	write_burst(REG_FIFO, data, len);
}

static bool wait_for_fifo_room(void) {
	for (int w = 0; w < MAX_WAIT; w++) {
		if (!fifo_full()) {
			return true;
		}
		// Yield to other tasks to avoid starving the scheduler / watchdog
		vTaskDelay(1);
	}
	sequencer_stop();
	set_mode_sleep();
	ESP_LOGI(TAG, "FIFO still full; flags = %02X", read_fifo_flags());
	return false;
}

static void wait_for_transmit_done(void) {
	uint8_t mode;
	for (int w = 0; w < MAX_WAIT; w++) {
		mode = read_mode();
		if (mode == MODE_STDBY) {
			ESP_LOGD(TAG, "transmit done; waits = %d", w);
			return;
		}
		// Delay 1ms and yield to scheduler (prevents WDT timeout)
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	sequencer_stop();
	set_mode_sleep();
	ESP_LOGI(TAG, "transmit still not done; mode = %d", mode);
}

void transmit(uint8_t *buf, int count) {
	ESP_LOGD(TAG, "transmit %d-byte packet", count);
	clear_fifo();
	set_mode_standby();
	// Automatically enter Transmit state on FifoLevel interrupt.
	write_register(REG_FIFO_THRESH, TX_START_CONDITION | FIFO_THRESHOLD);
	write_register(REG_SEQ_CONFIG_1, SEQUENCER_START | IDLE_MODE_STANDBY | FROM_START_TO_TX);
	int avail = FIFO_SIZE;
	for (;;) {
		if (avail > count) {
			avail = count;
		}
		ESP_LOGD(TAG, "writing %d bytes to TX FIFO", avail);
		xmit(buf, avail);
		ESP_LOGD(TAG, "after xmit: mode = %d", read_mode());
		buf += avail;
		count -= avail;
		if (count == 0) {
			break;
		}
		// Wait until there is room for at least fifoSize - fifoThreshold bytes in the FIFO.
		// Err on the short side here to avoid TXFIFO underflow.
		vTaskDelay(pdMS_TO_TICKS(16));  // ~16ms wait for FIFO
		for (int w = 0; w < MAX_WAIT; w++) {
			if (!fifo_threshold_exceeded()) {
				avail = FIFO_SIZE - FIFO_THRESHOLD;
				break;
			}
			// Short delay with scheduler yield
			vTaskDelay(1);  // 1 tick (1ms @ 1kHz)
		}
		// If still no room, abort to avoid deadlock and WDT
		if (fifo_threshold_exceeded()) {
			ESP_LOGE(TAG, "Timeout waiting for FIFO threshold; aborting transmit");
			sequencer_stop();
			set_mode_sleep();
			return;
		}
	}
	if (!wait_for_fifo_room()) {
		return;
	}
	xmit_byte(0);
	wait_for_transmit_done();
	set_mode_standby();
	tx_packets++;
}

static bool packet_seen(void) {
	bool seen = (read_register(REG_IRQ_FLAGS_1) & SYNC_ADDRESS_MATCH) != 0;
	if (seen) {
		ESP_LOGD(TAG, "incoming packet seen");
	}
	return seen;
}

static inline uint8_t recv_byte(void) {
	return read_register(REG_FIFO);
}

static uint8_t last_rssi = 0xFF;

int read_rssi(void) {
	return -(int)last_rssi / 2;
}

typedef void wait_fn_t(int);

static int rx_common(wait_fn_t wait_fn, uint8_t *buf, int count, int timeout) {
	// Use unlimited length packet format (data sheet section 4.2.13.2).
	write_register(REG_PACKET_CONFIG_1, PACKET_FORMAT_FIXED);
	write_register(REG_PAYLOAD_LENGTH, 0);
	write_register(REG_PACKET_CONFIG_2, PACKET_MODE| 0);
#ifndef USE_POLLING
	gpio_intr_enable(LORA_DIO2);
#endif
	ESP_LOGD(TAG, "starting receive");
	set_mode_receive();
	if (!packet_seen()) {
		// Stay in RX mode.
		wait_fn(timeout);
		if (!packet_seen()) {
			set_mode_sleep();
			ESP_LOGD(TAG, "receive timeout");
			return 0;
		}
	}
	last_rssi = read_register(REG_RSSI);

	// FIFO read with absolute timeout (max 80ms for reading)
	// Medtronic packets are max ~70 bytes @ 16384 bps = ~35ms
	int64_t fifo_start = esp_timer_get_time();
	const int64_t FIFO_TIMEOUT_US = 80 * 1000;  // 80ms max

	int n = 0;
	int w = 0;
	while (n < count) {
		// Check absolute timeout first
		if ((esp_timer_get_time() - fifo_start) > FIFO_TIMEOUT_US) {
			ESP_LOGD(TAG, "FIFO read timeout after %d bytes", n);
			break;
		}
		if (fifo_empty()) {
			usleep(500);  // 500us - short wait for FIFO data
			w++;
			if (w >= MAX_WAIT) {
				ESP_LOGD(TAG, "max RX FIFO wait reached");
				break;
			}
			if ((w % 20) == 0) {
				taskYIELD();  // Yield every ~10ms to prevent WDT
			}
			continue;
		}
		uint8_t b = recv_byte();
		if (b == 0) {
			break;
		}
		buf[n++] = b;
		w = 0;
	}
	set_mode_sleep();
	clear_fifo();
#ifndef USE_POLLING
	gpio_intr_disable(LORA_DIO2);
#endif
	if (n > 0) {
		// Remove spurious final byte consisting of just one or two high bits.
		uint8_t b = buf[n-1];
		if (b == 0x80 || b == 0xC0) {
			ESP_LOGD(TAG, "end-of-packet glitch %X with RSSI %d", b >> 6, read_rssi());
			n--;
		}
	}
	if (n > 0) {
		rx_packets++;
	}
	return n;
}

#ifdef USE_POLLING
// Polling mode: опрашиваем регистр SyncMatch каждые 500us
#define POLL_INTERVAL	500  // microseconds

static void wait_for_packet(int timeout) {
	int timeout_us = timeout * MILLISECOND;
	while (!packet_seen() && timeout_us > 0) {
		usleep(POLL_INTERVAL);
		timeout_us -= POLL_INTERVAL;
		// Yield every 10ms to prevent WDT
		if ((timeout_us % (10 * MILLISECOND)) == 0) {
			taskYIELD();
		}
	}
}

int sleep_receive(uint8_t *buf, int count, int timeout) {
	// В polling режиме sleep_receive работает как обычный receive
	return rx_common(wait_for_packet, buf, count, timeout);
}

#else
// Interrupt mode: ждём notification от ISR или light sleep с таймером

static void wait_for_packet(int timeout) {
	ESP_LOGD(TAG, "waiting for interrupt");
	// Use atomic_store with release semantics to ensure proper synchronization
	// with the ISR that uses atomic_load with acquire semantics
	atomic_store_explicit(&rx_waiting_task, xTaskGetCurrentTaskHandle(), memory_order_release);
	xTaskNotifyWait(0, 0, 0, pdMS_TO_TICKS(timeout));
	atomic_store_explicit(&rx_waiting_task, NULL, memory_order_release);
	ESP_LOGD(TAG, "finished waiting");
}

static void sleep_until_interrupt(int timeout) {
	// Wait for UART TX to complete before entering light sleep
	uart_wait_tx_idle_polling(UART_NUM_0);
	uint64_t us = (uint64_t)timeout * MILLISECOND;
	esp_sleep_enable_timer_wakeup(us);
	esp_light_sleep_start();
	esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);
}

int sleep_receive(uint8_t *buf, int count, int timeout) {
	return rx_common(sleep_until_interrupt, buf, count, timeout);
}

#endif

int receive(uint8_t *buf, int count, int timeout) {
	return rx_common(wait_for_packet, buf, count, timeout);
}

uint32_t read_frequency(void) {
	uint8_t frf[3];
	read_burst(REG_FRF_MSB, frf, sizeof(frf));
	uint32_t f = (frf[0] << 16) | (frf[1] << 8) | frf[2];
	return ((uint64_t)f * FXOSC) >> 19;
}

void set_frequency(uint32_t freq_hz) {
	uint32_t f = (((uint64_t)freq_hz << 19) + FXOSC/2) / FXOSC;
	uint8_t frf[3];
	frf[0] = f >> 16;
	frf[1] = f >> 8;
	frf[2] = f;
	write_burst(REG_FRF_MSB, frf, sizeof(frf));
}

int read_version(void) {
	return read_register(REG_VERSION);
}

int version_major(int v) {
	return v >> 4;
}

int version_minor(int v) {
	return v & 0xF;
}
