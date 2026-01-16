#include <stdint.h>
#include <stdbool.h>

#include "version.h"

#define TAG		"PICKLE_MY"
#define LOG_LOCAL_LEVEL	ESP_LOG_DEBUG
#include <esp_log.h>

#define MILLISECONDS	1000
#define SECONDS		1000000

#define MHz		1000000

#define STATE_OK "OK"

void gnarl_init(void);
void start_gnarl_task(void);
uint32_t gnarl_load_frequency(void);
void rfspy_command(const uint8_t *buf, int count, int rssi);
void send_code(const uint8_t code);
void send_bytes(const uint8_t *buf, int count);
void print_bytes(const char* msg, const uint8_t *buf, int count);
bool ble_is_connected(void);
bool ble_is_ready(void);  // Returns true after NimBLE fully initialized
