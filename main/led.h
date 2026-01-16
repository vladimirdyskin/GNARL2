#ifndef _LED_H
#define _LED_H

#include <stdbool.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include "module.h"

// LED modes (from RileyLink protocol)
#define LED_MODE_OFF  0x00
#define LED_MODE_ON   0x01
#define LED_MODE_AUTO 0x02

// LED event types for task queue
typedef enum {
	LED_EVENT_RADIO_TX,
	LED_EVENT_RADIO_RX,
	LED_EVENT_BLE,
} led_event_t;

// Current LED mode
extern uint8_t current_led_mode;

// Initialize LED GPIO and start LED task
void led_init(void);

// Low-level LED control
#ifdef LED_IS_WS2812
// WS2812 RGB LED (ESP32-C6 Super Mini) - implemented in gnarl.c
void led_on(void);
void led_off(void);
#else
// Standard GPIO LED (ESP32 Heltec)
static inline void led_on(void) {
	gpio_set_level(LED, 1);
}

static inline void led_off(void) {
	gpio_set_level(LED, 0);
}
#endif

// Load saved LED mode from NVS
void led_load_mode(void);

// Set LED mode (called from CmdLED) - saves to NVS
void led_set_mode(uint8_t mode);

// Activity indicators (send event to LED task - non-blocking)
void led_indicate_radio_tx(void);
void led_indicate_radio_rx(void);
void led_indicate_ble_activity(void);

#endif // _LED_H
