#ifndef _MODULE_H
#define _MODULE_H

// Debug options (uncomment to enable)
// #define BLE_DEBUG      // BLE packet hex dump (BLE > / BLE <)
// #define RADIO_DEBUG    // Radio TX/RX packet hex dump

// BLE device name (as seen by iAPS/Loop)
// Max 8 characters for reliable BLE advertising
#define BLE_NAME        "MYGN"

// Light sleep when BLE disconnected (~0.8mA vs ~20-40mA active)
// Uncomment to enable light sleep mode
// #define BLE_SLEEP

#ifdef CONFIG_IDF_TARGET_ESP32C6
    //=========================================================================
    // ESP32-C6 Super Mini + RFM95
    //=========================================================================
    
    // WS2812 RGB LED on GPIO8 (directly controllable, active high)
    #define LED             GPIO_NUM_8
    #define LED_IS_WS2812   1           // Use WS2812 driver instead of GPIO
    
    // Boot button (directly usable)
    #define BUTTON          GPIO_NUM_9
    
    // No OLED on Super Mini
    // #define OLED_ENABLE
    
    // SPI for RFM95
    #define LORA_SCK        GPIO_NUM_6
    #define LORA_MOSI       GPIO_NUM_3
    #define LORA_MISO       GPIO_NUM_2
    #define LORA_CS         GPIO_NUM_5
    #define LORA_RST        GPIO_NUM_7
    
    // RFM95 DIO pins (directly usable for interrupt)
    // DIO0/DIO1 not connected — using DIO2 for SyncAddress
    #define LORA_DIO2       GPIO_NUM_14
    
    // Battery sensing enabled on Super Mini with external divider on GPIO0
    // Divider: 1MΩ (R1) + 1MΩ (R2) = 1:1 ratio -> VBAT = ADC * 2.0
    // 1S LiPo: 2.5-4.2V -> ADC sees 1.25-2.1V
    #define BATTERY_ADC        GPIO_NUM_0
    // VBAT voltage divider (kΩ): 1MΩ + 1MΩ
    #define VDIV_R1_KOHM    1000
    #define VDIV_R2_KOHM    1000

#else
    //=========================================================================
    // ESP32 Heltec WiFi LoRa 32 V2
    //=========================================================================
    
    #define BUTTON          GPIO_NUM_0
    #define LED             GPIO_NUM_25
    // #define LED_IS_WS2812            // Standard GPIO LED
    
    // OLED I2C (optional)
    #define OLED_SDA        GPIO_NUM_4
    #define OLED_SCL        GPIO_NUM_15
    #define OLED_RST        GPIO_NUM_16
    // #define OLED_ENABLE              // Uncomment to enable OLED
    
    // SPI for RFM95
    #define LORA_SCK        GPIO_NUM_5
    #define LORA_MOSI       GPIO_NUM_27
    #define LORA_MISO       GPIO_NUM_19
    #define LORA_CS         GPIO_NUM_18
    #define LORA_RST        GPIO_NUM_14
    
    // RFM95 DIO pins
    #define LORA_DIO0       GPIO_NUM_26
    #define LORA_DIO1       GPIO_NUM_33
    #define LORA_DIO2       GPIO_NUM_32
    
    // Battery sense ADC pin
    #define BATTERY_ADC             GPIO_NUM_37

    // VBAT sense enable (active LOW)
    #define BATTERY_SENSE_EN        GPIO_NUM_21
    #define BATTERY_SENSE_EN_ACTIVE 0

    // VBAT voltage divider (kΩ): ~220k/100k => VBAT ≈ ADC × 3.2
    #define VDIV_R1_KOHM    220
    #define VDIV_R2_KOHM    100

#endif

#endif // _MODULE_H