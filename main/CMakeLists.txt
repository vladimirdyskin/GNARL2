idf_component_register(
    SRCS "main.c"
         "gnarl.c"
         "ble.c"
         "adc.c"
    INCLUDE_DIRS "../include"
                 "."
    REQUIRES radio medtronic esp_timer nvs_flash bt driver esp_adc led_strip esp_driver_uart
)
