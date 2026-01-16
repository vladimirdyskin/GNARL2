# ESP32 + RFM95 GNARL Firmware

RileyLink-compatible firmware for ESP32 Heltec WiFi LoRa 32 V2 with RFM95 radio module.
Implements RFSpy/GNARL protocol for communication with Medtronic insulin pumps via iAPS/Loop/AndroidAPS.

## Credits

This project uses code from:
- [GNARL](https://github.com/ecc1/gnarl/) - original GNARL implementation
- [Pickle](https://github.com/d3xr/pickle/) - ESP32 port

## Supported Hardware

### ESP32 Heltec WiFi LoRa 32 V2 (default)

- **MCU:** ESP32 Xtensa LX6 dual-core @ 80 MHz
- **Radio:** RFM95 (SX1276) in OOK mode @ 868.25 MHz
- **BLE:** NimBLE stack

| Function | GPIO |
|----------|------|
| LED | 25 |
| Button | 0 |
| LORA_SCK | 5 |
| LORA_MOSI | 27 |
| LORA_MISO | 19 |
| LORA_CS | 18 |
| LORA_RST | 14 |
| LORA_DIO0 | 26 |
| LORA_DIO2 | 32 |
| BATTERY_ADC | 37 |

### ESP32-C6 Super Mini + RFM95 (alternative)

- **MCU:** ESP32-C6 RISC-V single-core @ 160 MHz
- **Radio:** RFM95 (SX1276) in OOK mode @ 868.25 MHz
- **LED:** WS2812 RGB

| Function | GPIO |
|----------|------|
| LED (WS2812) | 8 |
| Button | 9 |
| LORA_SCK | 6 |
| LORA_MOSI | 3 |
| LORA_MISO | 2 |
| LORA_CS | 5 |
| LORA_RST | 7 |
| LORA_DIO2 | 14 |
| BATTERY_ADC | 0 |

To build for ESP32-C6:
```bash
idf.py set-target esp32c6
idf.py build
```

## Requirements

- **ESP-IDF:** v5.2+ (tested with v5.4.1)
- **Python:** 3.8+
- **CMake:** 3.16+

## Setup

### 1. Install ESP-IDF

```bash
# Clone ESP-IDF
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# Install tools
cd ~/esp/esp-idf
./install.sh esp32

# Add to shell (add to ~/.bashrc or ~/.zshrc)
alias get_idf='. ~/esp/esp-idf/export.sh'
```

### 2. Configure Pump Settings

Create `include/pump_config.h`:

```c
#ifndef _PUMP_CONFIG_H
#define _PUMP_CONFIG_H

#define PUMP_ID         "123456"      // Your pump serial number
#define PUMP_FREQUENCY  868250000     // Working frequency (Hz)
#define MMTUNE_START    868150000     // mmtune scan start frequency

#endif
```

**Never commit pump_config.h to git!**

### 3. Build & Flash

```bash
# Activate ESP-IDF environment
get_idf

# Navigate to project
cd esp32rfm95

# Build
idf.py build

# Flash (replace port with your device)
idf.py -p /dev/cu.usbserial-0001 flash

# Monitor serial output
idf.py -p /dev/cu.usbserial-0001 monitor

# Build + Flash + Monitor
idf.py -p /dev/cu.usbserial-0001 flash monitor
```

### 4. Clean Build

If you change `sdkconfig.defaults`, run:

```bash
idf.py fullclean
idf.py build
```

## Configuration

### Options (`include/module.h`)

```c
// Debug options (uncomment to enable)
// #define BLE_DEBUG      // BLE packet hex dump
// #define RADIO_DEBUG    // Radio TX/RX packet hex dump

// BLE device name (max 8 characters)
#define BLE_NAME        "MYGN"

// Light sleep when BLE disconnected
#define BLE_SLEEP
```

### Power Management (`sdkconfig.defaults`)

- Tickless idle enabled for automatic light sleep
- BLE connection parameters optimized for power saving
- Logging level set to ERROR only

## Usage

1. Flash firmware to device
2. Open iAPS/Loop/AndroidAPS
3. Add new RileyLink device (appears as "RileyLink")
4. Device will auto-connect and sync with pump

## Troubleshooting

### Device not found in app
- Check BLE is enabled on phone
- Restart the device
- Check serial monitor for errors

### Pump not responding
- Verify `PUMP_ID` in pump_config.h
- Check frequency (usually 868250000 Hz for EU)
- Ensure pump is in range (~1-2 meters)

### Watchdog timeout
- Check `CONFIG_ESP_TASK_WDT_TIMEOUT_S=30` in sdkconfig.defaults
- Run `idf.py fullclean && idf.py build`

## Files

| File | Description |
|------|-------------|
| `main/main.c` | Entry point, hardware init |
| `main/gnarl.c` | RFSpy protocol handler |
| `main/ble.c` | NimBLE GATT services |
| `main/adc.c` | Battery monitoring |
| `components/radio/rfm95.c` | RFM95 driver |
| `components/medtronic/` | Medtronic protocol |
| `include/module.h` | GPIO pins, debug options |
| `include/pump_config.h` | Pump settings (private) |

## License

Based on GNARL project. See LICENSE file.
