# ESP32 + RFM95 GNARL Прошивка

RileyLink-совместимая прошивка для ESP32 Heltec WiFi LoRa 32 V2 с радиомодулем RFM95.
Реализует протокол RFSpy/GNARL для связи с инсулиновыми помпами Medtronic через iAPS/Loop/AndroidAPS.

## Благодарности

Проект использует код из:
- [GNARL](https://github.com/ecc1/gnarl/) - оригинальная реализация GNARL
- [Pickle](https://github.com/d3xr/pickle/) - порт на ESP32

## Поддерживаемое оборудование

### ESP32 Heltec WiFi LoRa 32 V2 (по умолчанию)

- **MCU:** ESP32 Xtensa LX6 dual-core @ 80 MHz
- **Радио:** RFM95 (SX1276) в режиме OOK @ 868.25 МГц
- **BLE:** NimBLE стек

| Функция | GPIO |
|---------|------|
| LED | 25 |
| Кнопка | 0 |
| LORA_SCK | 5 |
| LORA_MOSI | 27 |
| LORA_MISO | 19 |
| LORA_CS | 18 |
| LORA_RST | 14 |
| LORA_DIO0 | 26 |
| LORA_DIO2 | 32 |
| BATTERY_ADC | 37 |

### ESP32-C6 Super Mini + RFM95 (альтернатива)

- **MCU:** ESP32-C6 RISC-V single-core @ 160 MHz
- **Радио:** RFM95 (SX1276) в режиме OOK @ 868.25 МГц
- **LED:** WS2812 RGB

| Функция | GPIO |
|---------|------|
| LED (WS2812) | 8 |
| Кнопка | 9 |
| LORA_SCK | 6 |
| LORA_MOSI | 3 |
| LORA_MISO | 2 |
| LORA_CS | 5 |
| LORA_RST | 7 |
| LORA_DIO2 | 14 |
| BATTERY_ADC | 0 |

Для сборки под ESP32-C6:
```bash
idf.py set-target esp32c6
idf.py build
```

## Требования

- **ESP-IDF:** v5.2+ (проверено на v5.4.1)
- **Python:** 3.8+
- **CMake:** 3.16+

## Установка

### 1. Установка ESP-IDF

```bash
# Клонировать ESP-IDF
git clone -b v5.4.1 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf

# Установить инструменты
cd ~/esp/esp-idf
./install.sh esp32

# Добавить в shell (добавить в ~/.bashrc или ~/.zshrc)
alias get_idf='. ~/esp/esp-idf/export.sh'
```

### 2. Настройка параметров помпы

Создать `include/pump_config.h`:

```c
#ifndef _PUMP_CONFIG_H
#define _PUMP_CONFIG_H

#define PUMP_ID         "123456"      // Серийный номер помпы
#define PUMP_FREQUENCY  868250000     // Рабочая частота (Гц)
#define MMTUNE_START    868150000     // Начальная частота mmtune

#endif
```

**Никогда не коммитьте pump_config.h в git!**

### 3. Сборка и прошивка

```bash
# Активировать окружение ESP-IDF
get_idf

# Перейти в проект
cd esp32rfm95

# Сборка
idf.py build

# Прошивка (замените порт на ваш)
idf.py -p /dev/cu.usbserial-0001 flash

# Мониторинг serial
idf.py -p /dev/cu.usbserial-0001 monitor

# Сборка + Прошивка + Мониторинг
idf.py -p /dev/cu.usbserial-0001 flash monitor
```

### 4. Чистая сборка

При изменении `sdkconfig.defaults`:

```bash
idf.py fullclean
idf.py build
```

## Конфигурация

### Опции (`include/module.h`)

```c
// Опции отладки (раскомментировать для включения)
// #define BLE_DEBUG      // Дамп BLE пакетов
// #define RADIO_DEBUG    // Дамп TX/RX пакетов

// Имя BLE устройства (макс. 8 символов)
#define BLE_NAME        "MYGN"

// Light sleep при отключённом BLE
#define BLE_SLEEP
```

### Энергосбережение (`sdkconfig.defaults`)

- Tickless idle для автоматического light sleep
- Параметры BLE оптимизированы для экономии энергии
- Уровень логирования — только ERROR

## Использование

1. Прошить устройство
2. Открыть iAPS/Loop/AndroidAPS
3. Добавить новое устройство RileyLink (отображается как "RileyLink")
4. Устройство автоматически подключится и синхронизируется с помпой

## Решение проблем

### Устройство не найдено в приложении
- Проверьте включён ли BLE на телефоне
- Перезагрузите устройство
- Проверьте serial монитор на ошибки

### Помпа не отвечает
- Проверьте `PUMP_ID` в pump_config.h
- Проверьте частоту (обычно 868250000 Гц для EU)
- Убедитесь что помпа в зоне досягаемости (~1-2 метра)

### Watchdog timeout
- Проверьте `CONFIG_ESP_TASK_WDT_TIMEOUT_S=30` в sdkconfig.defaults
- Выполните `idf.py fullclean && idf.py build`

## Файлы

| Файл | Описание |
|------|----------|
| `main/main.c` | Точка входа, инициализация |
| `main/gnarl.c` | Обработчик протокола RFSpy |
| `main/ble.c` | NimBLE GATT сервисы |
| `main/adc.c` | Мониторинг батареи |
| `components/radio/rfm95.c` | Драйвер RFM95 |
| `components/medtronic/` | Протокол Medtronic |
| `include/module.h` | GPIO пины, опции отладки |
| `include/pump_config.h` | Настройки помпы (приватный) |

## Лицензия

Основано на проекте GNARL. См. файл LICENSE.
