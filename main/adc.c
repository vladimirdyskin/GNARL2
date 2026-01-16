#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdbool.h>
#include <stdatomic.h>

#include "module.h"

#include "adc.h"

const static char *TAG = "ADC";

// Atomic for thread-safe access from BLE callbacks and ADC task
// Ensures correct behavior on both Xtensa (ESP32) and RISC-V (ESP32-C6)
static _Atomic int battery_voltage;

// Battery charge thresholds for percentage calculation (used by battery_percent())
// Full charge: 4.2V for Li-Ion/Li-Po
// Empty threshold: 3.5V (conservative safe minimum to protect battery)
#define BATTERY_FULL_MV  (4200)
#define BATTERY_EMPTY_MV (3500)

#ifdef BATTERY_ADC
// Battery monitoring enabled

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// Keep ADC sampling lightweight to avoid impacting radio/BLE.
#define AVG_SAMPLES (4)
#define AVG_PERIOD (1) // seconds total time spent sampling per update
// With a high-impedance VBAT divider (e.g. 100k/100k), the ADC sampling cap
// can hold charge and readings may lag. Do a few dummy reads to settle.
#define DUMMY_READS (2)

// How often to refresh the cached battery voltage.
#define UPDATE_PERIOD_MS (60000)  // 60 seconds - avoid interfering with radio/BLE
// Scale the ADC input voltage back to the actual battery voltage using the
// board's resistor divider (see include/module.h).
#define SCALE_VOLTAGE(raw_mv) ((int)((int64_t)(raw_mv) * (VDIV_R1_KOHM + VDIV_R2_KOHM) / VDIV_R2_KOHM))

// LiPo/Li-ion sanity bounds. If the computed value is outside these bounds,
// it's very likely the divider/pin definition doesn't match the actual board.
#define LIPO_MIN_MV (2500)
#define LIPO_MAX_MV (4500)

// Critical battery level - below this, enter deep sleep to prevent
// infinite reboot loops that would drain battery completely
#define BATTERY_CRITICAL_MV (3300)

// Raw ADC saturation indicator (12-bit).
#define ADC_RAW_SAT (4090)

static bool warned_divider_mismatch;
static bool warned_adc_saturation;

static int choose_battery_mv(int adc_mv, int scaled_mv)
{
    // Prefer scaled value when it looks like a plausible battery voltage.
    if (scaled_mv >= LIPO_MIN_MV && scaled_mv <= LIPO_MAX_MV)
    {
        return scaled_mv;
    }

    // If scaling produces an impossible value (e.g., >5V on a 1S LiPo), fall
    // back to unscaled and warn once.
    if (!warned_divider_mismatch)
    {
        warned_divider_mismatch = true;
        ESP_LOGW(TAG,
                 "battery scaling looks wrong: adc=%dmV scaled=%dmV (VDIV_R1_KOHM=%d VDIV_R2_KOHM=%d). "
                 "Falling back to unscaled. Check BATTERY_ADC/VDIV in include/module.h.",
                 adc_mv,
                 scaled_mv,
                 (int)VDIV_R1_KOHM,
                 (int)VDIV_R2_KOHM);
    }
    return adc_mv;
}


#endif // BATTERY_ADC

uint16_t get_battery_voltage(void)
{
    return atomic_load(&battery_voltage);
}

uint8_t battery_percent(uint16_t batt_mv)
{
    if (batt_mv > BATTERY_FULL_MV)
        batt_mv = BATTERY_FULL_MV;
    else if (batt_mv < BATTERY_EMPTY_MV)
        batt_mv = BATTERY_EMPTY_MV;

    uint8_t percent = (batt_mv - BATTERY_EMPTY_MV) * 100 / (BATTERY_FULL_MV - BATTERY_EMPTY_MV);

    ESP_LOGD(TAG, "(Conv) Battery voltage: %d mV, percentage: %d%%", batt_mv, percent);

    return percent;
}

#ifdef BATTERY_ADC
// Full ADC task for battery monitoring (ESP32 Heltec)

static void adc_task(void *arg)
{
    adc_unit_t unit;
    adc_channel_t channel;
    esp_err_t err = adc_oneshot_io_to_channel(BATTERY_ADC, &unit, &channel);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_io_to_channel(%d) failed: %s", (int)BATTERY_ADC, esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_unit_handle_t adc_handle;
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = unit,
    };
    err = adc_oneshot_new_unit(&init_config, &adc_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_new_unit failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    adc_oneshot_chan_cfg_t config = {
		// ADC_ATTEN_DB_12: 0-3100mV range (best for 1S LiPo through 1:1 divider)
		// 1S LiPo 2.5-4.2V -> divider 1:1 -> ADC sees 1.25-2.1V (well within range)
		.atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    err = adc_oneshot_config_channel(adc_handle, channel, &config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "adc_oneshot_config_channel failed: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }

    adc_cali_handle_t cali_handle = NULL;

    // ADC calibration scheme selection:
    // - ESP32: Line Fitting (ADC_CALI_SCHEME_VER_LINE_FITTING)
    // - ESP32-S2/S3/C3/C6/H2: Curve Fitting (ADC_CALI_SCHEME_VER_CURVE_FITTING)
    // Reference: ESP-IDF ADC Calibration documentation
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = config.atten,
        .bitwidth = config.bitwidth,
    };
    err = adc_cali_create_scheme_curve_fitting(&cali_config, &cali_handle);
    ESP_LOGI(TAG, "ADC calibration scheme: Curve Fitting");
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .atten = config.atten,
        .bitwidth = config.bitwidth,
    };
    err = adc_cali_create_scheme_line_fitting(&cali_config, &cali_handle);
    ESP_LOGI(TAG, "ADC calibration scheme: Line Fitting");
#else
    ESP_LOGW(TAG, "No ADC calibration scheme supported on this chip");
    err = ESP_ERR_NOT_SUPPORTED;
#endif
    if (err != ESP_OK)
    {
        // Calibration is optional; don't reboot if unavailable.
        ESP_LOGW(TAG, "ADC calibration not available: %s", esp_err_to_name(err));
        cali_handle = NULL;
    }

#ifdef BATTERY_SENSE_EN
    // Heltec VBAT sense: GPIO21 must be asserted (LOW) to connect the divider
    // and power VEXT. Some OLED drivers may alter this pin, so we also
    // re-assert it during sampling below.
    gpio_set_direction(BATTERY_SENSE_EN, GPIO_MODE_OUTPUT);
    gpio_set_level(BATTERY_SENSE_EN, BATTERY_SENSE_EN_ACTIVE);
#endif

    // One-time probe (DEBUG only): helps confirm the correct VBAT ADC GPIO on this board.
    // Note: we only probe ADC1 GPIOs here.

    TickType_t last_wake_time = xTaskGetTickCount();

    for (;;)
    {
#ifdef BATTERY_SENSE_EN
        // Ensure VBAT divider is connected for this update window.
        gpio_set_level(BATTERY_SENSE_EN, BATTERY_SENSE_EN_ACTIVE);
#endif

        int averaged_result = 0;
        int valid_samples = 0;
        for (int i = 0; i < AVG_SAMPLES; i++)
        {
            int raw_output, voltage;

			// Dummy reads to help settle when source impedance is high.
			for (int d = 0; d < DUMMY_READS; d++)
			{
				(void)adc_oneshot_read(adc_handle, channel, &raw_output);
			}

            err = adc_oneshot_read(adc_handle, channel, &raw_output);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "adc_oneshot_read failed: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
                continue;
            }

            if (cali_handle)
            {
                err = adc_cali_raw_to_voltage(cali_handle, raw_output, &voltage);
                if (err != ESP_OK)
                {
                    ESP_LOGW(TAG, "adc_cali_raw_to_voltage failed: %s", esp_err_to_name(err));
                    vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
                    continue;
                }
                if (raw_output >= ADC_RAW_SAT && !warned_adc_saturation)
                {
                    warned_adc_saturation = true;
                    ESP_LOGW(TAG,
                             "ADC is saturated (raw=%d). This usually means wrong BATTERY_ADC pin or missing/incorrect divider.",
                             raw_output);
                }
                ESP_LOGD(TAG, "ADC raw: %d, voltage: %d mV", raw_output, voltage);
            }
            else
            {
                ESP_LOGD(TAG, "ADC raw: %d (no calibration)", raw_output);
                vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
                continue;
            }

            if (atomic_load(&battery_voltage) <= 1000)
            {
                const int scaled = SCALE_VOLTAGE(voltage);
                atomic_store(&battery_voltage, choose_battery_mv(voltage, scaled));
            }

            averaged_result += voltage;
            valid_samples++;

            vTaskDelay(pdMS_TO_TICKS(AVG_PERIOD * 1000 / AVG_SAMPLES));
        }
        if (valid_samples == 0)
        {
            ESP_LOGW(TAG, "ADC: no valid samples");
            vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UPDATE_PERIOD_MS));
            continue;
        }

        averaged_result /= valid_samples;

        const int scaled_avg = SCALE_VOLTAGE(averaged_result);
        const int chosen = choose_battery_mv(averaged_result, scaled_avg);
        ESP_LOGI(TAG, "VBAT: %d mV (%d%%)", chosen, battery_percent(chosen));
        atomic_store(&battery_voltage, chosen);

        // Check for critical battery level to prevent infinite reboot loops
        if (chosen < BATTERY_CRITICAL_MV) {
            ESP_LOGE(TAG, "CRITICAL: Battery voltage %d mV below safe threshold %d mV",
                     chosen, BATTERY_CRITICAL_MV);
            ESP_LOGE(TAG, "Entering deep sleep to prevent battery damage");
            vTaskDelay(pdMS_TO_TICKS(100)); // Allow log to flush
            esp_deep_sleep_start();
        }

		vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UPDATE_PERIOD_MS));
    }
}

void adc_init(void)
{
	ESP_LOGI(TAG, "adc_init: starting task, BATTERY_ADC=GPIO%d", (int)BATTERY_ADC);
	ESP_LOGI(TAG, "Battery config: divider %d/%d kΩ (ratio 1:%.1f), 1S LiPo",
			 (int)VDIV_R1_KOHM, (int)VDIV_R2_KOHM,
			 (float)(VDIV_R1_KOHM + VDIV_R2_KOHM) / VDIV_R2_KOHM);
	// Low priority: ADC is best-effort telemetry and must not interfere with radio/BLE.
    xTaskCreate(adc_task, "adc", 3072, NULL, 1, NULL);
}

#else

void adc_init(void)
{
    // No battery ADC - report 99% (4193mV ~ 99%)
    atomic_store(&battery_voltage, 4193);
    ESP_LOGI(TAG, "adc_init: no BATTERY_ADC, reporting 99%%");
}

#endif // BATTERY_ADC