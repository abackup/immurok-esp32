
#include "battery.h"

#include <stdbool.h>
#include <stddef.h>

#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "battery";

// 标准 XIAO ESP32-S3：D0 = GPIO1 = ADC1_CHANNEL_0
#define BATTERY_ADC_UNIT ADC_UNIT_1
#define BATTERY_ADC_CHANNEL ADC_CHANNEL_3
#define BATTERY_ADC_ATTEN ADC_ATTEN_DB_12

// 分压电阻：电池正极 -- R_TOP -- ADC -- R_BOTTOM -- GND
#define BATTERY_R_TOP_OHMS 100000
#define BATTERY_R_BOTTOM_OHMS 100000

// 实际电池电压 = ADC 引脚电压 × 此比例
#define BATTERY_DIVIDER_RATIO                              \
    ((float)(BATTERY_R_TOP_OHMS + BATTERY_R_BOTTOM_OHMS) / \
     (float)BATTERY_R_BOTTOM_OHMS)

#define BATTERY_SAMPLE_COUNT 16

static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali_handle;
static bool s_initialized;

// 常见单节锂电池的近似静置电压曲线。
// 实际百分比会受到电芯型号、负载、电池温度和老化影响。
typedef struct
{
    uint16_t mv;
    uint8_t percent;
} battery_point_t;

static const battery_point_t s_curve[] = {
    {3000, 0},
    {3450, 5},
    {3680, 10},
    {3740, 20},
    {3770, 30},
    {3790, 40},
    {3820, 50},
    {3870, 60},
    {3920, 70},
    {3980, 80},
    {4050, 90},
    {4150, 95},
    {4200, 100},
};

static uint8_t battery_voltage_to_percent(uint16_t mv)
{
    const size_t count = sizeof(s_curve) / sizeof(s_curve[0]);

    if (mv <= s_curve[0].mv)
    {
        return 0;
    }

    if (mv >= s_curve[count - 1].mv)
    {
        return 100;
    }

    for (size_t i = 1; i < count; ++i)
    {
        if (mv <= s_curve[i].mv)
        {
            const uint16_t x0 = s_curve[i - 1].mv;
            const uint16_t x1 = s_curve[i].mv;
            const uint8_t y0 = s_curve[i - 1].percent;
            const uint8_t y1 = s_curve[i].percent;

            // 分段线性插值
            const uint32_t numerator =
                (uint32_t)(mv - x0) * (uint32_t)(y1 - y0);

            return (uint8_t)(y0 + numerator / (x1 - x0));
        }
    }

    return 100;
}

esp_err_t battery_init(void)
{
    if (s_initialized)
    {
        return ESP_OK;
    }

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_RETURN_ON_ERROR(
        adc_oneshot_new_unit(&unit_cfg, &s_adc_handle),
        TAG, "ADC unit init failed");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    esp_err_t err = adc_oneshot_config_channel(
        s_adc_handle, BATTERY_ADC_CHANNEL, &chan_cfg);

    if (err != ESP_OK)
    {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return err;
    }

    // ESP32-S3 使用曲线拟合校准方案。
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = BATTERY_ADC_UNIT,
        .chan = BATTERY_ADC_CHANNEL,
        .atten = BATTERY_ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    err = adc_cali_create_scheme_curve_fitting(
        &cali_cfg, &s_cali_handle);

    if (err != ESP_OK)
    {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = NULL;
        return err;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Battery ADC initialized on GPIO4");

    return ESP_OK;
}

esp_err_t battery_get_reading(battery_reading_t *reading)
{
    if (reading == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(battery_init(), TAG, "Battery init failed");

    int raw_sum = 0;

    for (int i = 0; i < BATTERY_SAMPLE_COUNT; ++i)
    {
        int raw = 0;

        ESP_RETURN_ON_ERROR(
            adc_oneshot_read(s_adc_handle, BATTERY_ADC_CHANNEL, &raw),
            TAG, "ADC read failed");

        raw_sum += raw;
    }

    const int raw_avg = raw_sum / BATTERY_SAMPLE_COUNT;

    int pin_mv = 0;

    ESP_RETURN_ON_ERROR(
        adc_cali_raw_to_voltage(s_cali_handle, raw_avg, &pin_mv),
        TAG, "ADC calibration failed");

    // 先恢复分压前的电池电压，再换算为 mV。
    int battery_mv = (int)((float)pin_mv * BATTERY_DIVIDER_RATIO + 0.5f);

    if (battery_mv < 0)
    {
        battery_mv = 0;
    }

    if (battery_mv > UINT16_MAX)
    {
        battery_mv = UINT16_MAX;
    }

    reading->voltage_mv = (uint16_t)battery_mv;
    reading->adc_raw = (uint16_t)raw_avg;
    reading->percent = battery_voltage_to_percent(
        reading->voltage_mv);

    ESP_LOGI(TAG, "ADC raw=%u, battery=%u mV, level=%u%%",
             reading->adc_raw,
             reading->voltage_mv,
             reading->percent);

    return ESP_OK;
}
