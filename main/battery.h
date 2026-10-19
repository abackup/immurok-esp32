#pragma once

#include <stdint.h>
#include "esp_err.h"

typedef struct
{
    uint16_t voltage_mv; // 电池电压，单位 mV
    uint16_t adc_raw;    // ADC 原始读数
    uint8_t percent;     // 估算剩余电量，0～100
} battery_reading_t;

// 初始化电池 ADC。重复调用不会重复初始化。
esp_err_t battery_init(void);

// 读取当前电池电量、电压和 ADC 原始值。
esp_err_t battery_get_reading(battery_reading_t *reading);
