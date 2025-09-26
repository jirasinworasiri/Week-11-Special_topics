// การทดลองท 1: อานคา Potentiometer
// ตวอยางพนฐาน ADC ของ ESP32 โดยใช ESP-IDF

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// กำหนด pin ทใช
#define POTENTIOMETER_CHANNEL ADC1_CHANNEL_6  // GPIO34 (ADC1_CH6)
#define DEFAULT_VREF    1100        // ใช adc2_vref_to_gpio() เพอใหไดคาประมาณทดกวา
#define NO_OF_SAMPLES   64          // การสมสญญาณหลายครง

static const char *TAG = "ADC_POT";
static esp_adc_cal_characteristics_t *adc_chars;

static bool check_efuse(void)
{
    // ตรวจสอบวา TP ถกเขยนลงใน eFuse หรอไม
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: รองรบ");
    } else {
        ESP_LOGI(TAG, "eFuse Two Point: ไมรองรบ");
    }
    // ตรวจสอบวา Vref ถกเขยนลงใน eFuse หรอไม
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Vref: รองรบ");
    } else {
        ESP_LOGI(TAG, "eFuse Vref: ไมรองรบ");
    }
    return true;
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ใชการปรบเทยบแบบ Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ใชการปรบเทยบแบบ eFuse Vref");
    } else {
        ESP_LOGI(TAG, "ใชการปรบเทยบแบบ Default Vref");
    }
}

void app_main(void)
{
    // ตรวจสอบวา Two Point หรอ Vref ถกเขยนลงใน eFuse หรอไม
    check_efuse();

    // กำหนดคา ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(POTENTIOMETER_CHANNEL, ADC_ATTEN_DB_11);

    // ปรบเทยบ ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    ESP_LOGI(TAG, "ทดสอบ ADC Potentiometer ของ ESP32");
    ESP_LOGI(TAG, "Pin: GPIO34 (ADC1_CH6)");
    ESP_LOGI(TAG, "ชวง: 0-3.3V");
    ESP_LOGI(TAG, "ความละเอยด: 12-bit (0-4095)");
    ESP_LOGI(TAG, "Attenuation: 11dB");
    ESP_LOGI(TAG, "-------------------------");

    // อานคา ADC1 อยางตอเนอง
    while (1) {
        uint32_t adc_reading = 0;

        // การสมสญญาณหลายครง
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)POTENTIOMETER_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;

        // แปลง adc_reading เปนแรงดนในหนวย mV
        uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float voltage = voltage_mv / 1000.0f;

        // แปลงเปนเปอรเซนต
        float percentage = (adc_reading / 4095.0f) * 100.0f;

        // แสดงผลลพธ
        ESP_LOGI(TAG, "คา ADC: %d | แรงดน: %.2fV | เปอรเซนต: %.1f%%", 
                 adc_reading, voltage, percentage);

        vTaskDelay(pdMS_TO_TICKS(500));  // หนวงเวลา 500ms
    }
}