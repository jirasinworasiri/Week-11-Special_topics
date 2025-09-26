// การทดลองที่ 3: ปรับปรุงความแม่นยำ ADC
// Enhanced ADC Reading with Oversampling and Filtering using ESP-IDF

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// กำหนด pin และพารามิเตอร์
#define SENSOR_CHANNEL ADC1_CHANNEL_6  // GPIO34 (ADC1_CH6)
#define DEFAULT_VREF    1100           // ใช้ adc2_vref_to_gpio() เพื่อให้ได้ค่าประมาณที่ดีกว่า
#define OVERSAMPLES     100            // จำนวนครั้งในการ oversample
#define FILTER_SIZE     10             // ขนาดของ Moving Average Filter

static const char *TAG = "ADC_ENHANCED";
static esp_adc_cal_characteristics_t *adc_chars;

// ตัวแปรสำหรับ Moving Average Filter
static float filterBuffer[FILTER_SIZE];
static int filterIndex = 0;
static float filterSum = 0.0;
static bool filterInitialized = false;

static bool check_efuse(void)
{
    // ตรวจสอบว่า TP ถูกเขียนลงใน eFuse หรือไม่
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Two Point: รองรับ");
    } else {
        ESP_LOGI(TAG, "eFuse Two Point: ไม่รองรับ");
    }
    // ตรวจสอบว่า Vref ถูกเขียนลงใน eFuse หรือไม่
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        ESP_LOGI(TAG, "eFuse Vref: รองรับ");
    } else {
        ESP_LOGI(TAG, "eFuse Vref: ไม่รองรับ");
    }
    return true;
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        ESP_LOGI(TAG, "ใช้การปรับเทียบแบบ Two Point Value");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        ESP_LOGI(TAG, "ใช้การปรับเทียบแบบ eFuse Vref");
    } else {
        ESP_LOGI(TAG, "ใช้การปรับเทียบแบบ Default Vref");
    }
}

// ฟังก์ชันอ่านค่าด้วย Oversampling
static float readADCOversampling(adc1_channel_t channel, int samples) 
{
    uint64_t sum = 0;
    for (int i = 0; i < samples; i++) {
        sum += adc1_get_raw(channel);
        vTaskDelay(pdMS_TO_TICKS(1));  // หน่วงเวลาเล็กน้อยระหว่างการอ่าน
    }
    return (float)sum / samples;
}

// ฟังก์ชัน Moving Average Filter
static float movingAverageFilter(float newValue) 
{
    if (!filterInitialized) {
        // เริ่มต้นตัวกรองครั้งแรก
        for (int i = 0; i < FILTER_SIZE; i++) {
            filterBuffer[i] = newValue;
        }
        filterSum = newValue * FILTER_SIZE;
        filterInitialized = true;
        return newValue;
    }

    // ลบค่าเก่าออกจาก sum
    filterSum -= filterBuffer[filterIndex];

    // เพิ่มค่าใหม่
    filterBuffer[filterIndex] = newValue;
    filterSum += newValue;

    // อัพเดท index
    filterIndex = (filterIndex + 1) % FILTER_SIZE;

    // คืนค่าเฉลี่ย
    return filterSum / FILTER_SIZE;
}

void app_main(void)
{
    // ตรวจสอบว่า Two Point หรือ Vref ถูกเขียนลงใน eFuse หรือไม่
    check_efuse();

    // กำหนดค่า ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_CHANNEL, ADC_ATTEN_DB_11);

    // ปรับเทียบ ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    ESP_LOGI(TAG, "ทดสอบการปรับปรุงความแม่นยำ ADC");
    ESP_LOGI(TAG, "เทคนิค: Oversampling + Moving Average Filter");
    ESP_LOGI(TAG, "Pin: GPIO34 (ADC1_CH6)");
    ESP_LOGI(TAG, "Oversamples: %d, Filter Size: %d", OVERSAMPLES, FILTER_SIZE);
    ESP_LOGI(TAG, "----------------------------------------");

    // อ่านค่า ADC อย่างต่อเนื่องด้วยเทคนิคต่างๆ
    while (1) {
        // อ่านค่าแบบปกติ (ครั้งเดียว)
        uint32_t rawValue = adc1_get_raw(SENSOR_CHANNEL);

        // อ่านค่าด้วย Oversampling
        float oversampledValue = readADCOversampling(SENSOR_CHANNEL, OVERSAMPLES);

        // ผ่านตัวกรอง Moving Average
        float filteredValue = movingAverageFilter(oversampledValue);

        // แปลงเป็นแรงดันด้วย calibration
        uint32_t raw_voltage_mv = esp_adc_cal_raw_to_voltage(rawValue, adc_chars);
        uint32_t oversampled_voltage_mv = esp_adc_cal_raw_to_voltage((uint32_t)oversampledValue, adc_chars);
        uint32_t filtered_voltage_mv = esp_adc_cal_raw_to_voltage((uint32_t)filteredValue, adc_chars);

        float rawVoltage = raw_voltage_mv / 1000.0;
        float oversampledVoltage = oversampled_voltage_mv / 1000.0;
        float filteredVoltage = filtered_voltage_mv / 1000.0;

        // แสดงผลเปรียบเทียบ
        ESP_LOGI(TAG, "=== การเปรียบเทียบ ===");
        ESP_LOGI(TAG, "Raw ADC: %d (%.3fV)", rawValue, rawVoltage);
        ESP_LOGI(TAG, "Oversampled: %.1f (%.3fV)", oversampledValue, oversampledVoltage);
        ESP_LOGI(TAG, "Filtered: %.1f (%.3fV)", filteredValue, filteredVoltage);
        ESP_LOGI(TAG, "");

        vTaskDelay(pdMS_TO_TICKS(2000));  // หน่วงเวลา 2 วินาที
    }
}