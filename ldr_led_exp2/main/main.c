#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

// กำหนด pin ที่ใช้
#define LDR_CHANNEL ADC1_CHANNEL_7  // GPIO35 (ADC1_CH7)
#define DEFAULT_VREF    1100        // ใช้ adc2_vref_to_gpio() เพื่อให้ได้ค่าประมาณที่ดีกว่า
#define NO_OF_SAMPLES   64          // การสุ่มสัญญาณหลายครั้ง

static const char *TAG = "ADC_LDR";
static esp_adc_cal_characteristics_t *adc_chars;

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

void app_main(void)
{
    // ตรวจสอบว่า Two Point หรือ Vref ถูกเขียนลงใน eFuse หรือไม่
    check_efuse();

    // กำหนดค่า ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_CHANNEL, ADC_ATTEN_DB_11);

    // ปรับเทียบ ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    ESP_LOGI(TAG, "ทดสอบเซนเซอร์แสง LDR ของ ESP32");
    ESP_LOGI(TAG, "Pin: GPIO35 (ADC1_CH7)");
    ESP_LOGI(TAG, "ช่วง: 0-3.3V");
    ESP_LOGI(TAG, "ความละเอียด: 12-bit (0-4095)");
    ESP_LOGI(TAG, "Attenuation: 11dB");
    ESP_LOGI(TAG, "------------------------");

    // อ่านค่า ADC จาก LDR อย่างต่อเนื่อง
    while (1) {
        uint32_t adc_reading = 0;

        // การสุ่มสัญญาณหลายครั้ง
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)LDR_CHANNEL);
        }
        adc_reading /= NO_OF_SAMPLES;

        // แปลง adc_reading เป็นแรงดันในหน่วย mV
        uint32_t voltage_mv = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        float voltage = voltage_mv / 1000.0;

        // แปลงเป็นระดับแสง (สมมติ)
        float lightLevel = (adc_reading / 4095.0) * 100.0;

        // กำหนดสถานะแสง
        const char* lightStatus;
        if (lightLevel < 20) {
            lightStatus = "มืด";
        } else if (lightLevel < 50) {
            lightStatus = "แสงน้อย";
        } else if (lightLevel < 80) {
            lightStatus = "แสงปานกลาง";
        } else {
            lightStatus = "แสงจ้า";
        }

        // แสดงผลลัพธ์
        ESP_LOGI(TAG, "ADC: %d | แรงดัน: %.2fV | ระดับแสง: %.1f%% | สถานะ: %s", 
                 adc_reading, voltage, lightLevel, lightStatus);

        vTaskDelay(pdMS_TO_TICKS(1000));  // หน่วงเวลา 1 วินาที
    }
}
