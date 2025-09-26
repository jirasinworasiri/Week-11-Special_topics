/*
 * ADC + LDR (ภาษาไทย): Oversampling + Moving Average + PWM LED + Limit Alert
 * วงจรตามภาพ: 3V3 -> LDR -> โหนดสัญญาณ -> (R470->GND, C1 1nF->GND) -> GPIO35
 * LED: GPIO18 -> LED -> R470 -> GND
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"          // legacy ADC (ยังใช้ได้)
#include "esp_adc_cal.h"         // legacy calibration
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"         // esp_rom_delay_us()

#define TAG "LDR_AUTO_LED_TH"

/* -------------------- พินใช้งาน -------------------- */
#define LED_GPIO         GPIO_NUM_18          // เปลี่ยนเป็น GPIO_NUM_2 หากใช้ LED ออนบอร์ด
#define SENSOR_CHANNEL   ADC1_CHANNEL_7       // GPIO35 (ADC1_CH7) ตามภาพวงจร
#define SENSOR_GPIO      35

/* -------------------- พารามิเตอร์อ่านค่า -------------------- */
#define DEFAULT_VREF     1100                 // mV (ใช้คาลิเบรต legacy)
#define OVERSAMPLES      64                   // เที่ยวอ่านต่อรอบสำหรับ oversampling
#define FILTER_SIZE      10                   // Moving Average ขนาดบัฟเฟอร์
#define SAMPLE_PERIOD_MS 200                  // คาบการรายงาน/อัปเดต

/* -------------------- PWM (LEDC) -------------------- */
#define LEDC_TIMER_IDX   LEDC_TIMER_0
#define LEDC_MODE_SEL    LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_IDX LEDC_CHANNEL_0
#define LEDC_DUTY_RES    LEDC_TIMER_13_BIT    // 13-bit (0..8191)
#define LEDC_FREQ_HZ     5000

/* -------------------- นโยบายควบคุม/การเตือน -------------------- */
/* สเกลแรงดันคาดหวังของวงจรนี้ ~ 0..3300 mV (มืด -> mV ต่ำ, สว่าง -> mV สูง) */
#define MV_MIN_SCALE     0                    // ปรับได้หากอยากบีบสเกล
#define MV_MAX_SCALE     3300

/* แบ่งเขตการตัดสินใจ (เปอร์เซ็นต์แสง) */
#define LOW_LIGHT_BAND_PCT    30.0f          // แสงน้อย: หรี่ไฟ
#define HIGH_LIGHT_BAND_PCT   70.0f          // แสงมาก: เพิ่มไฟ

/* ขีดจำกัดการเตือน (เปอร์เซ็นต์แสง) และเงื่อนไขความต่อเนื่อง */
#define ALERT_DARK_PCT        5.0f           // มืดมากผิดปกติ
#define ALERT_BRIGHT_PCT      95.0f          // สว่างมากผิดปกติ
#define ALERT_HOLD_COUNT      10             // ต้องเกินลิมิตต่อเนื่องกี่รอบจึงเตือน
#define ALERT_BLINK_TIMES     3              // จำนวนครั้งกระพริบตอนเตือน

/* -------------------- ตัวแปรภายใน -------------------- */
static esp_adc_cal_characteristics_t *adc_chars;
static bool filt_init = false;

/* เคาน์เตอร์สำหรับการเตือนและสถานะล่าสุด */
typedef enum { ALERT_NONE=0, ALERT_DARK, ALERT_BRIGHT } alert_t;
static int dark_cnt = 0, bright_cnt = 0;
static alert_t last_alert = ALERT_NONE;

/* -------------------- ฟังก์ชันช่วย -------------------- */
static void ledc_init(void) {
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_MODE_SEL,
        .timer_num        = LEDC_TIMER_IDX,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    ledc_channel_config_t cconfig = {
        .gpio_num       = LED_GPIO,
        .speed_mode     = LEDC_MODE_SEL,
        .channel        = LEDC_CHANNEL_IDX,
        .intr_type      = LEDC_INTR_DISABLE,
        .timer_sel      = LEDC_TIMER_IDX,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&cconfig));
}

static inline void set_led_duty(uint32_t duty) {
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE_SEL, LEDC_CHANNEL_IDX, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE_SEL, LEDC_CHANNEL_IDX));
}

static uint32_t oversample_read(adc1_channel_t ch, int samples) {
    uint64_t sum = 0;
    for (int i = 0; i < samples; ++i) {
        sum += adc1_get_raw(ch);
        esp_rom_delay_us(100);
    }
    return (uint32_t)(sum / samples);
}

static float moving_average(float x) {
    static float buf[FILTER_SIZE];
    static int idx = 0, count = 0;
    if (!filt_init) {
        for (int i = 0; i < FILTER_SIZE; ++i) buf[i] = x;
        idx = 0; count = FILTER_SIZE; filt_init = true;
        return x;
    }
    buf[idx] = x;
    idx = (idx + 1) % FILTER_SIZE;
    float s = 0.0f;
    for (int i = 0; i < count; ++i) s += buf[i];
    return s / count;
}

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void blink_alert(int times) {
    const uint32_t duty_max = (1U << LEDC_DUTY_RES) - 1U;
    for (int i = 0; i < times; ++i) {
        set_led_duty(duty_max); vTaskDelay(pdMS_TO_TICKS(120));
        set_led_duty(0);        vTaskDelay(pdMS_TO_TICKS(120));
    }
}

/* -------------------- main -------------------- */
void app_main(void) {
    ESP_LOGI(TAG,
        "เริ่มทำงาน | LDR: GPIO%d (ADC1_CH7) | LED PWM: GPIO%d | OVERSAMPLES=%d FILTER_SIZE=%d",
        SENSOR_GPIO, LED_GPIO, OVERSAMPLES, FILTER_SIZE);

    /* LEDC */
    ledc_init();

    /* ADC legacy + calibration */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(SENSOR_CHANNEL, ADC_ATTEN_DB_12);   // ใช้ DB_12 (เทียบเท่า ~3.3V ช่วงบน)
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    (void)esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    const uint32_t duty_max = (1U << LEDC_DUTY_RES) - 1U;

    while (1) {
        /* 1) อ่านค่า + ทำให้เรียบ */
        uint32_t raw_os = oversample_read(SENSOR_CHANNEL, OVERSAMPLES);
        float raw_ma = moving_average((float)raw_os);
        uint32_t raw = (uint32_t)raw_ma;

        /* 2) แปลงเป็น mV และเปอร์เซ็นต์แสง (0..100%) */
        uint32_t mv = esp_adc_cal_raw_to_voltage(raw, adc_chars);
        float mv_clamped = clampf((float)mv, MV_MIN_SCALE, MV_MAX_SCALE);
        float light_pct = 100.0f * (mv_clamped - MV_MIN_SCALE) / (float)(MV_MAX_SCALE - MV_MIN_SCALE);
        light_pct = clampf(light_pct, 0.0f, 100.0f);

        /* 3) ตัดสินใจสถานะเพื่อรายงาน */
        const char *status;
        if (light_pct < LOW_LIGHT_BAND_PCT)      status = "แสงน้อย: หรี่ LED";
        else if (light_pct > HIGH_LIGHT_BAND_PCT)status = "แสงมาก: เพิ่มความสว่าง LED";
        else                                     status = "แสงปานกลาง: ปรับตามสัดส่วน";

        /* 4) คำนวณ PWM (เพิ่มสว่างตามแสงภายนอก) */
        uint32_t duty = (uint32_t)( (light_pct / 100.0f) * duty_max );
        set_led_duty(duty);

        /* 5) ระบบเตือนเมื่อเกินขีดจำกัดต่อเนื่อง */
        alert_t now = ALERT_NONE;
        if (light_pct <= ALERT_DARK_PCT)      { dark_cnt++;  bright_cnt = 0; if (dark_cnt >= ALERT_HOLD_COUNT)  now = ALERT_DARK; }
        else if (light_pct >= ALERT_BRIGHT_PCT){ bright_cnt++; dark_cnt = 0; if (bright_cnt >= ALERT_HOLD_COUNT) now = ALERT_BRIGHT; }
        else                                   { dark_cnt = 0; bright_cnt = 0; }

        if (now != ALERT_NONE && now != last_alert) {
            if (now == ALERT_DARK)   { ESP_LOGW(TAG, "เตือน: มืดมากผิดปกติ (%.1f%%) ต่อเนื่อง", light_pct); }
            if (now == ALERT_BRIGHT) { ESP_LOGW(TAG, "เตือน: สว่างมากผิดปกติ (%.1f%%) ต่อเนื่อง", light_pct); }
            blink_alert(ALERT_BLINK_TIMES);
            last_alert = now;
        }
        if (now == ALERT_NONE) last_alert = ALERT_NONE; // เคลียร์เมื่อกลับสู่ช่วงปกติ

        /* 6) พิมพ์สถานะภาษาไทยแบบสั้นตามที่ต้องการ */
        ESP_LOGI(TAG, "ADC: %u | แรงดัน: %.2fV | ระดับแสง: %.1f%% | สถานะ: %s | PWM: %4u/ %u",
                 raw, mv / 1000.0f, light_pct, status, duty, duty_max);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}