/*
 * LDR + Buzzer + LED (ภาษาไทย) + ปุ่มกด:
 * - แสงน้อย -> หรี่ LED, แสงมาก -> เพิ่มความสว่าง
 * - ต่ำกว่าเกณฑ์ -> Buzzer ดัง (มีฮิสเทอรีซีส)
 * - ปุ่ม BOOT (GPIO0):
 *      กดสั้น  <1s  -> ปิดเสียงเตือนชั่วคราว 30s (Mute)
 *      กดค้าง >=1s  -> คาลิเบรตเกณฑ์จากสภาพแสงปัจจุบัน (TH_ON/TH_OFF)
 * Log: "ADC: <raw> | แรงดัน: <V>V | ระดับแสง: <เปอร์เซ็นต์>% | สถานะ: <ข้อความ>"
 *
 * วงจรเซนเซอร์ตามเดิม:
 *   3V3 -> LDR -> โหนดสัญญาณ -> (R=470Ω->GND, C=1nF->GND) -> GPIO35 (ADC1_CH7)
 * Buzzer (Active) + Q1 2N3904: 3V3 -> (+)Buzzer, (-)Buzzer -> C Q1, E Q1->GND, B Q1<-R10k<-GPIO18
 * LED สถานะ: GPIO2 -> LED -> R 220~470Ω -> GND
 */

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"          // legacy ADC (IDF 5.x ยังใช้ได้)
#include "esp_adc_cal.h"         // legacy calibration
#include "esp_log.h"
#include "esp_err.h"
#include "esp_rom_sys.h"         // esp_rom_delay_us()

#define TAG "LDR_BTN_BUZZ_LED_TH"

/* --------- พินใช้งาน --------- */
#define LDR_ADC_CHANNEL   ADC1_CHANNEL_7     // GPIO35
#define LDR_ADC_GPIO      35
#define BUZZER_GPIO       GPIO_NUM_18        // ผ่าน Q1 2N3904 (Active-high)
#define LED_GPIO          GPIO_NUM_2         // LED สถานะ
#define BTN_GPIO          GPIO_NUM_0         // ปุ่ม BOOT บนบอร์ด (pull-up, กด=0)

/* --------- พารามิเตอร์อ่านค่า/กรอง --------- */
#define DEFAULT_VREF      1100               // mV
#define OVERSAMPLES       64                 // จำนวนครั้งอ่าน/รอบ
#define FILTER_SIZE       10                 // Moving Average
#define SAMPLE_PERIOD_MS  100                // คาบการอ่าน/รายงาน (ทำให้ปุ่มตอบสนองไวขึ้น)

/* --------- PWM (LEDC) สำหรับ LED --------- */
#define LEDC_TIMER_IDX    LEDC_TIMER_0
#define LEDC_MODE_SEL     LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_IDX  LEDC_CHANNEL_0
#define LEDC_DUTY_RES     LEDC_TIMER_13_BIT  // 13-bit (0..8191)
#define LEDC_FREQ_HZ      5000

/* --------- เกณฑ์แจ้งเตือน (ค่าเริ่มต้น) --------- */
#define THRESHOLD_MV_ON_DEFAULT   1000       // < เกณฑ์นี้ = มืด -> เปิด Buzzer
#define THRESHOLD_MV_OFF_DEFAULT  1200       // > เกณฑ์นี้ = หายมืด -> ปิด Buzzer

/* --------- สเกลแรงดันสำหรับคิดเปอร์เซ็นต์แสง --------- */
#define MV_MIN_SCALE      50                 // ~มืดสุด
#define MV_MAX_SCALE      3300               // ~สว่างสุด (3.3V)

/* --------- ปุ่มกด --------- */
#define BTN_DEBOUNCE_MS   30
#define BTN_LONG_MS       1000
#define MUTE_MS           30000              // กดสั้น -> ปิดเสียง 30 วินาที

/* --------- ตัวแปรภายใน --------- */
static esp_adc_cal_characteristics_t *adc_chars;
static bool  filt_init  = false;
static bool  buzzer_on  = false;

/* เกณฑ์ runtime (เปลี่ยนได้ด้วยการกดปุ่มค้าง) */
static int   th_on_mv  = THRESHOLD_MV_ON_DEFAULT;
static int   th_off_mv = THRESHOLD_MV_OFF_DEFAULT;

/* mute จนถึงเวลา tick นี้ */
static TickType_t mute_until_tick = 0;

/* --------- ฟังก์ชันช่วย --------- */
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

/* ปุ่ม: คืนค่าเหตุการณ์ (ไม่มี/กดสั้น/กดค้าง) */
typedef enum { BTN_EVT_NONE=0, BTN_EVT_SHORT, BTN_EVT_LONG } btn_evt_t;

static btn_evt_t poll_button(void) {
    static int stable = 1;  // pull-up -> ว่าง = 1
    static TickType_t last_change = 0;
    static TickType_t press_tick  = 0;

    TickType_t now = xTaskGetTickCount();
    int level = gpio_get_level(BTN_GPIO);

    if (level != stable) {
        // debounce
        if ((now - last_change) * portTICK_PERIOD_MS >= BTN_DEBOUNCE_MS) {
            last_change = now;
            stable = level;
            if (stable == 0) {
                // กดลง
                press_tick = now;
            } else {
                // ปล่อยปุ่ม -> คำนวณเวลาที่กด
                TickType_t held_ms = (now - press_tick) * portTICK_PERIOD_MS;
                if (held_ms >= BTN_LONG_MS) return BTN_EVT_LONG;
                if (held_ms >= BTN_DEBOUNCE_MS) return BTN_EVT_SHORT;
            }
        }
    }
    return BTN_EVT_NONE;
}

/* คาลิเบรตเกณฑ์จากสภาพแสงปัจจุบัน */
static void recalibrate_thresholds(adc1_channel_t ch) {
    const int N = 20;
    uint64_t sum_mv = 0;
    for (int i = 0; i < N; ++i) {
        uint32_t raw = oversample_read(ch, OVERSAMPLES);
        uint32_t mv  = esp_adc_cal_raw_to_voltage(raw, adc_chars);
        sum_mv += mv;
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    int base = (int)(sum_mv / N);
    // ตั้งฮิสเทอรีซีสรอบ ๆ ค่าฐาน
    th_on_mv  = base - 200;  // ต่ำกว่านี้ถือว่ามืดมาก (เปิด buzzer)
    th_off_mv = base + 100;  // สูงกว่านี้ถือว่ากลับสว่าง (ปิด buzzer)

    if (th_on_mv  < MV_MIN_SCALE) th_on_mv  = MV_MIN_SCALE;
    if (th_off_mv > MV_MAX_SCALE) th_off_mv = MV_MAX_SCALE;
    if (th_off_mv <= th_on_mv + 50) th_off_mv = th_on_mv + 50; // กันชน

    ESP_LOGW(TAG, "คาลิเบรตใหม่จากสภาพแสง: base=%dmV -> TH_ON=%dmV, TH_OFF=%dmV",
             base, th_on_mv, th_off_mv);
}

/* --------- main --------- */
void app_main(void) {
    ESP_LOGI(TAG,
        "เริ่มระบบ | LDR: GPIO%d(ADC1_CH7) | BUZZER: GPIO%d | LED: GPIO%d | BTN: GPIO%d | OVERSAMPLES=%d FILTER_SIZE=%d",
        LDR_ADC_GPIO, BUZZER_GPIO, LED_GPIO, BTN_GPIO, OVERSAMPLES, FILTER_SIZE);

    /* ตั้ง BUZZER เป็นเอาต์พุต */
    gpio_config_t buz = {
        .pin_bit_mask = 1ULL << BUZZER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0, .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&buz));
    gpio_set_level(BUZZER_GPIO, 0);

    /* ตั้ง LED PWM */
    ledc_init();

    /* ตั้งปุ่ม (pull-up, กด=0) */
    gpio_config_t btn = {
        .pin_bit_mask = 1ULL << BTN_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&btn));

    /* ตั้ง ADC legacy + calibration (ใช้ DB_12 แทน DB_11) */
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_12); // ~0..3.3V
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    (void)esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    const uint32_t duty_max = (1U << LEDC_DUTY_RES) - 1U;

    while (1) {
        /* --- อ่านเหตุการณ์ปุ่ม --- */
        btn_evt_t evt = poll_button();
        TickType_t now = xTaskGetTickCount();
        if (evt == BTN_EVT_SHORT) {
            mute_until_tick = now + pdMS_TO_TICKS(MUTE_MS);
            ESP_LOGW(TAG, "ปุ่ม: กดสั้น -> ปิดเสียงเตือนชั่วคราว %d ms", (int)MUTE_MS);
        } else if (evt == BTN_EVT_LONG) {
            recalibrate_thresholds(LDR_ADC_CHANNEL);
        }

        /* 1) อ่านค่า + ทำให้เรียบ */
        uint32_t raw_os = oversample_read(LDR_ADC_CHANNEL, OVERSAMPLES);
        float raw_ma = moving_average((float)raw_os);
        uint32_t raw = (uint32_t)raw_ma;

        /* 2) แปลงเป็น mV และเปอร์เซ็นต์แสง */
        uint32_t mv = esp_adc_cal_raw_to_voltage(raw, adc_chars);
        float mv_c = clampf((float)mv, MV_MIN_SCALE, MV_MAX_SCALE);
        float light_pct = 100.0f * (mv_c - MV_MIN_SCALE) / (float)(MV_MAX_SCALE - MV_MIN_SCALE);
        if (light_pct < 0) light_pct = 0; else if (light_pct > 100) light_pct = 100;

        /* 3) LED สว่างตามแสง (สู้แสง): แสงมาก -> LED มาก */
        uint32_t duty = (uint32_t)((light_pct / 100.0f) * duty_max);
        set_led_duty(duty);

        /* 4) เปิด/ปิด Buzzer ด้วยฮิสเทอรีซีส + mute */
        bool mute_active = (now < mute_until_tick);
        if (!buzzer_on && mv < th_on_mv) {
            buzzer_on = true;
        } else if (buzzer_on && mv > th_off_mv) {
            buzzer_on = false;
        }
        gpio_set_level(BUZZER_GPIO, (buzzer_on && !mute_active) ? 1 : 0);

        /* 5) รายงานภาษาไทยสั้น ๆ */
        const char* status;
        if (mute_active) {
            status = "ปิดเสียงชั่วคราว (Mute)";
        } else if (buzzer_on) {
            status = "แสงน้อยกว่าเกณฑ์ -> Buzzer ดัง";
        } else if (mv < th_off_mv && mv >= th_on_mv) {
            status = "เข้าใกล้เกณฑ์";
        } else {
            status = "แสงปกติ";
        }

        ESP_LOGI(TAG, "ADC: %u | แรงดัน: %.2fV | ระดับแสง: %.1f%% | สถานะ: %s (TH_ON=%dmV, TH_OFF=%dmV)",
                 raw, mv / 1000.0f, light_pct, status, th_on_mv, th_off_mv);

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}