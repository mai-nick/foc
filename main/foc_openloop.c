#include "foc_openloop.h"
#include <math.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_rom_sys.h"
#include "soc/timer_group_struct.h"

#define PI 3.14159265358f
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static float voltage_power_supply = 15.0f;
static float shaft_angle = 0, open_loop_timestamp = 0;
static float zero_electric_angle = 0;
static float ina240_1_voltage_zero = 0, ina240_2_voltage_zero = 0;
static bool motor_enabled = false;
static const int pole_pairs = 7;

static bool last_button_state = true;
static int64_t last_button_us = 0;

static inline float _normalizeAngle(float angle) {
    float a = fmodf(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}
static inline float _electricalAngle(float shaft_angle) {
    return shaft_angle * pole_pairs;
}

static void setPwm(float Ua, float Ub, float Uc) {
    float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
    float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
    float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);
    uint32_t duty_max = (1 << LEDC_TIMER_8_BIT) - 1;
    if (motor_enabled) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, dc_a * duty_max);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, dc_b * duty_max);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, dc_c * duty_max);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
    }
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
}

static void setPhaseVoltage(float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);
    float Ualpha = -Uq * sinf(angle_el);
    float Ubeta  =  Uq * cosf(angle_el);
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (sqrtf(3) * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - sqrtf(3) * Ubeta) / 2 + voltage_power_supply / 2;
    setPwm(Ua, Ub, Uc);
}

/* ---- 硬件独立看门狗 (TG0, 2s超时, 寄存器喂狗零延时) ---- */
static void hw_wdt_init(void) {
    TIMERG0.wdtwprotect.val = 0x50D83AA1;
    TIMERG0.wdtconfig0.val = 0;
    TIMERG0.wdtconfig0.val = (1u << 30) | (80000u << 16) | 2000u;
    TIMERG0.wdtwprotect.val = 0;
}
static inline void hw_wdt_feed(void) {
    TIMERG0.wdtwprotect.val = 0x50D83AA1;
    TIMERG0.wdtfeed.val = 1;
    TIMERG0.wdtwprotect.val = 0;
}

static float velocityOpenloop(float target_velocity) {
    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    float Uq = voltage_power_supply / 3;
    if (motor_enabled)
        setPhaseVoltage(Uq, 0, _electricalAngle(shaft_angle));
    open_loop_timestamp = now_us;
    return Uq;
}

static float readCurrentINA240(int adc_channel, float voltage_zero) {
    int sum = 0;
    for (int i = 0; i < 10; i++) sum += adc1_get_raw(adc_channel);
    float adc = sum / 10.0f;
    float voltage = (adc * 3.3f) / 4095.0f * 1.05f;
    if (fabsf(voltage - voltage_zero) < 0.05f) voltage = voltage_zero;
    return (voltage - voltage_zero) / (0.01f * 50.0f);
}

void foc_init(void) {
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .freq_hz         = 30000,
        .duty_resolution = LEDC_TIMER_8_BIT,
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch[3] = {
        {.gpio_num = FOC_PWM_A_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
         .channel = LEDC_CHANNEL_0, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = FOC_PWM_B_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
         .channel = LEDC_CHANNEL_1, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = FOC_PWM_C_GPIO, .speed_mode = LEDC_LOW_SPEED_MODE,
         .channel = LEDC_CHANNEL_2, .timer_sel = LEDC_TIMER_0, .duty = 0},
    };
    for (int i = 0; i < 3; i++) ledc_channel_config(&ch[i]);

    gpio_set_direction(FOC_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FOC_EN_GPIO, 0);

    gpio_set_direction(FOC_BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(FOC_BUTTON_GPIO, GPIO_PULLUP_ONLY);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);

    printf("Calibrating INA240...\n");
    float sum_a = 0, sum_b = 0;
    for (int i = 0; i < 2000; i++) {
        sum_a += adc1_get_raw(ADC1_CHANNEL_5) * 3.3f / 4095.0f;
        sum_b += adc1_get_raw(ADC1_CHANNEL_4) * 3.3f / 4095.0f;
        esp_rom_delay_us(1000);
    }
    ina240_1_voltage_zero = sum_a / 2000;
    ina240_2_voltage_zero = sum_b / 2000;
    printf("INA240 Zero: V1=%.3f V, V2=%.3f V\n", ina240_1_voltage_zero, ina240_2_voltage_zero);

    open_loop_timestamp = esp_timer_get_time();
    hw_wdt_init();
    printf("FOC OK (15V,%dpp) HW_WDT=2s\n", pole_pairs);
}

void foc_set_enabled(bool en) {
    motor_enabled = en;
    gpio_set_level(FOC_EN_GPIO, en ? 1 : 0);
    if (!en) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    }
}
bool foc_get_enabled(void) { return motor_enabled; }

void foc_task(void *pvParameters) {
    int64_t last_control = 0, last_measure = 0, last_calib = 0;
    float voltage_a_sum = 0, voltage_b_sum = 0;
    int calib_count = 0;
    int wdt_cnt = 0;
    const int CALIB_SAMPLES = 100;

    while (1) {
        int64_t now = esp_timer_get_time();

        if (now - last_button_us >= 100000) {
            bool btn = gpio_get_level(FOC_BUTTON_GPIO);
            if (btn == 0 && last_button_state == 1) {
                foc_set_enabled(!motor_enabled);
                printf("Motor: %s\n", motor_enabled ? "ON" : "OFF");
                voltage_a_sum = voltage_b_sum = 0;
                calib_count = 0;
            }
            last_button_state = btn;
            last_button_us = now;
        }

        if (motor_enabled && now - last_control >= 100) {
            velocityOpenloop(50.0f);
            last_control = now;
        }

        if (!motor_enabled && now - last_calib >= 1000000) {
            float cur_a = readCurrentINA240(ADC1_CHANNEL_5, ina240_1_voltage_zero);
            float cur_b = readCurrentINA240(ADC1_CHANNEL_4, ina240_2_voltage_zero);
            if (fabsf(cur_a) > 0.01f || fabsf(cur_b) > 0.01f) {
                int s_a = 0, s_b = 0;
                for (int i = 0; i < CALIB_SAMPLES; i++) {
                    s_a += adc1_get_raw(ADC1_CHANNEL_5);
                    s_b += adc1_get_raw(ADC1_CHANNEL_4);
                    esp_rom_delay_us(100);
                }
                float v_a = (s_a / (float)CALIB_SAMPLES) * 3.3f / 4095.0f * 1.05f;
                float v_b = (s_b / (float)CALIB_SAMPLES) * 3.3f / 4095.0f * 1.05f;
                voltage_a_sum += v_a; voltage_b_sum += v_b;
                calib_count++;
                ina240_1_voltage_zero = voltage_a_sum / calib_count;
                ina240_2_voltage_zero = voltage_b_sum / calib_count;
                printf("Calib: V1_zero=%.3f V2_zero=%.3f\n",
                       ina240_1_voltage_zero, ina240_2_voltage_zero);
            }
            last_calib = now;
        }

        if (now - last_measure >= 100000) {
            float ia = readCurrentINA240(ADC1_CHANNEL_5, ina240_1_voltage_zero);
            float ib = readCurrentINA240(ADC1_CHANNEL_4, ina240_2_voltage_zero);
            printf("Ia=%.2f Ib=%.2f enabled=%d\n", ia, ib, motor_enabled);
            last_measure = now;
        }

        /* ~100ms喂一次硬件看门狗 (不影响控制时序) */
        if (++wdt_cnt >= 10000) { wdt_cnt = 0; hw_wdt_feed(); }
    }
}
