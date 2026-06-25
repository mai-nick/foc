#include "foc_openloop.h"
#include <math.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_rom_sys.h"
#include "soc/timer_group_struct.h"

/* =================================================================
 * ADC 通道映射 (ESP32-S3)
 *
 *   FOC_ADC_A_GPIO = GPIO5 → ADC1_CHANNEL_4  (相电流 A)
 *   FOC_ADC_B_GPIO = GPIO4 → ADC1_CHANNEL_3  (相电流 B)
 *
 *   ⚠ 不要搞混: GPIO 编号 ≠ ADC 通道编号
 *       GPIO5 是 ADC1_CHANNEL_4, 不是 CHANNEL_5!
 *       GPIO4 是 ADC1_CHANNEL_3, 不是 CHANNEL_4!
 * ================================================================= */

#define PI 3.14159265358f
#define SQRT3 1.73205080757f
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

static float voltage_power_supply = 15.0f;
static float shaft_angle = 0, open_loop_timestamp = 0;
static float zero_electric_angle = 0;
static float ina240_1_voltage_zero = 0, ina240_2_voltage_zero = 0;
static bool motor_enabled = false;
static const int pole_pairs = 7;

/* =================================================================
 * PI 控制器 — 用于电流环 (Id/Iq)
 * ================================================================= */
typedef struct {
    float integral;
    float Kp;
    float Ki;
    float limit;           /* 输出限幅               */
    float integral_limit;  /* 积分抗饱和限幅           */
} pi_controller_t;

static pi_controller_t pi_id = {0, 2.0f, 1.0f, 7.5f, 4.0f};
static pi_controller_t pi_iq = {0, 5.0f, 2.0f, 7.5f, 5.0f};

/* 电流环 PI 输出 (全局, 供调试打印) */
static float pi_Ud_out = 0, pi_Uq_out = 0;
static float pi_Id_actual = 0, pi_Iq_actual = 0;  /* 实际 Id/Iq (Park 变换后) */
static const float iq_target = 0.5f;
static const float id_target = 0.0f;
static const float iq_ff_voltage = 3.0f;  /* 前馈电压: 让启动瞬间就有转矩, 消除PI爬坡延迟 */

static bool last_button_state = true;
static int64_t last_button_us = 0;

/* =================================================================
 * 工具函数
 * ================================================================= */
static inline float _normalizeAngle(float angle) {
    float a = fmodf(angle, 2 * PI);
    return a >= 0 ? a : (a + 2 * PI);
}
static inline float _electricalAngle(float shaft_angle) {
    return shaft_angle * pole_pairs;
}

static float pi_update(pi_controller_t *pi, float error, float Ts) {
    pi->integral += error * Ts;
    pi->integral = _constrain(pi->integral, -pi->integral_limit, pi->integral_limit);
    return _constrain(pi->Kp * error + pi->Ki * pi->integral, -pi->limit, pi->limit);
}

static inline void pi_reset(pi_controller_t *pi) {
    pi->integral = 0;
}

/* =================================================================
 * PWM 输出
 * ================================================================= */
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

/* =================================================================
 * 逆 Park + 逆 Clark → 三相电压
 *
 *  逆 Park:  Uα = Ud·cosθ − Uq·sinθ
 *            Uβ = Ud·sinθ + Uq·cosθ
 *  逆 Clark: Ua = Uα + Vdc/2
 *            Ub = (−Uα + √3·Uβ)/2 + Vdc/2
 *            Uc = (−Uα − √3·Uβ)/2 + Vdc/2
 * ================================================================= */
static void setPhaseVoltage(float Uq, float Ud, float angle_el) {
    angle_el = _normalizeAngle(angle_el + zero_electric_angle);
    float ct = cosf(angle_el);
    float st = sinf(angle_el);
    /* 逆 Park (完整版本, 支持 Ud ≠ 0) */
    float Ualpha = Ud * ct - Uq * st;
    float Ubeta  = Ud * st + Uq * ct;
    /* 逆 Clark */
    float Ua = Ualpha + voltage_power_supply / 2;
    float Ub = (SQRT3 * Ubeta - Ualpha) / 2 + voltage_power_supply / 2;
    float Uc = (-Ualpha - SQRT3 * Ubeta) / 2 + voltage_power_supply / 2;
    setPwm(Ua, Ub, Uc);
}

/* =================================================================
 * 硬件独立看门狗 (TG0, 2s 超时)
 * ================================================================= */
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

/* =================================================================
 * 电流读取
 * ================================================================= */

/* 快速读取 (3次采样, 用于电流环控制) */
static float readCurrentFast(int adc_channel, float voltage_zero) {
    int sum = 0;
    for (int i = 0; i < 3; i++) sum += adc1_get_raw(adc_channel);
    float voltage = (sum / 3.0f * 3.3f / 4095.0f) * 1.05f;
    return (voltage - voltage_zero) / 0.5f;  /* 0.01Ω × 50 倍增益 = 0.5 */
}

/* 精确读取 (10次采样 + 死区, 用于调试打印) */
static float readCurrentINA240(int adc_channel, float voltage_zero) {
    int sum = 0;
    for (int i = 0; i < 10; i++) sum += adc1_get_raw(adc_channel);
    float adc = sum / 10.0f;
    float voltage = (adc * 3.3f) / 4095.0f * 1.05f;
    if (fabsf(voltage - voltage_zero) < 0.05f) voltage = voltage_zero;
    return (voltage - voltage_zero) / 0.5f;
}

/* =================================================================
 * 开环角度更新 (10kHz)
 * ================================================================= */
static void updateAngle(float target_velocity) {
    int64_t now_us = esp_timer_get_time();
    float Ts = (now_us - open_loop_timestamp) * 1e-6f;
    if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    shaft_angle = _normalizeAngle(shaft_angle + target_velocity * Ts);
    open_loop_timestamp = now_us;
}

/* =================================================================
 * 电流环 FOC 控制 (1kHz)
 *
 *  读取相电流 → Clark → Park → PI(Id/Iq) → 逆 Park → PWM
 * ================================================================= */
static void currentFocLoop(float angle_el, float Ts) {
    /* 1. 读取两相电流 */
    float ia = readCurrentFast(ADC1_CHANNEL_4, ina240_1_voltage_zero);
    float ib = readCurrentFast(ADC1_CHANNEL_3, ina240_2_voltage_zero);

    /* 2. Clark 变换: Iα = Ia, Iβ = (Ia + 2·Ib) / √3 */
    float i_alpha = ia;
    float i_beta  = (ia + 2.0f * ib) / SQRT3;

    /* 3. Park 变换: Id, Iq */
    float ct = cosf(angle_el);
    float st = sinf(angle_el);
    float id =  i_alpha * ct + i_beta * st;
    float iq = -i_alpha * st + i_beta * ct;

    /* 保存实际电流供调试打印 */
    pi_Id_actual = id;
    pi_Iq_actual = iq;

    /* 4. PI 控制: 误差 → Ud/Uq + 前馈电压(让电机启动即有转矩) */
    float Ud_ref = pi_update(&pi_id, id_target - id, Ts);
    float Uq_ref = pi_update(&pi_iq, iq_target - iq, Ts) + iq_ff_voltage;

    /* 保存 PI 输出供调试打印 */
    pi_Ud_out = Ud_ref;
    pi_Uq_out = Uq_ref;

    /* 5. 逆 Park + 逆 Clark → 三相 PWM */
    setPhaseVoltage(Uq_ref, Ud_ref, angle_el);
}

/* =================================================================
 * 公共接口
 * ================================================================= */

void foc_init(void) {
    /* ---- PWM 配置 (30kHz, 8bit) ---- */
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

    /* ---- 使能引脚 & 按键 ---- */
    gpio_set_direction(FOC_EN_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FOC_EN_GPIO, 0);

    gpio_set_direction(FOC_BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(FOC_BUTTON_GPIO, GPIO_PULLUP_ONLY);

    /* ---- ADC 配置 ---- */
    adc1_config_width(ADC_WIDTH_BIT_12);
    /* ADC_A=GPIO5→CH4  ADC_B=GPIO4→CH3 */
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_12);

    /* ---- INA240 零位校准 (修复: 加上 1.05 增益因子, 与读取函数一致) ---- */
    printf("Calibrating INA240...\n");
    float sum_a = 0, sum_b = 0;
    for (int i = 0; i < 2000; i++) {
        sum_a += adc1_get_raw(ADC1_CHANNEL_4) * 3.3f / 4095.0f * 1.05f;
        sum_b += adc1_get_raw(ADC1_CHANNEL_3) * 3.3f / 4095.0f * 1.05f;
        esp_rom_delay_us(1000);
    }
    ina240_1_voltage_zero = sum_a / 2000;
    ina240_2_voltage_zero = sum_b / 2000;
    printf("INA240 Zero: V1=%.3f V, V2=%.3f V\n",
           ina240_1_voltage_zero, ina240_2_voltage_zero);

    open_loop_timestamp = esp_timer_get_time();
    hw_wdt_init();
    printf("FOC OK (%.0fV, %dpp, Iq_target=%.1fA) HW_WDT=2s\n",
           voltage_power_supply, pole_pairs, iq_target);
}

void foc_set_enabled(bool en) {
    motor_enabled = en;
    gpio_set_level(FOC_EN_GPIO, en ? 1 : 0);
    /* 切换时重置 PI 积分器, 防止积分过冲 */
    pi_reset(&pi_id);
    pi_reset(&pi_iq);
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

/* =================================================================
 * FOC 主任务
 * ================================================================= */
void foc_task(void *pvParameters) {
    int64_t last_angle    = 0;
    int64_t last_curr_ctrl = 0;
    int64_t last_measure  = 0;
    int64_t last_calib    = 0;
    int wdt_cnt = 0;

    while (1) {
        int64_t now = esp_timer_get_time();

        /* ---- 按键检测 (100ms 防抖) ---- */
        if (now - last_button_us >= 100000) {
            bool btn = gpio_get_level(FOC_BUTTON_GPIO);
            if (btn == 0 && last_button_state == 1) {
                foc_set_enabled(!motor_enabled);
                printf("Motor: %s (Iq_tgt=%.1fA FF=%.1fV)\n",
                       motor_enabled ? "ON (current loop)" : "OFF",
                       iq_target, iq_ff_voltage);
            }
            last_button_state = btn;
            last_button_us = now;
        }

        /* ---- 开环角度更新 (10kHz ≈ 100μs) ---- */
        if (motor_enabled && now - last_angle >= 100) {
            updateAngle(50.0f);
            last_angle = now;
        }

        /* ---- 电流环控制 (1kHz ≈ 1000μs) ---- */
        if (motor_enabled && now - last_curr_ctrl >= 1000) {
            float Ts = (now - last_curr_ctrl) * 1e-6f;
            if (Ts > 0.01f) Ts = 0.001f;
            currentFocLoop(_electricalAngle(shaft_angle), Ts);
            last_curr_ctrl = now;
        }

        /* ---- 停止时零位自动校准 (每 1s, 无条件校准) ---- */
        if (!motor_enabled && now - last_calib >= 1000000) {
            int s_a = 0, s_b = 0;
            for (int i = 0; i < 100; i++) {
                s_a += adc1_get_raw(ADC1_CHANNEL_4);
                s_b += adc1_get_raw(ADC1_CHANNEL_3);
                esp_rom_delay_us(100);
            }
            float v_a = (s_a / 100.0f) * 3.3f / 4095.0f * 1.05f;
            float v_b = (s_b / 100.0f) * 3.3f / 4095.0f * 1.05f;
            /* 一阶低通滤波 (α=0.1), 平滑更新零位, 抑制噪声 */
            ina240_1_voltage_zero += 0.1f * (v_a - ina240_1_voltage_zero);
            ina240_2_voltage_zero += 0.1f * (v_b - ina240_2_voltage_zero);
            printf("Zero calib: V1=%.3f V2=%.3f\n",
                   ina240_1_voltage_zero, ina240_2_voltage_zero);
            last_calib = now;
        }

        /* ---- 状态打印 (100ms) ---- */
        if (now - last_measure >= 100000) {
            float ia = readCurrentINA240(ADC1_CHANNEL_4, ina240_1_voltage_zero);
            float ib = readCurrentINA240(ADC1_CHANNEL_3, ina240_2_voltage_zero);
            if (motor_enabled)
                printf("Ia=%.2f Ib=%.2f ON  Id=%.2f Iq=%.2f Uq=%.1f\n",
                       ia, ib, pi_Id_actual, pi_Iq_actual, pi_Uq_out);
            else
                printf("Ia=%.2f Ib=%.2f OFF\n", ia, ib);
            last_measure = now;
        }

        /* ---- 硬件看门狗喂狗 (~100ms 级别) ---- */
        if (++wdt_cnt >= 10000) { wdt_cnt = 0; hw_wdt_feed(); }
    }
}
