#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Pin mapping */
#define FOC_PWM_A_GPIO      GPIO_NUM_10
#define FOC_PWM_B_GPIO      GPIO_NUM_11
#define FOC_PWM_C_GPIO      GPIO_NUM_12
#define FOC_EN_GPIO         GPIO_NUM_9
#define FOC_ADC_A_GPIO      GPIO_NUM_5
#define FOC_ADC_B_GPIO      GPIO_NUM_4
#define FOC_BUTTON_GPIO     GPIO_NUM_14

void foc_init(void);
void foc_set_enabled(bool en);
bool foc_get_enabled(void);
void foc_task(void *pvParameters);
