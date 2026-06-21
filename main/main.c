#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_task_wdt.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "foc_openloop.h"

/* ================ 硬件引脚定义 ================ */
#define LED_GPIO_48     GPIO_NUM_48
#define LED_GPIO_21     GPIO_NUM_21
#define LED_ON          0           /* 低电平亮（下拉亮） */
#define LED_OFF         1

/* ================ RTOS 对象句柄 ================ */
static QueueHandle_t    s_blink_queue;      /* 消息队列 — 任务间通信      */
static SemaphoreHandle_t s_sync_sem;        /* 二值信号量 — 任务同步      */
static SemaphoreHandle_t s_mutex;           /* 互斥量    — 共享资源保护  */

/* ================ GPIO 初始化 ================ */
static void gpio_init(void)
{
    /* 使用位掩码同时配置两个 LED 引脚 */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO_48) | (1ULL << LED_GPIO_21),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    /* 初始状态：LED 熄灭 */
    gpio_set_level(LED_GPIO_48, LED_OFF);
    gpio_set_level(LED_GPIO_21, LED_OFF);
}

/* ==========================================================
 * 任务 1 — LED48 闪烁（高优先级）
 * 展示：vTaskDelayUntil 精确周期、队列发送、信号量释放
 * ========================================================== */
static void vTaskBlink48(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t   ulLoopCount   = 0;

    while (1) {
        /* ---- 亮 ---- */
        gpio_set_level(LED_GPIO_48, LED_ON);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(80));

        /* ---- 灭（周期 500ms） ---- */
        gpio_set_level(LED_GPIO_48, LED_OFF);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(420));

        ulLoopCount++;

        /* ① 向队列发送消息 — 演示 RTOS 任务间通信 */
        uint32_t msg = 48;
        xQueueSend(s_blink_queue, &msg, 0);

        /* ② 释放信号量，通知同步任务 — 演示 RTOS 同步机制 */
        xSemaphoreGive(s_sync_sem);

        /* 每 10 次打印一次任务堆栈水位 — 演示 RTOS 诊断 API */
        if (ulLoopCount % 10 == 0) {
            UBaseType_t stack_free = uxTaskGetStackHighWaterMark(NULL);
            printf("[blink48] loop=%lu  stack_free=%u words\r\n",
                   ulLoopCount, (unsigned)stack_free);
            fflush(stdout);
        }
    }
}

/* ==========================================================
 * 任务 2 — LED21 闪烁（同样高优先级，但不同周期）
 * 展示：独立调度、不同时序
 * ========================================================== */
static void vTaskBlink21(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        gpio_set_level(LED_GPIO_21, LED_ON);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50));

        gpio_set_level(LED_GPIO_21, LED_OFF);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(250));   /* 周期 300ms */

        /* 向队列发送消息 */
        uint32_t msg = 21;
        xQueueSend(s_blink_queue, &msg, 0);
    }
}

/* ==========================================================
 * 任务 3 — 队列监控（中等优先级）
 * 展示：xQueueReceive 阻塞等待、消息统计
 * ========================================================== */
static void vTaskQueueMonitor(void *pvParameters)
{
    uint32_t msg;
    uint32_t count48 = 0, count21 = 0;

    while (1) {
        /* 阻塞等待队列消息 —— portMAX_DELAY 表示一直等待 */
        if (xQueueReceive(s_blink_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg == 48)
                count48++;
            else if (msg == 21)
                count21++;

            /* 每 30 条消息打印一次统计 */
            if ((count48 + count21) % 30 == 0) {
                printf("[Queue] LED48=%lu  LED21=%lu  total=%lu\r\n",
                       count48, count21, count48 + count21);
                fflush(stdout);
            }
        }
    }
}

/* ==========================================================
 * 任务 4 — 信号量同步监控（中等优先级）
 * 展示：xSemaphoreTake 等待同步、任务通知驱动的附加行为
 * ========================================================== */
static void vTaskSyncMonitor(void *pvParameters)
{
    uint32_t sync_count = 0;

    while (1) {
        /* 等待信号量 —— 由 vTaskBlink48 每周期释放 */
        if (xSemaphoreTake(s_sync_sem, portMAX_DELAY) == pdTRUE) {
            sync_count++;

            /* 每 15 次同步，主动介入：闪一下 LED48 证明信号量触发 */
            if (sync_count % 15 == 0) {
                /* 互斥量保护：防止与 blink48 同时操作 LED48 造成竞争 */
                if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    printf("[Semaphore] Synced %lu times — toggling LED48\r\n",
                           sync_count);
                    fflush(stdout);

                    gpio_set_level(LED_GPIO_48, LED_ON);
                    vTaskDelay(pdMS_TO_TICKS(120));
                    gpio_set_level(LED_GPIO_48, LED_OFF);

                    xSemaphoreGive(s_mutex);        /* ★ 释放互斥量 */
                } else {
                    printf("[Semaphore] Sync %lu — mutex timeout, skipped\r\n",
                           sync_count);
                    fflush(stdout);
                }
            }
        }
    }
}

/* ==========================================================
 * 任务 5 — 互斥量演示任务（低优先级）
 * 展示：互斥量优先级继承、共享资源保护
 * ========================================================== */
static void vTaskMutexDemo(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(2000));

        /* 尝试获取互斥量（模拟需要独占访问共享资源） */
        if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            printf("[Mutex] Got mutex — simulating shared resource access\r\n");
            fflush(stdout);

            /* 模拟独占操作（使用任务通知做延迟，更轻量） */
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

            xSemaphoreGive(s_mutex);
        } else {
            printf("[Mutex] Mutex busy — other task is using it\r\n");
            fflush(stdout);
        }
    }
}

/* ==========================================================
 * 主函数入口
 * ========================================================== */
void app_main(void)
{
    esp_task_wdt_deinit();  /* 电机控制紧致循环, 关闭任务看门狗 */

    /* UART0 DMA TX: printf 非阻塞 */
    uart_driver_install(UART_NUM_0, 4096, 0, 0, NULL, 0);
    _Pragma("GCC diagnostic push")
    _Pragma("GCC diagnostic ignored \"-Wdeprecated-declarations\"")
    esp_vfs_dev_uart_use_driver(UART_NUM_0);
    _Pragma("GCC diagnostic pop")
    printf("\r\n"
           "========================================\r\n"
           "  FOC OpenLoop — FreeRTOS LED Demo\r\n"
           "  Target : ESP32-S3\r\n"
           "  LED48  : 500ms blink\r\n"
           "  LED21  : 300ms blink\r\n"
           "  RTOS Features:\r\n"
           "    • Multi-task scheduling\r\n"
           "    • Queue (inter-task comm)\r\n"
           "    • Binary semaphore (sync)\r\n"
           "    • Mutex (shared resource)\r\n"
           "    • vTaskDelayUntil (precise timing)\r\n"
           "    • uxTaskGetStackHighWaterMark\r\n"
           "========================================\r\n\r\n");
    fflush(stdout);

    /* ---- 硬件初始化 ---- */
    gpio_init();
    foc_init();

    /* ---- 创建 RTOS 内核对象 ---- */

    /* 消息队列：最多容纳 10 条消息，每条 4 字节 (uint32_t) */
    s_blink_queue = xQueueCreate(10, sizeof(uint32_t));
    configASSERT(s_blink_queue);

    /* 二值信号量：用于任务间同步 */
    s_sync_sem = xSemaphoreCreateBinary();
    configASSERT(s_sync_sem);

    /* 互斥量：带优先级继承，保护共享资源 */
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    /* ---- 创建任务 ----
     * 参数：函数名, 任务名, 栈深度(字), 参数, 优先级, 句柄
     * 优先级：3=最高  2=高  1=中  0=低 (idle)
     */
    xTaskCreatePinnedToCore(vTaskBlink48,      "blink48",  2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskBlink21,      "blink21",  2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(vTaskQueueMonitor, "qmon",     2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskSyncMonitor,  "syncmon",  2048, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(vTaskMutexDemo,    "mutex",    2048, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(foc_task,          "foc_ctrl", 4096, NULL, 3, NULL, 1);

    /* 删除主任务本身，把 CPU 让给上述 RTOS 任务 */
    vTaskDelete(NULL);
}
