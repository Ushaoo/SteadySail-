/**
 * @file uart_command.c
 * @brief UART 命令接收实现（使用标准I/O）
 * 
 * 注：该实现使用ESP-IDF的标准输入接口，简化了依赖。
 * UART已通过main.c的初始化自动配置为标准输入。
 */

#include "uart_command.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

static const char *TAG = "UART_CMD";

#define UART_NUM_PLACEHOLDER 0  // 使用标准输入，不需要真实UART编号

// 静态变量存储单例（用于任务回调）
static uart_command_t *g_cmd_receiver = NULL;

/* ========== UART 接收任务函数 ========== */

static void uart_command_task(void *pvParameters)
{
    uart_command_t *cmd_receiver = (uart_command_t *)pvParameters;

    if (cmd_receiver == NULL) {
        ESP_LOGE(TAG, "UART command task: NULL parameter");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UART command receiving task started");

    while (1) {
        // 从标准输入读取一个字节（非阻塞式，超时100ms）
        int ch = fgetc(stdin);

        if (ch != EOF) {
            char c = (char)ch;
            
            // 如果缓冲区未满，添加字符
            if (cmd_receiver->rx_index < UART_CMD_BUF_SIZE - 1) {
                cmd_receiver->rx_buffer[cmd_receiver->rx_index++] = c;
            }

            // 检查是否收到换行符（完整命令）
            if (c == '\n') {
                // 移除\r和\n
                int end = cmd_receiver->rx_index - 1;
                while (end >= 0 && (cmd_receiver->rx_buffer[end] == '\r' || 
                                    cmd_receiver->rx_buffer[end] == '\n')) {
                    end--;
                }
                cmd_receiver->rx_buffer[end + 1] = '\0';

                // 处理这一行数据
                if (cmd_receiver->rx_index > 1) {  // 至少有一个字符
                    uart_command_process_line(cmd_receiver, cmd_receiver->rx_buffer);
                }

                // 清空缓冲区
                cmd_receiver->rx_index = 0;
            }
        }

        // 让出 CPU 给其他任务
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ========== 初始化函数 ========== */

esp_err_t uart_command_init(uart_command_t *cmd_receiver, uart_angle_callback_t callback)
{
    if (cmd_receiver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(cmd_receiver, 0, sizeof(uart_command_t));

    cmd_receiver->uart_num = UART_NUM_PLACEHOLDER;
    cmd_receiver->on_angle_cmd = callback;
    cmd_receiver->last_angle = 90.0f;
    cmd_receiver->rx_index = 0;

    ESP_LOGI(TAG, "UART command initialized: reading from stdin");

    return ESP_OK;
}

/* ========== 启动任务 ========== */

esp_err_t uart_command_start(uart_command_t *cmd_receiver, uint32_t priority, uint32_t stack_size)
{
    if (cmd_receiver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    BaseType_t ret = xTaskCreate(
        uart_command_task,
        "uart_cmd_task",
        stack_size,
        (void *)cmd_receiver,
        priority,
        NULL
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed for uart command task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UART command receiving task started (priority=%lu, stack=%lu bytes)", 
             priority, stack_size);

    return ESP_OK;
}

/* ========== 命令处理 ========== */

esp_err_t uart_command_process_line(uart_command_t *cmd_receiver, const char *line)
{
    if (cmd_receiver == NULL || line == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 跳过前导空格
    while (*line == ' ' || *line == '\t') {
        line++;
    }

    // 尝试解析为浮点数
    char *endptr = NULL;
    float angle = strtof(line, &endptr);

    // 检查解析是否成功
    if (endptr == line || *line == '\0') {
        ESP_LOGW(TAG, "Invalid angle command: '%s'", line);
        return ESP_ERR_INVALID_ARG;
    }

    // 限制在有效范围内 [0°, 180°]
    if (angle < 0.0f) angle = 0.0f;
    if (angle > 180.0f) angle = 180.0f;

    cmd_receiver->last_angle = angle;

    ESP_LOGI(TAG, "Received angle command: %.1f°", angle);

    // 调用回调函数
    if (cmd_receiver->on_angle_cmd != NULL) {
        cmd_receiver->on_angle_cmd(angle);
    }

    return ESP_OK;
}

/* ========== 查询函数 ========== */

float uart_command_get_last_angle(uart_command_t *cmd_receiver)
{
    if (cmd_receiver == NULL) {
        return 90.0f;
    }

    return cmd_receiver->last_angle;
}

/* ========== 关闭函数 ========== */

esp_err_t uart_command_deinit(uart_command_t *cmd_receiver)
{
    if (cmd_receiver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    g_cmd_receiver = NULL;

    ESP_LOGI(TAG, "UART command deinitialized");

    return ESP_OK;
}
