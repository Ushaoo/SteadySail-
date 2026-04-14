/**
 * @file uart_command.h
 * @brief UART 命令接收 - 接收推进器角度指令
 * 
 * 串口协议：
 * - 波特率：115200
 * - 数据格式：纯文本数字 + 回车
 * - 命令格式：角度值 (0-180)
 * 
 * 示例：
 * - 发送 "90\r\n"   -> 推进器竖直向下
 * - 发送 "0\r\n"    -> 推进器水平向前
 * - 发送 "180\r\n"  -> 推进器水平向后
 */

#ifndef UART_COMMAND_H
#define UART_COMMAND_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdint.h>

/* ========== 常量定义 ========== */

#define UART_CMD_BUF_SIZE  128
#define UART_BAUDRATE      115200

/* ========== 回调函数指针 ========== */

/**
 * @brief 角度命令回调函数
 * 
 * 当成功接收到有效的角度命令时调用
 * 
 * @param angle 接收到的角度值 (0-180)
 */
typedef void (*uart_angle_callback_t)(float angle);

/* ========== 数据结构体 ========== */

/**
 * @brief UART 命令接收器
 */
typedef struct {
    uint8_t uart_num;                    ///< UART 端口号
    char rx_buffer[UART_CMD_BUF_SIZE];  ///< 接收缓冲区
    uint16_t rx_index;                   ///< 缓冲区索引
    uart_angle_callback_t on_angle_cmd;  ///< 角度命令回调
    float last_angle;                    ///< 上一次接收的角度
} uart_command_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 UART 命令接收
 * 
 * 默认使用 UART0，GPIO3(RX), GPIO1(TX)，波特率 115200
 * 
 * @param cmd_receiver UART 命令接收器指针
 * @param callback 角度命令回调函数
 * @return ESP_OK 成功，否则失败
 */
esp_err_t uart_command_init(uart_command_t *cmd_receiver, uart_angle_callback_t callback);

/**
 * @brief 启动 UART 命令接收任务
 * 
 * 创建 FreeRTOS 任务持续检查串口数据
 * 
 * @param cmd_receiver UART 命令接收器指针
 * @param priority 任务优先级
 * @param stack_size 堆栈大小 (字节)
 * @return ESP_OK 成功，否则失败
 */
esp_err_t uart_command_start(uart_command_t *cmd_receiver, uint32_t priority, uint32_t stack_size);

/**
 * @brief 手动处理接收到的一行数据
 * 
 * 用于测试或外部调用
 * 
 * @param cmd_receiver UART 命令接收器指针
 * @param line 接收到的一行数据（以 \0 结尾）
 * @return ESP_OK 成功解析有效命令，ESP_ERR_INVALID_ARG 格式错误
 */
esp_err_t uart_command_process_line(uart_command_t *cmd_receiver, const char *line);

/**
 * @brief 获取上一次接收的有效角度
 * 
 * @param cmd_receiver UART 命令接收器指针
 * @return 上一次接收的角度值 (0-180°)，未接收过返回 90°
 */
float uart_command_get_last_angle(uart_command_t *cmd_receiver);

/**
 * @brief 关闭 UART 命令接收
 * 
 * @param cmd_receiver UART 命令接收器指针
 * @return ESP_OK 成功，否则失败
 */
esp_err_t uart_command_deinit(uart_command_t *cmd_receiver);

#ifdef __cplusplus
}
#endif

#endif // UART_COMMAND_H
