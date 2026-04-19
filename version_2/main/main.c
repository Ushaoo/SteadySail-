#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "system_config.h"
#include <math.h>
#include <string.h>

// 引入我们的核心控制模块
#include "imu_driver.h"
#include "balance_controller.h"
#include "motor_control.h"
#include "steering_control.h"

static const char *TAG = "MAIN";

// 键盘控制的全局目标转向角度
static float g_steering_angle_deg = 0.0f;

// 强实时大本营控制任务 (100Hz)
void control_core_task(void *pvParameters) {
    dual_imu_data_t imu_data;
    balance_state_t state;
    
    // FreeRTOS 的高精度绝对延时器
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz

    while (1) {
        // --- 1. 更新小电机转向 PID 环 ---
        // 任何情况下更新 steering target并进行短路径算计
        steering_control_set_target(g_steering_angle_deg, g_steering_angle_deg);
        steering_control_update(); 

        // 提取目前的真实物理偏角用于主推进推力补偿
        float cur_steer_left, cur_steer_right;
        steering_control_get_current_angles(&cur_steer_left, &cur_steer_right);
        float actual_thrust_angle = (cur_steer_left + cur_steer_right) / 2.0f;

        // --- 2. 姿态平衡环 ---
        if (imu_driver_read(&imu_data) == ESP_OK) {
            
            // 将原始数据喂给算法，算出需要抵抗倾覆的力 tau_total
            balance_controller_update(&imu_data, &state);

            // 紧急防翻车保护（超过设定的安全角度立刻停推）
            if (ENABLE_EMERGENCY_STOP && fabsf(state.roll_deg) > 60.0f) {
                motor_control_emergency_stop();
            } else {
                // 3. 终极联动：将理论平衡力 tau_total 除以当前转角余弦 actual_thrust_angle，输出信号
                motor_control_set_thrust_with_compensation(state.tau_total, actual_thrust_angle);
            }
        } else {
            // 屏蔽刷屏：I2C 读取失败，由于没有IMU数据，停止推力分配，重置姿态状态变量（为了避免打印乱码）
            state.roll_deg = 0.0f;
            state.tau_total = 0.0f;
            motor_control_emergency_stop();
        }

        // 串口实时数据监测 (每 10 帧打一条，10Hz)
        static int print_cnt = 0;
        if (++print_cnt >= 10) { 
            printf("Target:%.1f | CurL:%.1f | CurR:%.1f | Roll:%.2f | Tau:%.2f\n", 
                   g_steering_angle_deg, cur_steer_left, cur_steer_right, state.roll_deg, state.tau_total);
            print_cnt = 0;
        }

        // 绝对延时：确保本次循环精准踩在 10 毫米节点
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SteadySail Version 2 - Dual IMU + Vector Thrust");
    
    // 1. 初始化所有设备
    if (imu_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU 硬件异常！确保连线正确！(或当前是在无传感器测试)");
    }
    balance_controller_init();
    motor_control_init();
    steering_control_init(); // 开启外部中断读取编码器
    
    ESP_LOGI(TAG, "SteadySail 就绪，当前模式: %d", CURRENT_RUN_MODE);

    // ************ 如果是 专用的大电机 ESC 校准模式 ************
#if CURRENT_RUN_MODE == MODE_CALIBRATE_ESC
    xTaskCreatePinnedToCore(motor_control_esc_calibrate_task, "esc_calibrate_task", 4096, NULL, 5, NULL, 1);
    ESP_LOGW(TAG, "注意: 当前运行于大电机校准模式！已挂起平衡与转向计算...");
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
#endif

    // 2. 将核心控制抛入后台的强心 Task (固定在 Core 1 上跑 100Hz)
    xTaskCreatePinnedToCore(control_core_task, "control_core_task", 4096, NULL, 5, NULL, 1);

    // 3. 通信与监控任务 (可以使用串口直接输入绝对角度数字，或者发送 a、d 微调)
    char rx_buf[64] = {0};
    int rx_len = 0;
    while (1) {
        int ch = getchar();
        if (ch != EOF) {
            if (ch == '\r' || ch == '\n') {
                if (rx_len > 0) {
                    rx_buf[rx_len] = '\0';
                    
                    // 检查是否是微调快捷键
                    if (strcmp(rx_buf, "a") == 0 || strcmp(rx_buf, "A") == 0) {
                        g_steering_angle_deg += 15.0f;
                        printf("\n>>> 偏角 +15，目标: %.1f 度 <<<\n", g_steering_angle_deg);
                    } 
                    else if (strcmp(rx_buf, "d") == 0 || strcmp(rx_buf, "D") == 0) {
                        g_steering_angle_deg -= 15.0f;
                        printf("\n>>> 偏角 -15，目标: %.1f 度 <<<\n", g_steering_angle_deg);
                    } 
                    else if (strcmp(rx_buf, "s") == 0 || strcmp(rx_buf, "S") == 0) {
                        g_steering_angle_deg = 0.0f;
                        printf("\n>>> 转向回正！ <<<\n");
                    } 
                    // 如果不是快捷键，尝试解析为绝对角度数字
                    else {
                        char *endptr = NULL;
                        float input_angle = strtof(rx_buf, &endptr);
                        if (endptr != rx_buf) { 
                            // 成功解析到数字
                            g_steering_angle_deg = input_angle;
                            printf("\n>>> 收到绝对角度指令! 转向目标定为: %.1f 度 <<<\n", g_steering_angle_deg);
                        } else {
                            printf("\n>>> 无效输入: %s <<<\n", rx_buf);
                        }
                    }
                    rx_len = 0;
                }
            } else if (rx_len < sizeof(rx_buf) - 1) {
                rx_buf[rx_len++] = (char)ch;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // 提高串口响应速度
    }
}