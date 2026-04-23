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

// 键盘控制的全局目标推力（前进为正，-100 到 100）
static float g_forward_thrust = 0.0f;

// IMU 专用测试任务 (100Hz)
void imu_test_task(void *pvParameters) {
    dual_imu_data_t imu_data;
    balance_state_t state;
    
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz

    ESP_LOGI(TAG, "========== IMU 测试模式启动 ==========");
    ESP_LOGI(TAG, "输出频率: 10Hz (每100ms一条)\n");
    
    int print_count = 0;
    while (1) {
        if (imu_driver_read(&imu_data) == ESP_OK) {
            // 融合得到四元数和欧拉角
            balance_controller_update(&imu_data, &state);
            
            // 每 10 帧打一条（10Hz）
            if (++print_count >= 10) {
                printf("\n--- IMU 数据输出 (100Hz读取, 10Hz显示) ---\n");
                
                #if USE_DUAL_IMU
                printf("[IMU1] Ax:%.3f Ay:%.3f Az:%.3f | Gx:%.3f Gy:%.3f Gz:%.3f\n",
                       imu_data.imu1.accel_x, imu_data.imu1.accel_y, imu_data.imu1.accel_z,
                       imu_data.imu1.gyro_x, imu_data.imu1.gyro_y, imu_data.imu1.gyro_z);
                printf("[IMU2] Ax:%.3f Ay:%.3f Az:%.3f | Gx:%.3f Gy:%.3f Gz:%.3f\n",
                       imu_data.imu2.accel_x, imu_data.imu2.accel_y, imu_data.imu2.accel_z,
                       imu_data.imu2.gyro_x, imu_data.imu2.gyro_y, imu_data.imu2.gyro_z);
                #else
                printf("[IMU] Ax:%.3f Ay:%.3f Az:%.3f | Gx:%.3f Gy:%.3f Gz:%.3f\n",
                       imu_data.imu1.accel_x, imu_data.imu1.accel_y, imu_data.imu1.accel_z,
                       imu_data.imu1.gyro_x, imu_data.imu1.gyro_y, imu_data.imu1.gyro_z);
                #endif
                
                printf("[融合] Roll:%.2f° Pitch:%.2f° Yaw:%.2f° | Tau:%.2f\n",
                       state.roll_deg, state.pitch_deg, state.yaw_deg, state.tau_total);
                printf("-----------------------------------\n");
                
                print_count = 0;
            }
        } else {
            static int err_count = 0;
            if (++err_count >= 100) {
                ESP_LOGW(TAG, "⚠️  IMU 读取失败 (连续失败次数: %d)", err_count);
                ESP_LOGW(TAG, "请检查: 1) I2C 线路连接  2) IMU 电源  3) 地址配置");
                err_count = 0;
            }
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 测试专用目标角度
static float g_test_steer_angle = 180.0f;

// 转向机构只转测试任务 (100Hz)
void steering_test_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    ESP_LOGI(TAG, "========== 独立转向机构测试模式 ==========");
    ESP_LOGI(TAG, "串口输入数字直接设置舵机角度(90~270度)");

    while(1) {
        // 大电机强制安全停转
        motor_control_emergency_stop();
        
        // 左右电机同步接受输入的测试角度
        steering_control_set_target(g_test_steer_angle, g_test_steer_angle);
        steering_control_update();

        // 打印当前闭环数据
        float act_L, act_R;
        steering_control_get_current_angles(&act_L, &act_R);
        
        static int print_cnt = 0;
        if (++print_cnt >= 20) { // 5Hz
            printf("[转向测试] Target: %.1f° | 实际L: %.1f°, 实际R: %.1f°\n", g_test_steer_angle, act_L, act_R);
            print_cnt = 0;
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 强实时大本营控制任务 (100Hz)
void control_core_task(void *pvParameters) {
    dual_imu_data_t imu_data;
    balance_state_t state;
    
    // FreeRTOS 的高精度绝对延时器
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 10ms = 100Hz

    // 低通滤波平滑目标角度 (避免舵机高频抖动)
    static float filter_target_L = 180.0f;
    static float filter_target_R = 180.0f;

    while (1) {
        // 获取当前的物理真实角度
        float cur_steer_left, cur_steer_right;
        steering_control_get_current_angles(&cur_steer_left, &cur_steer_right);

        // --- 姿态平衡环与推力矢量解算 ---
        if (imu_driver_read(&imu_data) == ESP_OK) {
            
            // 算出需要抵抗倾覆的垂直力矩 tau_total
            balance_controller_update(&imu_data, &state);

            // 紧急防翻车保护（超过设定的安全角度立刻停推）
            if (ENABLE_EMERGENCY_STOP && fabsf(state.roll_deg) > 60.0f) {
                motor_control_emergency_stop();
                steering_control_set_target(180.0f, 180.0f); // 翻车后舵机回正向下
                steering_control_update(); 
            } else {
                // ====== 核心：推力矢量融合解算 ======
                // 1. 获取目标前进推力幅值 (映射到 0~500 范围)
                float H_thrust = (g_forward_thrust / 100.0f) * 500.0f;
                // 或者说推力最大值就是 500，这是 PWM 从 1500 -> 2000 的最大增加量
                
                // 2. 获取目标垂直平衡推力 (假设 state.tau_total>0 时左倾，左边需要往下抗)
                // 现有的 THRUST_SCALE 转换系数 
                #define THRUST_SCALE 0.55f 
                float V_diff = state.tau_total * THRUST_SCALE; 
                
                // 将垂直推力限制在非负区间 (电机只往外喷水)
                // 正值：左边出力； 负值：右边出力
                float V_L = V_diff > 0.0f ? V_diff : 0.0f;
                float V_R = V_diff < 0.0f ? -V_diff : 0.0f;
                
                // 3. --- 目标角度解算 ---
                // atan2f(Horizontal, Vertical) 算出偏离垂直向下的角度
                float theta_L_rad = atan2f(H_thrust, V_L);
                float theta_R_rad = atan2f(H_thrust, V_R);
                
                float theta_L_deg = theta_L_rad * 180.0f / M_PI;
                float theta_R_deg = theta_R_rad * 180.0f / M_PI;
                
                // 映射到左右舵机的物理角度 (左：180为向下，90为朝后；右：180为向下，270为朝后)
                float target_angle_L = 180.0f - theta_L_deg; 
                float target_angle_R = 180.0f + theta_R_deg; 
                
                // 4. --- 舵机指令平滑滤波 ---
                filter_target_L = filter_target_L * 0.9f + target_angle_L * 0.1f;
                filter_target_R = filter_target_R * 0.9f + target_angle_R * 0.1f;
                
                steering_control_set_target(filter_target_L, filter_target_R);
                
                // 5. --- 目标标称推力幅值 ---
                float T_L_target = sqrtf(V_L*V_L + H_thrust*H_thrust);
                float T_R_target = sqrtf(V_R*V_R + H_thrust*H_thrust);
                
                // 6. --- 物理真实闭环：动态推力补偿 ---
                // 获取当前喷嘴实际的倾斜角（偏离垂直 180 度的角）
                float act_theta_L_deg = 180.0f - cur_steer_left; // 左边: 180为0, 90时为90
                float act_theta_R_deg = cur_steer_right - 180.0f; // 右边: 180为0, 270时为90
                
                float cos_L = cosf(act_theta_L_deg * M_PI / 180.0f);
                if (cos_L < 0.05f) cos_L = 0.05f; // 避免除零
                float T_L_safe = V_L / cos_L;

                float cos_R = cosf(act_theta_R_deg * M_PI / 180.0f);
                if (cos_R < 0.05f) cos_R = 0.05f;
                float T_R_safe = V_R / cos_R;
                
                // 最终推力取【物理延时补偿需求】和【目标矢量推力需求】的最大值，确保平衡垂直力只多不少
                float T_L_final = fmaxf(T_L_target, T_L_safe);
                float T_R_final = fmaxf(T_R_target, T_R_safe);
                
                // 下发至大电机
                motor_control_set_pwm_vector(T_L_final, T_R_final);
            }
        } else {
            // I2C 读取失败保护
            state.roll_deg = 0.0f;
            state.tau_total = 0.0f;
            motor_control_emergency_stop();
        }

        // 统一更新小电机位置 (下发滤波后的 PWM)
        steering_control_update(); 

        // 串口实时数据监测 (每 10 帧打一条，10Hz)
        static int print_cnt = 0;
        if (++print_cnt >= 10) { 
            printf("Fwd:%.1f%% | TgtL:%.1f (Act:%.1f) | TgtR:%.1f (Act:%.1f) | Roll:%.2f\n", 
                   g_forward_thrust, filter_target_L, cur_steer_left, filter_target_R, cur_steer_right, state.roll_deg);
            print_cnt = 0;
        }

        // 绝对延时：确保本次循环精准踩在 10 毫秒节点
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
    
    // 等待编码器稳定后进行校准（竖直状态下将编码器值设为0点基准）
    vTaskDelay(pdMS_TO_TICKS(100));
    steering_control_calibrate_encoders();
    
    ESP_LOGI(TAG, "SteadySail 就绪，当前模式: %d", CURRENT_RUN_MODE);

    // ************ 如果是 IMU 专用测试模式 ************
#if CURRENT_RUN_MODE == MODE_TEST_IMU_ONLY
    ESP_LOGI(TAG, "🎯 IMU 测试模式 - 实时显示传感器数据");
    xTaskCreatePinnedToCore(imu_test_task, "imu_test_task", 4096, NULL, 5, NULL, 1);
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }

    // ************ 如果是 专用的大电机 ESC 校准模式 ************
#elif CURRENT_RUN_MODE == MODE_CALIBRATE_ESC
    ESP_LOGW(TAG, "注意: 当前运行于大电机校准模式！");
    xTaskCreatePinnedToCore(motor_control_esc_calibrate_task, "esc_calibrate_task", 8192, NULL, 5, NULL, 1);
    // 校准模式下只运行ESC任务，永远不会到达下面的代码
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    
    // ************ 如果是 专用的小电机转向测试模式 ************
#elif CURRENT_RUN_MODE == MODE_TEST_STEERING_ONLY
    ESP_LOGW(TAG, "注意: 当前运行于独立转向机构测试模式！大电机禁用");
    xTaskCreatePinnedToCore(steering_test_task, "steering_test_task", 4096, NULL, 5, NULL, 1);
    
    // 串口接收角度数据
    char rx_buf[64] = {0};
    int rx_len = 0;
    while (1) {
        int ch = getchar();
        if (ch != EOF) {
            if (ch == '\r' || ch == '\n') {
                if (rx_len > 0) {
                    rx_buf[rx_len] = '\0';
                    char *endptr = NULL;
                    float input_val = strtof(rx_buf, &endptr);
                    if (endptr != rx_buf) {
                        g_test_steer_angle = input_val;
                        // 这里可以根据舵机的物理极限做一个约束，一般是 90～270（180朝下）
                        if(g_test_steer_angle < 90.0f) g_test_steer_angle = 90.0f;
                        if(g_test_steer_angle > 270.0f) g_test_steer_angle = 270.0f;
                        printf("\n>>> 收到角度指令! 目标设为: %.1f ° <<<\n", g_test_steer_angle);
                    } else {
                        printf("\n>>> 无效输入: %s <<<\n", rx_buf);
                    }
                    rx_len = 0;
                }
            } else if (rx_len < sizeof(rx_buf) - 1) {
                rx_buf[rx_len++] = (char)ch;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
#else
    // ============ 非单边测试模式：正常运行平衡融合控制 ============
    
    // 2. 将核心控制抛入后台的强心 Task (固定在 Core 1 上跑 100Hz)
    xTaskCreatePinnedToCore(control_core_task, "control_core_task", 4096, NULL, 5, NULL, 1);

    // 3. 通信与监控任务 (可以使用串口直接输入前进百分比，或者发送 w、s 微调)
    char rx_buf[64] = {0};
    int rx_len = 0;
    while (1) {
        int ch = getchar();
        if (ch != EOF) {
            if (ch == '\r' || ch == '\n') {
                if (rx_len > 0) {
                    rx_buf[rx_len] = '\0';
                    
                    // 检查是否是微调快捷键
                    if (strcmp(rx_buf, "w") == 0 || strcmp(rx_buf, "W") == 0) {
                        g_forward_thrust += 10.0f;
                        if(g_forward_thrust > 100.0f) g_forward_thrust = 100.0f;
                        printf("\n>>> 前进推力 +10%%，当前目标: %.1f %% <<<\n", g_forward_thrust);
                    } 
                    else if (strcmp(rx_buf, "s") == 0 || strcmp(rx_buf, "S") == 0) {
                        g_forward_thrust -= 10.0f;
                        if(g_forward_thrust < -100.0f) g_forward_thrust = -100.0f;
                        printf("\n>>> 前进推力 -10%%，当前目标: %.1f %% <<<\n", g_forward_thrust);
                    } 
                    else if (strcmp(rx_buf, "space") == 0 || rx_buf[0] == ' ') {
                        g_forward_thrust = 0.0f;
                        printf("\n>>> 推力归零！原地自平衡！ <<<\n");
                    } 
                    // 解析具体数字
                    else {
                        char *endptr = NULL;
                        float input_val = strtof(rx_buf, &endptr);
                        if (endptr != rx_buf) { 
                            g_forward_thrust = input_val;
                            if(g_forward_thrust > 100.0f) g_forward_thrust = 100.0f;
                            if(g_forward_thrust < -100.0f) g_forward_thrust = -100.0f;
                            printf("\n>>> 收到绝对推力指令! 目标设为: %.1f %% <<<\n", g_forward_thrust);
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
#endif
}
