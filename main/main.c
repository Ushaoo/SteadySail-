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
#include "control_params.h"
#include "blinker_bridge.h"

static const char *TAG = "MAIN";

// 键盘控制的全局目标推力（前进为正，-100 到 100）——跨文件可访问
volatile float g_forward_thrust = 0.0f;

// 供 Blinker 上报使用：由 control_core_task 每 10ms 刷新
volatile float g_last_roll_deg = 0.0f;

// 急停标志：true = 强制停止控制循环（电机停 / 舵机回正），false = 正常运行
volatile bool g_estop_active = false;

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

// 演示模式：手动模拟 Roll 角（度）。仅在 MODE_FULL_INTEGRATION + DEMO_MANUAL_ROLL 下使用。
volatile float g_demo_roll_deg = 0.0f;

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

        // 读取最近一次下发的舵机 PWM (1000~2000 us)
        uint32_t pwm_L = 1500, pwm_R = 1500;
        motor_control_get_last_steer_pwm(&pwm_L, &pwm_R);

        static int print_cnt = 0;
        if (++print_cnt >= 5) { // 20Hz
            ESP_LOGI(TAG, "[转向测试] Target: %.1f° | 实际L: %.1f°, 实际R: %.1f° | PWM_L: %lu us, PWM_R: %lu us",
                   g_test_steer_angle, act_L, act_R,
                   (unsigned long)pwm_L, (unsigned long)pwm_R);
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
    
    // 保存最后的平衡参数用于显示
    static float last_tau_total = 0.0f;
    static float last_V_balance = 0.0f;

    while (1) {
        // ====== 急停检查（最高优先级，遥控触发） ======
        if (g_estop_active) {
            // 仍然刷新 roll 显示，方便 App 端观察姿态
            if (imu_driver_read(&imu_data) == ESP_OK) {
                balance_controller_update(&imu_data, &state);
                g_last_roll_deg = state.roll_deg;
            }
            motor_control_emergency_stop();
            steering_control_set_target(180.0f, 180.0f);
            steering_control_update();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // 获取当前的物理真实角度
        float cur_steer_left, cur_steer_right;
        steering_control_get_current_angles(&cur_steer_left, &cur_steer_right);

        // --- 姿态平衡环与推力矢量解算 ---
        bool imu_ok = false;

#if (CURRENT_RUN_MODE == MODE_FULL_INTEGRATION) && DEMO_MANUAL_ROLL
        // ====== 演示模式：手动模拟 Roll，忽略 IMU ======
        // 直接用静态平衡近似公式合成 tau_total（omega=0, alpha=0, integral=0）：
        //   tau_disturb = m*g*(W/2)*sin(theta)
        //   tau_ff      = -tau_disturb + K_SELF*theta        (来自 balance_controller 内 tau_self = -K_SELF*theta)
        //   tau_pid     = g_balance_kp * (-theta)            (P 项；I/D 在静态下为 0)
        //   tau_total   = FEEDFORWARD_PARAM*tau_ff - FEEDBACK_PARAM*tau_pid
        // 之后再施加与 balance_controller 一致的角度死区平滑。
        {
            float theta = g_demo_roll_deg;
            float K_SELF_DEMO = 100.0f;  // 与 balance_controller.c 中 K_SELF 一致
            float tau_disturb = SYS_MASS * GRAVITY * (SYS_WIDTH / 2.0f)
                                * sinf(theta * (float)M_PI / 180.0f);
            float tau_ff  = -tau_disturb + K_SELF_DEMO * theta;
            float tau_pid =  g_balance_kp * (-theta);
            float tau_total = FEEDFORWARD_PARAM * tau_ff - FEEDBACK_PARAM * tau_pid;

            // 角度死区（与 balance_controller 同步）
            float abs_theta = fabsf(theta);
            float angle_factor;
            if (abs_theta < ANGLE_DEADZONE) {
                angle_factor = 0.0f;
            } else if (abs_theta < ANGLE_DEADZONE_SOFT) {
                float t = (abs_theta - ANGLE_DEADZONE) / (ANGLE_DEADZONE_SOFT - ANGLE_DEADZONE);
                angle_factor = t * t * (3.0f - 2.0f * t);
            } else {
                angle_factor = 1.0f;
            }
            tau_total *= angle_factor;

            state.roll_deg     = theta;
            state.pitch_deg    = 0.0f;
            state.yaw_deg      = 0.0f;
            state.omega_filtered = 0.0f;
            state.alpha        = 0.0f;
            state.tau_ff       = tau_ff;
            state.tau_pid      = tau_pid;
            state.tau_total    = tau_total;
            imu_ok = true;
        }
#else
        if (imu_driver_read(&imu_data) == ESP_OK) {
            // 算出需要抵抗倾覆的垂直力矩 tau_total
            balance_controller_update(&imu_data, &state);
            imu_ok = true;
        }
#endif

        if (imu_ok) {

            // 紧急防翻车保护（超过设定的安全角度立刻停推）
            if (ENABLE_EMERGENCY_STOP && fabsf(state.roll_deg) > 60.0f) {
                g_last_roll_deg = state.roll_deg;
                motor_control_emergency_stop();
                steering_control_set_target(180.0f, 180.0f); // 翻车后舵机回正向下
                steering_control_update(); 
            } else {
                // ====== 核心：同向差值平衡方案 ======
                // 两侧推力的竖直分量方向相同（均向下），靠"幅值差"产生平衡力矩；
                // 两侧水平分量保持相等（都等于 H_thrust），从而前进而不偏航。
                // 弱侧 V=0 → 舵机角度更偏水平；强侧 V 大 → 舵机角度更接近竖直。

                // 1. 目标前进推力（水平分量），两侧相等
                float H_thrust = (g_forward_thrust / 100.0f) * 500.0f;

                // 2. 平衡所需的竖直差值（带符号）
                #define THRUST_SCALE 0.55f
                float dV = state.tau_total * THRUST_SCALE;  // 带符号
                g_last_roll_deg = state.roll_deg;

                // 3. 分配两侧竖直分量（均 ≥ 0，方向相同——都朝下）
                //    约定：tau_total > 0 时需要左侧竖直推力更大（与原 invert 逻辑保持一致）
                float V_L = (dV > 0.0f) ?  dV : 0.0f;
                float V_R = (dV < 0.0f) ? -dV : 0.0f;

                // 4. 由 (V, H) 解算每侧的舵机偏角与推力幅值
                //    θ = atan2(H, V)，T = √(V² + H²)
                float theta_L_rad = atan2f(H_thrust, V_L);
                float theta_R_rad = atan2f(H_thrust, V_R);
                float theta_L_deg = theta_L_rad * 180.0f / (float)M_PI;
                float theta_R_deg = theta_R_rad * 180.0f / (float)M_PI;

                float T_L_target = sqrtf(V_L * V_L + H_thrust * H_thrust);
                float T_R_target = sqrtf(V_R * V_R + H_thrust * H_thrust);

                // 5. 映射到舵机物理角度（180° = 正向下；左右对称地向前倾）
                float target_angle_L = 180.0f - theta_L_deg;
                float target_angle_R = 180.0f + theta_R_deg;

                // 6. 舵机指令平滑滤波
                filter_target_L = filter_target_L * 0.9f + target_angle_L * 0.1f;
                filter_target_R = filter_target_R * 0.9f + target_angle_R * 0.1f;

                steering_control_set_target(filter_target_L, filter_target_R);

                // 7. 闭环补偿：根据实际舵机角度，保证竖直分量足够（不被衰减）
                float act_theta_L_deg = 180.0f - cur_steer_left;
                float act_theta_R_deg = cur_steer_right - 180.0f;
                float cos_L = cosf(act_theta_L_deg * (float)M_PI / 180.0f);
                float cos_R = cosf(act_theta_R_deg * (float)M_PI / 180.0f);
                if (cos_L < 0.05f) cos_L = 0.05f;
                if (cos_R < 0.05f) cos_R = 0.05f;

                float T_L_safe = V_L / cos_L;
                float T_R_safe = V_R / cos_R;
                float T_L_final = fmaxf(T_L_target, T_L_safe);
                float T_R_final = fmaxf(T_R_target, T_R_safe);

                // 保存显示用
                last_tau_total = state.tau_total;
                last_V_balance = fabsf(dV);

                // 8. 下发推力指令——两侧"同向"（均不反转），幅度差产生平衡力矩
                motor_control_set_pwm_bidirectional(T_L_final, T_R_final, false, false);
            }
        } else {
            // I2C 读取失败保护
            state.roll_deg = 0.0f;
            state.tau_total = 0.0f;
            motor_control_emergency_stop();
        }

        // ====== 编码器故障检测与处理 ======
        static bool enc_left_fault = false, enc_right_fault = false;
        static uint32_t enc_fault_warn_time = 0;
        bool enc_left_ok, enc_right_ok;
        steering_control_get_encoder_status(&enc_left_ok, &enc_right_ok);
        
        // 检测故障状态变化
        if (!enc_left_ok && !enc_left_fault) {
            enc_left_fault = true;
            ESP_LOGE(TAG, "🚨 LEFT ENCODER DISCONNECTED! Locking left servo position.");
            enc_fault_warn_time = xTaskGetTickCount();
        }
        if (!enc_right_ok && !enc_right_fault) {
            enc_right_fault = true;
            ESP_LOGE(TAG, "🚨 RIGHT ENCODER DISCONNECTED! Locking right servo position.");
            enc_fault_warn_time = xTaskGetTickCount();
        }
        
        // 恢复故障状态
        if (enc_left_ok && enc_left_fault) {
            enc_left_fault = false;
            ESP_LOGI(TAG, "✓ LEFT ENCODER RECOVERED!");
        }
        if (enc_right_ok && enc_right_fault) {
            enc_right_fault = false;
            ESP_LOGI(TAG, "✓ RIGHT ENCODER RECOVERED!");
        }
        
        // 定期提示故障状态
        if ((enc_left_fault || enc_right_fault) && xTaskGetTickCount() - enc_fault_warn_time > pdMS_TO_TICKS(5000)) {
            ESP_LOGW(TAG, "Encoder status - Left:%s Right:%s", 
                     enc_left_fault ? "FAULT" : "OK", enc_right_fault ? "FAULT" : "OK");
            enc_fault_warn_time = xTaskGetTickCount();
        }

        // 统一更新小电机位置 (下发滤波后的 PWM)
        steering_control_update(); 

        // 串口实时数据监测 (每 10 帧打一条，10Hz)
        static int print_cnt = 0;
        if (++print_cnt >= 10) { 
            // 获取实际下发的 PWM 脉宽
            uint32_t actual_pwm_L, actual_pwm_R;
            motor_control_get_last_pwm(&actual_pwm_L, &actual_pwm_R);
            
            printf("Fwd:%.1f%% | TgtL:%.1f (Act:%.1f) | TgtR:%.1f (Act:%.1f) | Roll:%.2f | PWM_L:%u PWM_R:%u | Tau:%.0f dV:%.0f | Enc:%s/%s\n", 
                   g_forward_thrust, filter_target_L, cur_steer_left, filter_target_R, cur_steer_right, 
                   state.roll_deg, actual_pwm_L, actual_pwm_R, last_tau_total, last_V_balance,
                   enc_left_fault ? "X" : "✓", enc_right_fault ? "X" : "✓");
            print_cnt = 0;
        }

        // 绝对延时：确保本次循环精准踩在 10 毫秒节点
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "SteadySail Version 2 - Dual IMU + Vector Thrust");

    // 0. 初始化全局可调参数（含 NVS 加载运行模式）
    control_params_init();

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

    // ************ 编译期按 CURRENT_RUN_MODE 分发任务 ************
#if CURRENT_RUN_MODE == MODE_TEST_IMU_ONLY
    ESP_LOGI(TAG, "🎯 IMU 测试模式 - 实时显示传感器数据");
    xTaskCreatePinnedToCore(imu_test_task, "imu_test_task", 4096, NULL, 5, NULL, 1);
    blinker_bridge_start();
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }

#elif CURRENT_RUN_MODE == MODE_CALIBRATE_ESC
    ESP_LOGW(TAG, "注意: 当前运行于大电机校准模式！");
    xTaskCreatePinnedToCore(motor_control_esc_calibrate_task, "esc_calibrate_task", 8192, NULL, 5, NULL, 1);
    blinker_bridge_start();
    while(1) { vTaskDelay(pdMS_TO_TICKS(1000)); }

#elif CURRENT_RUN_MODE == MODE_TEST_STEERING_ONLY
    ESP_LOGW(TAG, "注意: 当前运行于独立转向机构测试模式！大电机禁用");
    xTaskCreatePinnedToCore(steering_test_task, "steering_test_task", 4096, NULL, 5, NULL, 1);
    blinker_bridge_start();

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
    // ============ 其他模式（含 FULL_INTEGRATION / TEST_SENSORS / TEST_BALANCE_ONLY）：运行平衡融合控制 ============
    xTaskCreatePinnedToCore(control_core_task, "control_core_task", 4096, NULL, 5, NULL, 1);
    blinker_bridge_start();

    // 通信与监控任务 (串口输入前进推力)
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
                // 演示模式：r <角度>  设置模拟 Roll
                else if ((rx_buf[0] == 'r' || rx_buf[0] == 'R') &&
                         (rx_buf[1] == ' ' || rx_buf[1] == '=' || rx_buf[1] == ':')) {
#if (CURRENT_RUN_MODE == MODE_FULL_INTEGRATION) && DEMO_MANUAL_ROLL
                    char *endptr = NULL;
                    float roll_val = strtof(rx_buf + 2, &endptr);
                    if (endptr != rx_buf + 2) {
                        if (roll_val >  60.0f) roll_val =  60.0f;
                        if (roll_val < -60.0f) roll_val = -60.0f;
                        g_demo_roll_deg = roll_val;
                        printf("\n>>> [DEMO] 模拟 Roll 设为 %.2f° <<<\n", roll_val);
                    } else {
                        printf("\n>>> 无效 r 命令: %s （用法: r 15 或 r -20） <<<\n", rx_buf);
                    }
#else
                    printf("\n>>> r 命令仅在 MODE_FULL_INTEGRATION + DEMO_MANUAL_ROLL=1 下生效 <<<\n");
#endif
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
