#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
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

    // Scheme A 舵机就位率门控历史（提到外层，便于特殊路径下重置）
    static float last_thrust_motor_L = 0.0f;
    static float last_thrust_motor_R = 0.0f;

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
                // ====== 核心：相反竖直分量 + 相同水平分量平衡方案（不反转电机） ======
                // 利用舵机能旋转过 180° 的能力：一侧桨翻到上半圆即可"正转却向下推"。
                //   - 两侧水平分量都 = H_thrust  → 直行不偏航
                //   - 两侧竖直分量等大反向 (+dV / -dV) → 产生纠偏力矩
                //   - 两侧推力幅值相等  → 同时无电机反转
                //
                // 角度约定（θ：舵机物理角，0~360°，180°=正下）
                //   右桨：θ_R = 180° − atan2(H, V_R) × 180/π
                //   左桨：θ_L = 180° + atan2(H, V_L) × 180/π
                //   推力：T = √(V² + H²)

                // 1. 目标前进推力（水平分量），两侧相等
                float H_thrust = (g_forward_thrust / 100.0f) * 500.0f;

                // 2. 平衡所需的竖直分量（带符号）
                #define THRUST_SCALE 0.55f
                float dV = state.tau_total * THRUST_SCALE;  // 带符号
                g_last_roll_deg = state.roll_deg;

                // 3. 两侧竖直分量等大反向
                //    约定：tau_total > 0 → 右侧向上、左侧向下
                float V_R =  dV;   // 向上为正
                float V_L = -dV;

                // ===== 特殊路径：fwd ≈ 0 时，舵机锁 180°，电机正反转产生上下力 =====
                // 触发阈值：|H_thrust| < H_DEADZONE（默认对应 1% 推力）
                // 物理映射（受 L/R 互换影响）：
                //   原 V_R = +dV → 正常 V3 中作用于物理左电机；这里直接令物理左电机推力 = |dV|，
                //     正反转方向 = sign(V_R) = sign(dV)
                //   原 V_L = -dV → 作用于物理右电机；推力 = |dV|，方向 = sign(V_L) = -sign(dV)
                const float H_DEADZONE = 5.0f;
                if (fabsf(H_thrust) < H_DEADZONE) {
                    // 1) 舵机强制中立位 180°（同步把滤波器拉回，避免下次进入 V3 路径时残留）
                    filter_target_L = 180.0f;
                    filter_target_R = 180.0f;
                    steering_control_set_target(180.0f, 180.0f);

                    // 2) 电机正反转产生上下力差
                    float mag = fabsf(dV);
                    bool invert_phys_L = (dV < 0.0f);   // 物理左电机方向
                    bool invert_phys_R = (dV > 0.0f);   // 物理右电机方向
                    motor_control_set_pwm_bidirectional(mag, mag, invert_phys_L, invert_phys_R);

                    // 3) 重置 Scheme A 历史，避免下次重新进入 V3 路径时 last_thrust 残留导致跳变
                    last_thrust_motor_L = 0.0f;
                    last_thrust_motor_R = 0.0f;

                    last_tau_total = state.tau_total;
                    last_V_balance = fabsf(dV);
                    goto control_loop_tail;  // 跳过下面的 V3 + Scheme A 路径
                }

                // 4. 由 (V, H) 解算舵机偏角与推力幅值
                //    atan2(H, V) ∈ (-π, π]，可直接覆盖 0~360° 全部映射
                //    特别注意：dV=0 且 H=0 时 V_L = -0.0f，atan2(0,-0) = π，会让左舵机跑到 0°；
                //    所以这里显式处理"零矢量"情形，保持 180° 中立。
                const float ZERO_EPS = 1e-3f;
                float phi_R_deg, phi_L_deg;
                if (fabsf(V_R) < ZERO_EPS && fabsf(H_thrust) < ZERO_EPS) {
                    phi_R_deg = 0.0f;
                } else {
                    phi_R_deg = atan2f(H_thrust, V_R) * 180.0f / (float)M_PI;
                }
                if (fabsf(V_L) < ZERO_EPS && fabsf(H_thrust) < ZERO_EPS) {
                    phi_L_deg = 0.0f;
                } else {
                    phi_L_deg = atan2f(H_thrust, V_L) * 180.0f / (float)M_PI;
                }

                float T_R_target = sqrtf(V_R * V_R + H_thrust * H_thrust);
                float T_L_target = sqrtf(V_L * V_L + H_thrust * H_thrust);

                // 5. 映射到舵机物理角度（左右镜像，左 +、右 −）
                float target_angle_R = 180.0f - phi_R_deg;
                float target_angle_L = 180.0f + phi_L_deg;

                // 归一化到 [0, 360)
                while (target_angle_L < 0.0f)    target_angle_L += 360.0f;
                while (target_angle_L >= 360.0f) target_angle_L -= 360.0f;
                while (target_angle_R < 0.0f)    target_angle_R += 360.0f;
                while (target_angle_R >= 360.0f) target_angle_R -= 360.0f;

                // 6. 舵机指令平滑滤波（注意环形，避免穿越 0/360 边界产生反向跳变）
                float diff_L = target_angle_L - filter_target_L;
                while (diff_L > 180.0f)  diff_L -= 360.0f;
                while (diff_L < -180.0f) diff_L += 360.0f;
                filter_target_L += 0.1f * diff_L;
                while (filter_target_L < 0.0f)    filter_target_L += 360.0f;
                while (filter_target_L >= 360.0f) filter_target_L -= 360.0f;

                float diff_R = target_angle_R - filter_target_R;
                while (diff_R > 180.0f)  diff_R -= 360.0f;
                while (diff_R < -180.0f) diff_R += 360.0f;
                filter_target_R += 0.1f * diff_R;
                while (filter_target_R < 0.0f)    filter_target_R += 360.0f;
                while (filter_target_R >= 360.0f) filter_target_R -= 360.0f;

                // ⚠ 实测：左右物理装配相对算法是镜像的，此处把 L/R 整组互换下发
                //    互换后角度公式语义也跟着反了，所以再绕 180° 镜像一次（360 - x）
                float send_tgt_L = 360.0f - filter_target_R;
                float send_tgt_R = 360.0f - filter_target_L;
                while (send_tgt_L < 0.0f)    send_tgt_L += 360.0f;
                while (send_tgt_L >= 360.0f) send_tgt_L -= 360.0f;
                while (send_tgt_R < 0.0f)    send_tgt_R += 360.0f;
                while (send_tgt_R >= 360.0f) send_tgt_R -= 360.0f;
                steering_control_set_target(send_tgt_L, send_tgt_R);

                // 7. 推力下发——两侧均不反转（电机一律 PWM > 1500），方向完全由舵机决定
                //
                // 【舵机就位率门控】方案 A：上升非对称限速，下降不门控
                //   r = max(0, 1 - |Δθ|/θ_tol)  ∈ [0,1]
                //   T_out = T_prev + r * (T_cmd - T_prev)   (仅当 T_cmd > T_prev)
                //   T_out = T_cmd                            (T_cmd <= T_prev 直通)
                //   左右独立计算。物理左舵机 ↔ 物理左电机（接收 T_R_target，因为 L/R 已互换）
                //   注：last_thrust_motor_L/R 已提到 control_core_task 函数顶部声明
                const float SERVO_TOL_DEG = 15.0f;

                // 物理左侧：目标角 send_tgt_L，实际 cur_steer_left；门控 T_R_target（送给物理左电机）
                float err_servo_L = send_tgt_L - cur_steer_left;
                while (err_servo_L >  180.0f) err_servo_L -= 360.0f;
                while (err_servo_L < -180.0f) err_servo_L += 360.0f;
                float r_L = 1.0f - fabsf(err_servo_L) / SERVO_TOL_DEG;
                if (r_L < 0.0f) r_L = 0.0f;
                if (r_L > 1.0f) r_L = 1.0f;

                float thrust_motor_L = T_R_target;  // L/R 已互换：物理左电机收 T_R_target
                if (thrust_motor_L > last_thrust_motor_L) {
                    thrust_motor_L = last_thrust_motor_L + r_L * (thrust_motor_L - last_thrust_motor_L);
                }
                last_thrust_motor_L = thrust_motor_L;

                // 物理右侧：目标角 send_tgt_R，实际 cur_steer_right；门控 T_L_target（送给物理右电机）
                float err_servo_R = send_tgt_R - cur_steer_right;
                while (err_servo_R >  180.0f) err_servo_R -= 360.0f;
                while (err_servo_R < -180.0f) err_servo_R += 360.0f;
                float r_R = 1.0f - fabsf(err_servo_R) / SERVO_TOL_DEG;
                if (r_R < 0.0f) r_R = 0.0f;
                if (r_R > 1.0f) r_R = 1.0f;

                float thrust_motor_R = T_L_target;  // L/R 已互换：物理右电机收 T_L_target
                if (thrust_motor_R > last_thrust_motor_R) {
                    thrust_motor_R = last_thrust_motor_R + r_R * (thrust_motor_R - last_thrust_motor_R);
                }
                last_thrust_motor_R = thrust_motor_R;

                motor_control_set_pwm_bidirectional(thrust_motor_L, thrust_motor_R, false, false);

                // 保存显示用
                last_tau_total = state.tau_total;
                last_V_balance = fabsf(dV);

            control_loop_tail: ;  // 特殊路径（fwd≈0，纯反转模式）跳到这里
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
            
            // 转换到与编码器一致的"物理角"坐标显示，方便对比
            // (steering_control_set_target 内部对两侧都做了 360 - x 翻转)
            float disp_tgt_L = 360.0f - filter_target_L;
            float disp_tgt_R = 360.0f - filter_target_R;
            while (disp_tgt_L < 0.0f)    disp_tgt_L += 360.0f;
            while (disp_tgt_L >= 360.0f) disp_tgt_L -= 360.0f;
            while (disp_tgt_R < 0.0f)    disp_tgt_R += 360.0f;
            while (disp_tgt_R >= 360.0f) disp_tgt_R -= 360.0f;

            printf("Fwd:%.1f%% | TgtL:%.1f (Act:%.1f) | TgtR:%.1f (Act:%.1f) | Roll:%.2f | PWM_L:%u PWM_R:%u | Tau:%.0f dV:%.0f | Enc:%s/%s\n", 
                   g_forward_thrust, disp_tgt_L, cur_steer_left, disp_tgt_R, cur_steer_right, 
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
    steering_control_init(); // 开启外部中断读取编码器；内部会尝试从 NVS 加载历史校准

    // 注意：不再每次启动都自动校准。
    //  - 若 NVS 中已有保存的零点偏移，steering_control_init() 会自动加载。
    //  - 若 NVS 中没有数据（首次烧录后），舵机被锁定在 PWM=1500 中立位；
    //    用户把舵机摆正下方后，串口输入 'cal' 即可完成首次校准并写入 NVS。
    vTaskDelay(pdMS_TO_TICKS(100));

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
                    if (strcmp(rx_buf, "cal") == 0 || strcmp(rx_buf, "CAL") == 0) {
                        printf("\n>>> [CAL] 开始校准舵机零点（请确认舵机已摆正下方）...\n");
                        steering_control_calibrate_and_save();
                        printf(">>> [CAL] 校准完成，已写入 NVS。下次上电将自动加载。 <<<\n");
                        rx_len = 0;
                        continue;
                    }
                    if (strcmp(rx_buf, "reboot") == 0 || strcmp(rx_buf, "REBOOT") == 0) {
                        printf("\n>>> [REBOOT] 1 秒后重启 ESP32... <<<\n");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        esp_restart();
                    }
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
                // 校准舵机零点：把舵机摆到正下方（180° 竖直）后输入 'cal'
                else if (strcmp(rx_buf, "cal") == 0 || strcmp(rx_buf, "CAL") == 0) {
                    printf("\n>>> [CAL] 开始校准舵机零点（请确认舵机已摆正下方）...\n");
                    steering_control_calibrate_and_save();
                    printf(">>> [CAL] 校准完成，已写入 NVS。下次上电将自动加载。 <<<\n");
                }
                else if (strcmp(rx_buf, "reboot") == 0 || strcmp(rx_buf, "REBOOT") == 0) {
                    printf("\n>>> [REBOOT] 1 秒后重启 ESP32... <<<\n");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                }
                // 演示模式：r <角度>  设置模拟 Roll
                //   支持写法：r 15 / r=15 / r:15 / r15 / r-20
                else if ((rx_buf[0] == 'r' || rx_buf[0] == 'R') && rx_len >= 2) {
#if (CURRENT_RUN_MODE == MODE_FULL_INTEGRATION) && DEMO_MANUAL_ROLL
                    // 跳过 r 后可能存在的分隔符（空格 / = / :）
                    char *p = rx_buf + 1;
                    while (*p == ' ' || *p == '=' || *p == ':') p++;
                    char *endptr = NULL;
                    float roll_val = strtof(p, &endptr);
                    if (endptr != p) {
                        if (roll_val >  60.0f) roll_val =  60.0f;
                        if (roll_val < -60.0f) roll_val = -60.0f;
                        g_demo_roll_deg = roll_val;
                        printf("\n>>> [DEMO] 模拟 Roll 设为 %.2f° <<<\n", roll_val);
                    } else {
                        printf("\n>>> 无效 r 命令: %s （用法: r15 / r 15 / r-20） <<<\n", rx_buf);
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
