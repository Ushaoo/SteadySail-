#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
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
#include "bno055_driver.h"
#include "rc_input.h"
#include "driver/gpio.h"
#include "anchor_control.h"
#include "gps_nmea.h"
#include "fusion.h"

static const char *TAG = "MAIN";

// 键盘控制的全局目标推力（前进为正，-100 到 100）——跨文件可访问
volatile float g_forward_thrust = 0.0f;

// 差速转向状态：-1=左转，0=直行，+1=右转
// 由 Blinker 三按钮 tap 事件覆盖式设置（每次 tap 直接覆写）；
// 仅在 |g_forward_thrust| > TURN_MIN_FWD_PCT 时生效，否则被强制归零。
volatile int g_turn_state = 0;

// ====== 航向保持（BNO055 绝对偏航）======
// g_heading_hold_active = true  : 锁定航向，用 BNO055 偏航误差驱动差速 dH
// g_heading_hold_active = false : 正常手动差速（g_turn_state 决定方向）
// g_target_heading              : 锁定时的目标偏航角（0~360°，由 BNO055 当前值捕获）
volatile bool  g_heading_hold_active = false;
volatile float g_target_heading      = 0.0f;

// 供 Blinker 上报使用：由 control_core_task 每 10ms 刷新
volatile float g_last_roll_deg = 0.0f;

// 急停标志：true = 强制停止控制循环（电机停 / 舵机回正），false = 正常运行
volatile bool g_estop_active = false;

// 硬急停标志：true = 丢开所有控制逻辑，直接把 4 个通道都压到 1500us
//   与 g_estop_active 的区别：后者仍会跟 180° 舵机目标 (连续舵 1500us 才是"不转")。
volatile bool g_hard_estop = false;

// 磁控重启标志：磁铁重新吸合（上升沿）时由 ISR 置位，由 control_core_task 检测后重启
volatile bool g_mag_restart_pending = false;
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
#if USE_BNO055_FOR_ROLL
        // BNO055 模式：imu_test_task 直接通过 balance_controller 读取 BNO055 Roll 角
        balance_controller_update(&imu_data, &state);
        if (++print_count >= 10) {
            ESP_LOGI(TAG, "--- BNO055 融合数据 (100Hz读取, 10Hz显示) ---");
            ESP_LOGI(TAG, "Roll:%.2f° Pitch:%.2f° Yaw:%.2f° | Tau:%.2f",
                     state.roll_deg, state.pitch_deg, state.yaw_deg, state.tau_total);
            ESP_LOGI(TAG, "-----------------------------------");
            print_count = 0;
        }
#else
        if (imu_driver_read(&imu_data) == ESP_OK) {
            // 融合得到四元数和欧拉角
            balance_controller_update(&imu_data, &state);
            
            // 每 10 帧打一条（10Hz）
            if (++print_count >= 10) {
                ESP_LOGI(TAG, "--- IMU 数据输出 (100Hz读取, 10Hz显示) ---");
                
                #if USE_DUAL_IMU
                ESP_LOGI(TAG, "IMU1 | Ax:%.3f Ay:%.3f Az:%.3f | Gx:%.3f Gy:%.3f Gz:%.3f",
                         imu_data.imu1.accel_x, imu_data.imu1.accel_y, imu_data.imu1.accel_z,
                         imu_data.imu1.gyro_x, imu_data.imu1.gyro_y, imu_data.imu1.gyro_z);
                ESP_LOGI(TAG, "IMU2 | Ax:%.3f Ay:%.3f Az:%.3f | Gx:%.3f Gy:%.3f Gz:%.3f",
                         imu_data.imu2.accel_x, imu_data.imu2.accel_y, imu_data.imu2.accel_z,
                         imu_data.imu2.gyro_x, imu_data.imu2.gyro_y, imu_data.imu2.gyro_z);
                #else
                ESP_LOGI(TAG, "IMU | Ax:%.3f Ay:%.3f Az:%.3f | Gx:%.3f Gy:%.3f Gz:%.3f",
                         imu_data.imu1.accel_x, imu_data.imu1.accel_y, imu_data.imu1.accel_z,
                         imu_data.imu1.gyro_x, imu_data.imu1.gyro_y, imu_data.imu1.gyro_z);
                #endif
                
                ESP_LOGI(TAG, "融合 | Roll:%.2f° Pitch:%.2f° Yaw:%.2f° | Tau:%.2f",
                         state.roll_deg, state.pitch_deg, state.yaw_deg, state.tau_total);
                ESP_LOGI(TAG, "-----------------------------------");
                
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
#endif  // USE_BNO055_FOR_ROLL
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// 测试专用目标角度
static float g_test_steer_angle = 180.0f;

// 演示模式：手动模拟 Roll 角（度）。仅在 MODE_FULL_INTEGRATION + DEMO_MANUAL_ROLL 下使用。
volatile float g_demo_roll_deg = 0.0f;

// 转向机构只转测试任务 (100Hz)
//
// 【响应时间分析】
//   每次检测到 g_test_steer_angle 变化（=收到串口/Blinker 新指令），
//   重置 t0，逐帧打印 [t=NNms] err/pwm；当 |err_L|<TOL 且 |err_R|<TOL 持续
//   STEADY_FRAMES 帧后，打印 [ARRIVED]；超过 TIMEOUT_MS 还没到则打印 [TIMEOUT]。
//   到达后回到"静默"状态，不再每帧刷屏，直到下一次目标变化。
void steering_test_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    ESP_LOGI(TAG, "========== 独立转向机构测试模式 ==========");
    ESP_LOGI(TAG, "串口输入数字直接设置舵机角度(90~270度)");
    ESP_LOGI(TAG, "[响应分析] 每次目标变化会自动测量到达时间，无需额外操作");

    // ===== 响应时间统计变量 =====
    const float    ARRIVE_TOL_DEG = 2.0f;   // 误差进入此阈值视为到达
    const int      STEADY_FRAMES  = 5;      // 连续 5 帧 (50ms) 都在阈值内才算稳定
    const uint32_t TIMEOUT_MS     = 2000;   // 2s 还没到达就报超时

    float    last_target   = g_test_steer_angle;
    bool     measuring     = false;         // 是否在测量中（目标变化后未到达/未超时）
    int64_t  t0_us         = 0;             // 目标变化时刻 (esp_timer_get_time)
    int      steady_cnt_L  = 0, steady_cnt_R = 0;
    bool     arrived_L     = false, arrived_R = false;
    int64_t  arrive_us_L   = 0, arrive_us_R = 0;

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

        // ===== 响应时间统计逻辑 =====
        float cur_target = g_test_steer_angle;
        if (cur_target != last_target) {
            // 目标变化 -> 重置统计
            t0_us        = esp_timer_get_time();
            measuring    = true;
            steady_cnt_L = 0;
            steady_cnt_R = 0;
            arrived_L    = false;
            arrived_R    = false;
            arrive_us_L  = 0;
            arrive_us_R  = 0;
            ESP_LOGW(TAG, "[STEP] target %.1f° -> %.1f°  开始测量到达时间", last_target, cur_target);
            last_target = cur_target;
        }

        if (measuring) {
            int64_t now_us  = esp_timer_get_time();
            uint32_t elapsed_ms = (uint32_t)((now_us - t0_us) / 1000);

            // 误差用环形最短路径（避免 0/360 边界跨越）
            float err_L = act_L - cur_target;
            while (err_L >  180.0f) err_L -= 360.0f;
            while (err_L < -180.0f) err_L += 360.0f;
            float err_R = act_R - cur_target;
            while (err_R >  180.0f) err_R -= 360.0f;
            while (err_R < -180.0f) err_R += 360.0f;

            // 每帧打印过程数据 (100Hz)
            ESP_LOGI(TAG, "[t=%4u ms] L=%.1f° (err=%+.2f) R=%.1f° (err=%+.2f) pwm_L=%lu pwm_R=%lu",
                     (unsigned)elapsed_ms, act_L, err_L, act_R, err_R,
                     (unsigned long)pwm_L, (unsigned long)pwm_R);

            // 单侧到达判定
            if (!arrived_L) {
                if (fabsf(err_L) <= ARRIVE_TOL_DEG) {
                    if (++steady_cnt_L >= STEADY_FRAMES) {
                        // 取首帧进入阈值的时间点（回退 (STEADY_FRAMES-1)*10ms）
                        arrive_us_L = now_us - (int64_t)(STEADY_FRAMES - 1) * 10000;
                        arrived_L = true;
                        uint32_t t_L_ms = (uint32_t)((arrive_us_L - t0_us) / 1000);
                        ESP_LOGW(TAG, "[ARRIVED L] %u ms (容差 ±%.1f°)", (unsigned)t_L_ms, ARRIVE_TOL_DEG);
                    }
                } else {
                    steady_cnt_L = 0;
                }
            }
            if (!arrived_R) {
                if (fabsf(err_R) <= ARRIVE_TOL_DEG) {
                    if (++steady_cnt_R >= STEADY_FRAMES) {
                        arrive_us_R = now_us - (int64_t)(STEADY_FRAMES - 1) * 10000;
                        arrived_R = true;
                        uint32_t t_R_ms = (uint32_t)((arrive_us_R - t0_us) / 1000);
                        ESP_LOGW(TAG, "[ARRIVED R] %u ms (容差 ±%.1f°)", (unsigned)t_R_ms, ARRIVE_TOL_DEG);
                    }
                } else {
                    steady_cnt_R = 0;
                }
            }

            // 双侧都到 -> 测量结束
            if (arrived_L && arrived_R) {
                uint32_t t_L_ms = (uint32_t)((arrive_us_L - t0_us) / 1000);
                uint32_t t_R_ms = (uint32_t)((arrive_us_R - t0_us) / 1000);
                uint32_t t_max  = (t_L_ms > t_R_ms) ? t_L_ms : t_R_ms;
                ESP_LOGW(TAG, "[DONE] target=%.1f°  L_arrive=%u ms  R_arrive=%u ms  整体=%u ms",
                         cur_target, (unsigned)t_L_ms, (unsigned)t_R_ms, (unsigned)t_max);
                measuring = false;
            }
            // 超时
            else if (elapsed_ms >= TIMEOUT_MS) {
                ESP_LOGE(TAG, "[TIMEOUT] %u ms 仍未到达: L_err=%+.2f° R_err=%+.2f° (容差 ±%.1f°)",
                         (unsigned)elapsed_ms, err_L, err_R, ARRIVE_TOL_DEG);
                measuring = false;
            }
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// SPI 编码器原始数据打印任务（仅在 MODE_TEST_ENCODER_SPI 下启动，200ms/次）
void encoder_spi_raw_print_task(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    while (1) {
        uint8_t rx_l[6] = {0}, rx_r[6] = {0};
        bool ok_l = steering_control_spi_read_raw(0, rx_l);
        bool ok_r = steering_control_spi_read_raw(1, rx_r);

        if (ok_l) {
            uint8_t angle_h = rx_l[2], angle_l = rx_l[3];
            uint8_t status  = rx_l[4] & 0x07;
            uint16_t raw15  = ((uint16_t)angle_h << 7) | (angle_l >> 1);
            float angle     = raw15 / 32768.0f * 360.0f;
            ESP_LOGI(TAG, "ENC L | rx=%02X %02X %02X %02X %02X %02X | h=%02X l=%02X status=%d raw15=%5u angle=%6.1f",
                     rx_l[0], rx_l[1], rx_l[2], rx_l[3], rx_l[4], rx_l[5],
                     angle_h, angle_l, status, raw15, angle);
        } else {
            ESP_LOGE(TAG, "ENC L SPI 传输失败");
        }

        if (ok_r) {
            uint8_t angle_h = rx_r[2], angle_l = rx_r[3];
            uint8_t status  = rx_r[4] & 0x07;
            uint16_t raw15  = ((uint16_t)angle_h << 7) | (angle_l >> 1);
            float angle     = raw15 / 32768.0f * 360.0f;
            ESP_LOGI(TAG, "ENC R | rx=%02X %02X %02X %02X %02X %02X | h=%02X l=%02X status=%d raw15=%5u angle=%6.1f",
                     rx_r[0], rx_r[1], rx_r[2], rx_r[3], rx_r[4], rx_r[5],
                     angle_h, angle_l, status, raw15, angle);
        } else {
            ESP_LOGE(TAG, "ENC R SPI 传输失败");
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

    // 用于硬急停首次触发时打印一次日志（避免 ISR 内调日志）
    static bool s_hard_estop_logged = false;
    // 恢复检测：GPIO 稳定回到非急停电平的连续帧数（50帧×10ms = 500ms）
    static int  s_recovery_count    = 0;

    while (1) {
        // ====== 硬急停（最高优先级，直接压 4 路 PWM = 1500us）======
        if (g_hard_estop) {
            // 首次进入时打印一次，后续循环不重复刷屏
            if (!s_hard_estop_logged) {
                ESP_LOGE(TAG, "⛔ 磁控开关断开！硬急停已激活，所有输出锁定 1500µs");
                s_hard_estop_logged = true;
            }

            motor_control_emergency_stop();             // 两个 ESC
            motor_control_set_steering_pwm(1500, 1500); // 两个舵机

            // 恢复检测：轮询 GPIO，非急停电平持续 500ms 才视为磁铁真正吸合
            // （不依赖 ISR 边沿，彻底规避触点抖动和开关类型差异）
            if (gpio_get_level((gpio_num_t)PIN_MAG_ESTOP) != MAG_ESTOP_TRIGGER_LEVEL) {
                if (++s_recovery_count >= 50) {
                    ESP_LOGW(TAG, "✅ 磁控开关重新吸合，1s 后重启...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                }
            } else {
                s_recovery_count = 0;  // 电平不稳，重新计数
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // 硬急停解除后重置计数（正常情况下不会走到这里，除非手动清 g_hard_estop）
        s_hard_estop_logged = false;
        s_recovery_count    = 0;

        // ====== 急停检查（高优先级，遥控触发）======
        if (g_estop_active) {
            // 仍然刷新 roll 显示，方便 App 端观察姿态
#if USE_BNO055_FOR_ROLL
            balance_controller_update(&imu_data, &state);  // BNO055 路径：内部读取
            g_last_roll_deg = state.roll_deg;
#else
            if (imu_driver_read(&imu_data) == ESP_OK) {
                balance_controller_update(&imu_data, &state);
                g_last_roll_deg = state.roll_deg;
            }
#endif
            motor_control_emergency_stop();
            steering_control_set_target(180.0f, 180.0f);
            steering_control_update();
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
            continue;
        }

        // ====== RC 遥感油门：有信号时直接覆写 g_forward_thrust ======
        // 摇杆归中（raw=0）时也归零——松开摇杆=停船，优先于 Blinker/串口设定值。
        // 巡航激活时 rc_input_get_throttle() 返回巡航值而非 0，正常接管。
        // 摇杆未插/信号超时时 rc_input_is_valid()==false，Blinker/串口可控制。
        if (rc_input_is_valid()) {
            g_forward_thrust = rc_input_get_throttle();
        }

        // ====== 虚拟锚点开关轮询（100Hz + 50ms 消抖 + 边沿触发） ======
        {
            static int s_anc_sw_stable = ANCHOR_SW_INIT_AS_ACTIVE ? ANCHOR_SW_ACTIVE_LEVEL
                                                                  : !ANCHOR_SW_ACTIVE_LEVEL;
            static int s_anc_sw_last_raw = -1;
            static int s_anc_sw_cnt      = 0;
            const int  STABLE_FRAMES     = ANCHOR_SW_DEBOUNCE_MS / 10;

            int raw = gpio_get_level((gpio_num_t)PIN_ANCHOR_SWITCH);
            if (raw == s_anc_sw_last_raw) {
                if (s_anc_sw_cnt < STABLE_FRAMES) s_anc_sw_cnt++;
            } else {
                s_anc_sw_cnt      = 0;
                s_anc_sw_last_raw = raw;
            }
            if (s_anc_sw_cnt >= STABLE_FRAMES && raw != s_anc_sw_stable) {
                s_anc_sw_stable = raw;
                if (raw == ANCHOR_SW_ACTIVE_LEVEL) {
                    ESP_LOGW(TAG, "\xF0\x9F\xAA\x9D 锚点开关闭合 → 抛锚");
                    anchor_set_here();
                } else {
                    ESP_LOGW(TAG, "\xF0\x9F\xAA\x9D 锚点开关断开 → 起锚");
                    anchor_release();
                }
            }
        }

        // ====== 虚拟锚点接管（若激活） ======
        // 锚点会按需覆写 g_forward_thrust 和 g_target_heading / g_heading_hold_active；
        // 未激活时此函数立刻返回，不影响 RC / Blinker 控制。
        // ====== 传感器融合更新（100Hz）======
        fusion_update();

        anchor_control_update();

        // ====== 定速巡航联动定航向 ======
        // 检测巡航状态边沿：激活时自动锁定当前航向；取消时自动解除航向保持。
        {
            static bool s_prev_cruising = false;
            bool now_cruising = rc_input_is_cruising();
            if (now_cruising && !s_prev_cruising) {
                // 巡航刚激活：锁定当前航向
                float hdg = 0.0f;
                if (bno055_get_heading(&hdg) == ESP_OK) {
                    g_target_heading      = hdg;
                    g_heading_hold_active = true;
                    ESP_LOGI(TAG, "CRZ | 定速 %.1f%% + 定航向 %.1f° 同时激活",
                             g_forward_thrust, hdg);
                } else {
                    g_heading_hold_active = false;
                    ESP_LOGW(TAG, "CRZ | 定速 %.1f%% 激活，BNO055 读取失败，仅定速",
                             g_forward_thrust);
                }
            } else if (!now_cruising && s_prev_cruising) {
                // 巡航刚取消：解除航向保持
                g_heading_hold_active = false;
                ESP_LOGI(TAG, "CRZ | 定速取消，航向保持已解除");
            }
            s_prev_cruising = now_cruising;
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
#if USE_BNO055_FOR_ROLL
        // BNO055 路径：balance_controller 内部读取传感器，无需 imu_driver_read
        balance_controller_update(&imu_data, &state);
        imu_ok = true;
#else
        if (imu_driver_read(&imu_data) == ESP_OK) {
            // 算出需要抗抗倒覆的垂直力矩 tau_total
            balance_controller_update(&imu_data, &state);
            imu_ok = true;
        }
#endif
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

                    goto control_loop_tail;  // 跳过下面的 V3 + Scheme A 路径
                }

                // 4. 由 (V, H) 解算舵机偏角与推力幅值
                //    atan2(H, V) ∈ (-π, π]，可直接覆盖 0~360° 全部映射
                //    特别注意：dV=0 且 H=0 时 V_L = -0.0f，atan2(0,-0) = π，会让左舵机跑到 0°；
                //    所以这里显式处理"零矢量"情形，保持 180° 中立。
                //
                // ===== 差速转向：左右水平分量取不等值 =====
                // 仅在 |g_forward_thrust| > TURN_MIN_FWD_PCT 且 g_turn_state ≠ 0 时生效。
                // 物理映射注意：本文件后续把 L/R 整组互换下发（send_tgt_L = 360 - filter_target_R），
                // 因此 H_L / H_R 这里指的是"算法系"的左右，与物理桨的对应关系会在 L/R 互换中处理；
                // 经实测验证：g_turn_state=+1 (右转按钮) 时 H_L > H_R → 算法左侧推力大 →
                // 互换后物理右侧电机推力大 → 船头向左偏 ⇒ 与按钮语义反了。
                // 因此这里把 turn_dir 取反，保证按钮上的"右"就是物理船的"右转"。
                float H_L = H_thrust;
                float H_R = H_thrust;
                int turn_dir = 0;
                if (fabsf(g_forward_thrust) > TURN_MIN_FWD_PCT) {
                    turn_dir = -g_turn_state;  // 取反以匹配物理 L/R 互换后的转向语义
                }
                if (turn_dir != 0) {
                    float dH = (float)turn_dir * (TURN_DELTA_H * 0.5f);
                    H_L = H_thrust + dH;
                    H_R = H_thrust - dH;
                }

                // ===== 航向保持（BNO055 绝对偏航角 PI 控制） =====
                // turn_f 按下 → g_heading_hold_active=true，g_target_heading=当前偏航。
                // 此时忽略 g_turn_state，用偏航误差连续调整 dH。
                // turn_l / turn_r 按下 → g_heading_hold_active=false，退回手动差速。
                if (g_heading_hold_active && fabsf(g_forward_thrust) > TURN_MIN_FWD_PCT) {
                    float cur_heading = 0.0f;
                    if (bno055_get_heading(&cur_heading) == ESP_OK) {
                        // 偏航误差：取最短路径（±180°）
                        float yaw_err = g_target_heading - cur_heading;
                        if (yaw_err >  180.0f) yaw_err -= 360.0f;
                        if (yaw_err < -180.0f) yaw_err += 360.0f;

                        // PI 控制：积分仅在误差较小时累积，防止大角度偏差积分饱和
                        static float heading_integral = 0.0f;
                        const float HEADING_INT_LIMIT = TURN_DELTA_H / HEADING_KI;
                        if (fabsf(yaw_err) < 30.0f) {
                            heading_integral += yaw_err * 0.01f;  // dt = 0.01s
                            if (heading_integral >  HEADING_INT_LIMIT) heading_integral =  HEADING_INT_LIMIT;
                            if (heading_integral < -HEADING_INT_LIMIT) heading_integral = -HEADING_INT_LIMIT;
                        } else {
                            heading_integral = 0.0f;  // 大误差时清积分
                        }

                        float dH_heading = HEADING_KP * yaw_err + HEADING_KI * heading_integral;

                        // 钳制到差速上限（与手动差速共用 TURN_DELTA_H）
                        if (dH_heading >  TURN_DELTA_H) dH_heading =  TURN_DELTA_H;
                        if (dH_heading < -TURN_DELTA_H) dH_heading = -TURN_DELTA_H;

                        // 偏航误差 > 0 → 当前航向偏左 → 需右转 → H_R 增大
                        H_L = H_thrust - dH_heading;
                        H_R = H_thrust + dH_heading;
                    }
                    // BNO055 读取失败：保持上一帧的 H_L/H_R（已是 H_thrust+dH_heading），
                    // 下一帧继续尝试。
                }

                // ===== 方案 C：H 自动降级以满足舵机角度限制（左右独立） =====
                // 舵机允许范围 [80°, 280°]，即 target = 180 ± phi，|phi| ≤ 100°。
                // 当 V_i < 0 且 H/|V_i| > tan(80°)≈5.67 时 phi < 100°（target 越界），
                // 将 H 钳制在 TAN80*|V_i|，使 phi 恰好 = 100°，target 落在 280°/80° 边界。
                // 差速后两侧 H 不同，必须分别钳制 H_L_eff、H_R_eff。
                //
                // 【V 死区】只有 V_i 显著为负（< -V_DEAD）才触发限制。
                // 否则微小的 dV 噪声（如 ±0.5）会让 H_max 跌到接近 0，瞬间杀掉前进推力，
                // 引发"舵机刚到位 → 推力突然消失 → 卡在 PWM 1500"的现象。
                // V_DEAD=10 对应 ~6.5% 推力等量级，远高于平衡环路噪声。
                const float TAN80 = 5.6712818f;  // tan(80°)，对应 |phi|=100° 边界
                const float V_DEAD = 10.0f;

                float H_L_eff = H_L;
                if (V_L < -V_DEAD) {
                    float lim = TAN80 * fabsf(V_L);
                    if (H_L_eff >  lim) H_L_eff =  lim;
                    if (H_L_eff < -lim) H_L_eff = -lim;
                }

                float H_R_eff = H_R;
                if (V_R < -V_DEAD) {
                    float lim = TAN80 * fabsf(V_R);
                    if (H_R_eff >  lim) H_R_eff =  lim;
                    if (H_R_eff < -lim) H_R_eff = -lim;
                }

                const float ZERO_EPS = 1e-3f;
                float phi_R_deg, phi_L_deg;
                if (fabsf(V_R) < ZERO_EPS && fabsf(H_R_eff) < ZERO_EPS) {
                    phi_R_deg = 0.0f;
                } else {
                    phi_R_deg = atan2f(H_R_eff, V_R) * 180.0f / (float)M_PI;
                }
                if (fabsf(V_L) < ZERO_EPS && fabsf(H_L_eff) < ZERO_EPS) {
                    phi_L_deg = 0.0f;
                } else {
                    phi_L_deg = atan2f(H_L_eff, V_L) * 180.0f / (float)M_PI;
                }

                float T_R_target = sqrtf(V_R * V_R + H_R_eff * H_R_eff);
                float T_L_target = sqrtf(V_L * V_L + H_L_eff * H_L_eff);

                // 5. 映射到舵机物理角度（左右镜像，左 +、右 −）
                //    方案 C + 下方 clamp 共同确保 target_angle ∈ [80°, 280°]。
                float target_angle_R = 180.0f - phi_R_deg;
                float target_angle_L = 180.0f + phi_L_deg;

                // 6. 舵机指令平滑滤波
                //    两端点都在 [80, 280] 连续弧内（200° 弧），不跨 0/360，用普通减法即可；
                //    不能用环形最短路径（两端相距 >180° 时会穿过 0/360 禁区）。
                //    α=0.4 → τ≈15ms：滤掉 IMU 单帧噪声，但不显著滞后于真实目标变化。
                float diff_L = target_angle_L - filter_target_L;
                filter_target_L += 0.4f * diff_L;
                if (filter_target_L < 80.0f)  filter_target_L = 80.0f;
                if (filter_target_L > 280.0f) filter_target_L = 280.0f;

                float diff_R = target_angle_R - filter_target_R;
                filter_target_R += 0.4f * diff_R;
                if (filter_target_R < 80.0f)  filter_target_R = 80.0f;
                if (filter_target_R > 280.0f) filter_target_R = 280.0f;

                // ⚠ 实测：左右物理装配相对算法是镜像的，此处把 L/R 整组互换下发
                //    互换后角度公式语义也跟着反了，所以再绕 180° 镜像一次（360 - x）
                //    [80, 280] 区间关于 180° 中心对称，360-x 仍落在 [80, 280]。
#if STEER_SEND_LEFT_REVERSE
                float send_tgt_L = 360.0f - filter_target_R;
#else
                float send_tgt_L = filter_target_R;
#endif
#if STEER_SEND_RIGHT_REVERSE
                float send_tgt_R = 360.0f - filter_target_L;
#else
                float send_tgt_R = filter_target_L;
#endif
                // 安全夹制（双保险，浮点误差不会越界）
                if (send_tgt_L < 80.0f)  send_tgt_L = 80.0f;
                if (send_tgt_L > 280.0f) send_tgt_L = 280.0f;
                if (send_tgt_R < 80.0f)  send_tgt_R = 80.0f;
                if (send_tgt_R > 280.0f) send_tgt_R = 280.0f;
                steering_control_set_target(send_tgt_L, send_tgt_R);

                // 取出 PID 实际跟踪的目标角（编码器系，与 cur_steer_* 同参考系）
                // ⚠ 不能直接用 send_tgt_R 做门控判据：steering_control_set_target() 内部
                //   对右侧做了 360- 翻转（历史遗留，与本文件 send_tgt_R 的镜像运算"双重抵消"
                //   后 PID 才能让右舵机走到正确位置）。直接用 send_tgt_R 会让右侧 |err|≈180°，
                //   门控被锁死在 R_FLOOR，PWM 永远只到 1/3。
                float pid_tgt_L, pid_tgt_R;
                steering_control_get_target(&pid_tgt_L, &pid_tgt_R);

                // 7. 推力下发——两侧均不反转（电机一律 PWM > 1500），方向完全由舵机决定
                //
                // 【舵机就位率门控】方案 A 改进版：上升非对称限速，下降不门控
                //   r = R_FLOOR + (1-R_FLOOR) * max(0, 1 - |Δθ|/θ_tol)   ∈ [R_FLOOR, 1]
                //   T_out = T_prev + r * (T_cmd - T_prev)   (仅当 T_cmd > T_prev)
                //   T_out = T_cmd                            (T_cmd <= T_prev 直通)
                //
                // 【为什么需要 R_FLOOR > 0】
                //   舵机追大转角（如 180→270）期间 |err| 会持续 >SERVO_TOL_DEG，
                //   纯 r=0 会让 last_thrust_motor 永久卡死在初值（实测 PWM 卡 1523 ≈ 1/3 推力），
                //   永远等不到舵机就位。给一个最小爬升率 R_FLOOR=0.08，保证即使大 err
                //   也能以 ~10 帧 (100ms) 爬到目标的一半，3-5 个 100Hz 周期就能跟上 30% 油门。
                //
                //   左右独立计算后取 min（对称门控，避免左右推力不等导致 yaw）
                //   注：last_thrust_motor_L/R 已提到 control_core_task 函数顶部声明
                const float SERVO_TOL_DEG = 30.0f;
                const float R_FLOOR = 0.08f;

                // 物理左侧：用 PID 实际目标 pid_tgt_L 与编码器读数 cur_steer_left 比对
                float err_servo_L = pid_tgt_L - cur_steer_left;
                while (err_servo_L >  180.0f) err_servo_L -= 360.0f;
                while (err_servo_L < -180.0f) err_servo_L += 360.0f;
                float r_L_raw = 1.0f - fabsf(err_servo_L) / SERVO_TOL_DEG;
                if (r_L_raw < 0.0f) r_L_raw = 0.0f;
                if (r_L_raw > 1.0f) r_L_raw = 1.0f;
                float r_L = R_FLOOR + (1.0f - R_FLOOR) * r_L_raw;

                // 物理右侧：用 PID 实际目标 pid_tgt_R（已含内部 360- 翻转）与 cur_steer_right 比对
                float err_servo_R = pid_tgt_R - cur_steer_right;
                while (err_servo_R >  180.0f) err_servo_R -= 360.0f;
                while (err_servo_R < -180.0f) err_servo_R += 360.0f;
                float r_R_raw = 1.0f - fabsf(err_servo_R) / SERVO_TOL_DEG;
                if (r_R_raw < 0.0f) r_R_raw = 0.0f;
                if (r_R_raw > 1.0f) r_R_raw = 1.0f;
                float r_R = R_FLOOR + (1.0f - R_FLOOR) * r_R_raw;

                // ===== 对称门控 =====
                // Scheme A 本意：按"最慢就位的一侧"统一限速，保证左右推力上升对称。
                // 即使在纯前进 / 倾斜转弯任何情况，左右目标都该用同一个 r。
                // （原先按 |diff_target|<5° 判断对称在此应用中不成立——纯前进时
                //   两侧舵机本就是镜像关系 ~180° 张开。）
                {
                    float r_min = (r_L < r_R) ? r_L : r_R;
                    r_L = r_min;
                    r_R = r_min;
                }

                float thrust_motor_L = T_R_target;  // L/R 已互换：物理左电机收 T_R_target
                if (thrust_motor_L > last_thrust_motor_L) {
                    thrust_motor_L = last_thrust_motor_L + r_L * (thrust_motor_L - last_thrust_motor_L);
                }
                last_thrust_motor_L = thrust_motor_L;

                float thrust_motor_R = T_L_target;  // L/R 已互换：物理右电机收 T_L_target
                if (thrust_motor_R > last_thrust_motor_R) {
                    thrust_motor_R = last_thrust_motor_R + r_R * (thrust_motor_R - last_thrust_motor_R);
                }
                last_thrust_motor_R = thrust_motor_R;

                motor_control_set_pwm_bidirectional(thrust_motor_L, thrust_motor_R, false, false);

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

        // ====== 编码器失效硬保护 ======
        // 任意一侧编码器掉线 → 立即把所有 4 路 PWM 全部锁回 1500us（舵机停转 + 大电机停转）。
        // 比 steering_control_update() 内部的"PID 不输出"更彻底——直接覆盖本拍的推力下发，
        // 避免大电机带桨的同时舵机失控乱转。同时清零 Scheme A 历史，避免恢复瞬间跳变。
        if (enc_left_fault || enc_right_fault) {
            motor_control_set_pwm_bidirectional(0.0f, 0.0f, false, false);  // 大电机 → 1500
            motor_control_set_steering_pwm(1500, 1500);                      // 舵机 → 1500
            last_thrust_motor_L = 0.0f;
            last_thrust_motor_R = 0.0f;
            // 注意：steering_control_update() 仍会被调用，但其内部对失效侧也会输出 1500，
            // 不会覆盖我们刚才的设置（PID 静音 + 中立 PWM）。
        }

        // 统一更新小电机位置 (下发滤波后的 PWM)
        steering_control_update(); 

        // 串口实时数据监测 (每 10 帧打一条，10Hz)
        static int print_cnt = 0;
        if (++print_cnt >= MAIN_PRINT_INTERVAL) { 
            // 获取实际下发的 PWM 脉宽（大电机 & 舵机）
            uint32_t actual_pwm_L, actual_pwm_R;
            motor_control_get_last_pwm(&actual_pwm_L, &actual_pwm_R);
            uint32_t steer_pwm_L, steer_pwm_R;
            motor_control_get_last_steer_pwm(&steer_pwm_L, &steer_pwm_R);

            // 显示用的 Tgt = 真实下发给 PID 的目标角（编码器系），
            // 与 Act（编码器读数）同参考系，收敛后两者一致。
            float disp_tgt_L, disp_tgt_R;
            steering_control_get_target(&disp_tgt_L, &disp_tgt_R);

            float dbg_heading = 0.0f;
            bno055_get_heading(&dbg_heading);

            // 摇杆诊断信息（含上/下沿计数器、丢弃计数、原始脉宽）
            // rc_input_print_diag();
            uint32_t rc_pwm = rc_input_get_raw_pwm();

            if (rc_input_is_cruising() && g_heading_hold_active) {
                // 定速 + 定向：额外显示航向目标→当前
                ESP_LOGI(TAG, "RC:%luμs | Fwd:%.1f%%(CRZ) | L:%.1f→%.1f | R:%.1f→%.1f | SteerPWM:%u/%u | MotorPWM:%u/%u | Roll:%.2f° | Hdg:%.1f°→%.1f°(HOLD) | EncL:%s EncR:%s",
                         (unsigned long)rc_pwm,
                         g_forward_thrust,
                         disp_tgt_L, cur_steer_left, disp_tgt_R, cur_steer_right,
                         steer_pwm_L, steer_pwm_R,
                         actual_pwm_L, actual_pwm_R,
                         state.roll_deg,
                         g_target_heading, dbg_heading,
                         enc_left_fault ? "X" : "✓", enc_right_fault ? "X" : "✓");
            } else {
                ESP_LOGI(TAG, "RC:%luμs | Fwd:%.1f%%%s | L:%.1f→%.1f | R:%.1f→%.1f | SteerPWM:%u/%u | MotorPWM:%u/%u | Roll:%.2f° | Hdg:%.1f° | EncL:%s EncR:%s",
                         (unsigned long)rc_pwm,
                         g_forward_thrust, rc_input_is_cruising() ? "(CRZ)" : "",
                         disp_tgt_L, cur_steer_left, disp_tgt_R, cur_steer_right,
                         steer_pwm_L, steer_pwm_R,
                         actual_pwm_L, actual_pwm_R,
                         state.roll_deg, dbg_heading,
                         enc_left_fault ? "X" : "✓", enc_right_fault ? "X" : "✓");
            }
            print_cnt = 0;
        }

        // 绝对延时：确保本次循环精准踩在 10 毫秒节点
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

// ====== 磁控急停 ISR + 初始化 ======
// ISR 职责只有一个：检测到急停触发电平时立即设置 g_hard_estop。
// 恢复检测（吸合→重启）完全在任务内轮询，不依赖边沿方向，彻底避免
// 触点抖动和开关类型（NO/NC）造成的误判。
//
// 触发电平由 system_config.h 中 MAG_ESTOP_TRIGGER_LEVEL 决定：
//   NO 型（磁铁在位=触点闭合=LOW，移走=触点断开=HIGH）→ 设 1
//   NC 型（磁铁在位=触点断开=HIGH，移走=触点闭合=LOW）→ 设 0
static void IRAM_ATTR mag_estop_isr(void *arg)
{
    if (gpio_get_level((gpio_num_t)PIN_MAG_ESTOP) == MAG_ESTOP_TRIGGER_LEVEL) {
        g_hard_estop = true;
    }
}

// ====== 虚拟锚点开关（NO 型常开，内部上拉，纯轮询）======
static void anchor_switch_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_ANCHOR_SWITCH),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "\xF0\x9F\xAA\x9D 锚点开关已初始化 GPIO%d (LOW=抛锚 HIGH=起锚)",
             PIN_ANCHOR_SWITCH);
}

static void mag_estop_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_MAG_ESTOP),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,    // 内部上拉，无需外部电阻
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,     // 任意边沿均检查电平
    };
    gpio_config(&io_conf);

    // ISR service 可能已由 rc_input_init / steering_control_init 安装，
    // 返回 ESP_ERR_INVALID_STATE 表示已安装，属正常，直接继续。
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "mag_estop: ISR service 安装失败 (%s)", esp_err_to_name(err));
        return;
    }
    gpio_isr_handler_add((gpio_num_t)PIN_MAG_ESTOP, mag_estop_isr, NULL);
    ESP_LOGI(TAG, "磁控急停已初始化 GPIO%d (触发电平=%d，移走→硬急停，吸合→重启)",
             PIN_MAG_ESTOP, MAG_ESTOP_TRIGGER_LEVEL);
}

void app_main(void)
{
    ESP_LOGI(TAG, "SteadySail Version 2 - Dual IMU + Vector Thrust");

    // 0. 初始化全局可调参数（含 NVS 加载运行模式）
    control_params_init();

    // 1. 初始化所有设备
#if !USE_BNO055_FOR_ROLL
    // MPU6050 路径：I2C0 由 imu_driver_init 接管
    if (imu_driver_init() != ESP_OK) {
        ESP_LOGE(TAG, "IMU 硬件异常！确保连线正确！(或当前是在无传感器测试)");
    }
#endif
    balance_controller_init();
    motor_control_init();
    steering_control_init(); // 开启外部中断读取编码器；内部会尝试从 NVS 加载历史校准

    // 2. 初始化 RC 油门 PWM 输入（GPIO PIN_RC_THROTTLE）
    rc_input_init(PIN_RC_THROTTLE);  // 失败仅打印警告，不中断启动

    // 3. 初始化磁控急停（NC 干簧管，GPIO PIN_MAG_ESTOP；内部上拉，下降沿触发）
    mag_estop_init();

    // 3.1 初始化虚拟锚点模块 + 锚点开关 + GPS（顺序：模块先就绪，再启 GPS 喂数据）
    anchor_control_init();
    anchor_switch_init();
    gps_nmea_start();

    // 4. 初始化 BNO055（I2C0，GPIO 8/9，考接 MPU6050 原接口）
    //    USE_BNO055_FOR_ROLL=1 时：BNO055 同时负责 Roll 平衡与航向保持；=0 时仅用于航向保持
    if (bno055_init() != ESP_OK) {
#if USE_BNO055_FOR_ROLL
        ESP_LOGE(TAG, "⚠ BNO055 初始化失败！平衡控制和航向保持均不可用");
#else
        ESP_LOGW(TAG, "⚠ BNO055 初始化失败，航向保持功能不可用");
#endif
    }

    // 注意：不再每次启动都自动校准。
    //  - 若 NVS 中已有保存的零点偏移，steering_control_init() 会自动加载。
    //  - 若 NVS 中没有数据（首次烧录后），舵机被锁定在 PWM=1500 中立位；
    //    用户把舵机摆正下方后，串口输入 'cal' 即可完成首次校准并写入 NVS。
    vTaskDelay(pdMS_TO_TICKS(100));

    // 初始化 GPS+BNO055 传感器融合模块（必须在 fusion_calibrate/fusion_update 之前）
    fusion_init();

    // 加速度计零偏静态标定（船静止时，1.5s）
    fusion_calibrate_acc_bias(1500);

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
                        ESP_LOGI(TAG, "[CAL] 开始校准舵机零点（请确认舵机已摆正下方）...");
                        steering_control_calibrate_and_save();
                        ESP_LOGI(TAG, "[CAL] 校准完成，已写入 NVS。下次上电将自动加载。");
                        rx_len = 0;
                        continue;
                    }
                    if (strcmp(rx_buf, "reboot") == 0 || strcmp(rx_buf, "REBOOT") == 0) {
                        ESP_LOGW(TAG, "[REBOOT] 1 秒后重启 ESP32...");
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        esp_restart();
                    }
                    char *endptr = NULL;
                    float input_val = strtof(rx_buf, &endptr);
                    if (endptr != rx_buf) {
                        g_test_steer_angle = input_val;
                        if(g_test_steer_angle < 90.0f) g_test_steer_angle = 90.0f;
                        if(g_test_steer_angle > 270.0f) g_test_steer_angle = 270.0f;
                        ESP_LOGI(TAG, "收到角度指令! 目标设为: %.1f °", g_test_steer_angle);
                    } else {
                        ESP_LOGW(TAG, "无效输入: %s", rx_buf);
                    }
                    rx_len = 0;
                }
            } else if (rx_len < sizeof(rx_buf) - 1) {
                rx_buf[rx_len++] = (char)ch;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

#elif CURRENT_RUN_MODE == MODE_TEST_ENCODER_SPI
    ESP_LOGI(TAG, "[ENC SPI RAW] 模式启动 - 所有控制功能正常，同时输出编码器 SPI 原始字节 (200ms/次)");
    xTaskCreatePinnedToCore(encoder_spi_raw_print_task, "enc_spi_raw", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(control_core_task, "control_core_task", 4096, NULL, 5, NULL, 1);
    blinker_bridge_start();

    // 串口命令（cal / reboot / w / s）与全功能模式一致
    {
        char rx_buf[64] = {0};
        int rx_len = 0;
        while (1) {
            int ch = getchar();
            if (ch != EOF) {
                if (ch == '\r' || ch == '\n') {
                    if (rx_len > 0) {
                        rx_buf[rx_len] = '\0';
                        if (strcmp(rx_buf, "cal") == 0 || strcmp(rx_buf, "CAL") == 0) {
                            ESP_LOGI(TAG, "[CAL] 开始校准舵机零点...");
                            steering_control_calibrate_and_save();
                            ESP_LOGI(TAG, "[CAL] 校准完成。");
                        } else if (strcmp(rx_buf, "reboot") == 0 || strcmp(rx_buf, "REBOOT") == 0) {
                            ESP_LOGW(TAG, "[REBOOT] 1 秒后重启...");
                            vTaskDelay(pdMS_TO_TICKS(1000));
                            esp_restart();
                        } else if (strcmp(rx_buf, "w") == 0 || strcmp(rx_buf, "W") == 0) {
                            g_forward_thrust += 10.0f;
                            if (g_forward_thrust > 50.0f) g_forward_thrust = 50.0f;
                            ESP_LOGI(TAG, "推力 +10%%, 当前: %.1f%%", g_forward_thrust);
                        } else if (strcmp(rx_buf, "s") == 0 || strcmp(rx_buf, "S") == 0) {
                            g_forward_thrust -= 10.0f;
                            if (g_forward_thrust < -50.0f) g_forward_thrust = -50.0f;
                            ESP_LOGI(TAG, "推力 -10%%, 当前: %.1f%%", g_forward_thrust);
                        } else {
                            char *endptr = NULL;
                            float v = strtof(rx_buf, &endptr);
                            if (endptr != rx_buf) {
                                g_forward_thrust = v;
                                if (g_forward_thrust > 50.0f) g_forward_thrust = 50.0f;
                                if (g_forward_thrust < -50.0f) g_forward_thrust = -50.0f;
                                ESP_LOGI(TAG, "推力设为 %.1f%%", g_forward_thrust);
                            }
                        }
                        rx_len = 0;
                    }
                } else if (rx_len < (int)sizeof(rx_buf) - 1) {
                    rx_buf[rx_len++] = (char)ch;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
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
                    if(g_forward_thrust > 50.0f) g_forward_thrust = 50.0f;
                    ESP_LOGI(TAG, "前进推力 +10%%，当前目标: %.1f %%", g_forward_thrust);
                } 
                else if (strcmp(rx_buf, "s") == 0 || strcmp(rx_buf, "S") == 0) {
                    g_forward_thrust -= 10.0f;
                    if(g_forward_thrust < -50.0f) g_forward_thrust = -50.0f;
                    ESP_LOGI(TAG, "前进推力 -10%%，当前目标: %.1f %%", g_forward_thrust);
                } 
                else if (strcmp(rx_buf, "space") == 0 || rx_buf[0] == ' ') {
                    g_forward_thrust = 0.0f;
                    ESP_LOGI(TAG, "推力归零！原地自平衡！");
                } 
                // 校准舵机零点：把舵机摆到正下方（180° 竖直）后输入 'cal'
                else if (strcmp(rx_buf, "cal") == 0 || strcmp(rx_buf, "CAL") == 0) {
                    ESP_LOGI(TAG, "[CAL] 开始校准舵机零点（请确认舵机已摆正下方）...");
                    steering_control_calibrate_and_save();
                    ESP_LOGI(TAG, "[CAL] 校准完成，已写入 NVS。下次上电将自动加载。");
                }
                else if (strcmp(rx_buf, "reboot") == 0 || strcmp(rx_buf, "REBOOT") == 0) {
                    ESP_LOGW(TAG, "[REBOOT] 1 秒后重启 ESP32...");
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
                        ESP_LOGI(TAG, "[DEMO] 模拟 Roll 设为 %.2f°", roll_val);
                    } else {
                        ESP_LOGW(TAG, "无效 r 命令: %s （用法: r15 / r 15 / r-20）", rx_buf);
                    }
#else
                    ESP_LOGW(TAG, "r 命令仅在 MODE_FULL_INTEGRATION + DEMO_MANUAL_ROLL=1 下生效");
#endif
                }
                // 解析具体数字
                else {
                    char *endptr = NULL;
                    float input_val = strtof(rx_buf, &endptr);
                    if (endptr != rx_buf) { 
                        g_forward_thrust = input_val;
                        if(g_forward_thrust > 50.0f) g_forward_thrust = 50.0f;
                        if(g_forward_thrust < -50.0f) g_forward_thrust = -50.0f;
                        ESP_LOGI(TAG, "收到绝对推力指令! 目标设为: %.1f %%", g_forward_thrust);
                    } else {
                        ESP_LOGW(TAG, "无效输入: %s", rx_buf);
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
