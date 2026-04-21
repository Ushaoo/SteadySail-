#include "motor_control.h"
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR";

// ======================= PWM (LEDC) 配置 =======================
// 采用 50Hz 周期 20ms, 如果使用 14位分辨率 (0~16383), 那么：
// 1000 us = 1000 / 20000 * 16384 ≈ 819 
// 1500 us = 1500 / 20000 * 16384 ≈ 1228
// 2000 us = 2000 / 20000 * 16384 ≈ 1638

#define LEDC_TIMER               LEDC_TIMER_0
#define LEDC_MODE                LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES            LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY           50 // ESC 通常要求 50Hz

#define STEER_LEFT_CHANNEL       LEDC_CHANNEL_0
#define STEER_RIGHT_CHANNEL      LEDC_CHANNEL_1
#define THRUST_LEFT_CHANNEL      LEDC_CHANNEL_2
#define THRUST_RIGHT_CHANNEL     LEDC_CHANNEL_3

// 将微秒脉宽转为 LEDC 控制寄存器的占空比值
// 50Hz频率 -> 周期20000us，14位分辨率 -> 0~16383
// 公式: duty = (us / 20000) * 16384 = (us * 16384) / 20000
static inline uint32_t us_to_duty(uint32_t us) {
    uint64_t duty = ((uint64_t)us * 16384) / 20000;
    // 安全检查：确保不会超过最大值16383（14位分辨率）
    // 根据ESP32-S3文档，duty不能等于2^14=16384（会导致溢出）
    if (duty > 16383) {
        ESP_LOGW(TAG, "Duty %llu exceeds max 16383, clamped", duty);
        duty = 16383;
    }
    return (uint32_t)duty;
}



void motor_control_init(void) {
    ESP_LOGI(TAG, "=== Initializing Motor Control (LEDC PWM) ===");
    ESP_LOGI(TAG, "Thrust Left (GPIO %d), Right (GPIO %d)", PIN_THRUST_LEFT, PIN_THRUST_RIGHT);
    ESP_LOGI(TAG, "Steer Left (GPIO %d), Right (GPIO %d)", PIN_STEER_LEFT, PIN_STEER_RIGHT);
    
    // 1. 初始化定时器 (Timer 0, 50Hz)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,        // 14-bit (0~16383)
        .freq_hz          = LEDC_FREQUENCY,        // 50 Hz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_LOGI(TAG, "Configuring LEDC timer: 50Hz, 14-bit resolution");
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. 初始化 4 个硬件通道 (LEDC会自动处理GPIO的mux和方向设置)
    ledc_channel_config_t ch_config[4] = {
        { .gpio_num = PIN_STEER_LEFT,   .speed_mode = LEDC_MODE, .channel = STEER_LEFT_CHANNEL,   .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 },
        { .gpio_num = PIN_STEER_RIGHT,  .speed_mode = LEDC_MODE, .channel = STEER_RIGHT_CHANNEL,  .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 },
        { .gpio_num = PIN_THRUST_LEFT,  .speed_mode = LEDC_MODE, .channel = THRUST_LEFT_CHANNEL,  .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 },
        { .gpio_num = PIN_THRUST_RIGHT, .speed_mode = LEDC_MODE, .channel = THRUST_RIGHT_CHANNEL, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 }
    };

    for (int i = 0; i < 4; i++) {
        ESP_LOGI(TAG, "Configuring channel %d on GPIO %d", ch_config[i].channel, ch_config[i].gpio_num);
        ESP_ERROR_CHECK(ledc_channel_config(&ch_config[i]));
    }
    
    // 启用定时器使其开始输出PWM
    ledc_timer_resume(LEDC_MODE, LEDC_TIMER);
    ESP_LOGI(TAG, "LEDC timer resumed - PWM output enabled");
    
    // 3. 测试PWM输出
    ESP_LOGI(TAG, "Testing PWM output with 1500us (neutral)...");
    uint32_t duty_neutral = us_to_duty(1500);
    
    ledc_set_duty(LEDC_MODE, THRUST_LEFT_CHANNEL, duty_neutral);
    ledc_update_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
    ledc_set_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL, duty_neutral);
    ledc_update_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
    ledc_set_duty(LEDC_MODE, STEER_LEFT_CHANNEL, duty_neutral);
    ledc_update_duty(LEDC_MODE, STEER_LEFT_CHANNEL);
    ledc_set_duty(LEDC_MODE, STEER_RIGHT_CHANNEL, duty_neutral);
    ledc_update_duty(LEDC_MODE, STEER_RIGHT_CHANNEL);
    
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ESP_LOGI(TAG, "=== Motor Control Initialized Successfully! ===");
}

void motor_control_set_steering_pwm(uint32_t pwm_left_us, uint32_t pwm_right_us) {
#if CURRENT_RUN_MODE == MODE_TEST_SENSORS || CURRENT_RUN_MODE == MODE_TEST_BALANCE_ONLY
    // 如果系统不需要开转向功能，锁定为 1500 保持正冲下方
    pwm_left_us = 1500;
    pwm_right_us = 1500;
#endif
    
    // 限制脉宽范围
    pwm_left_us = (pwm_left_us < 1000) ? 1000 : (pwm_left_us > 2000) ? 2000 : pwm_left_us;
    pwm_right_us = (pwm_right_us < 1000) ? 1000 : (pwm_right_us > 2000) ? 2000 : pwm_right_us;
    
    // 直接操作 LEDC
    uint32_t duty_L = us_to_duty(pwm_left_us);
    uint32_t duty_R = us_to_duty(pwm_right_us);
    
    ledc_set_duty(LEDC_MODE, STEER_LEFT_CHANNEL, duty_L);
    ledc_update_duty(LEDC_MODE, STEER_LEFT_CHANNEL);
    ledc_set_duty(LEDC_MODE, STEER_RIGHT_CHANNEL, duty_R);
    ledc_update_duty(LEDC_MODE, STEER_RIGHT_CHANNEL);
    
    vTaskDelay(pdMS_TO_TICKS(30));
}

#define THRUST_SCALE 0.55f   // 力矩转 PWM 的比例系数（源自 Python）

void motor_control_set_thrust_with_compensation(float tau_total, float current_angle_deg) {
    float final_tau = tau_total;

    // 核心联动：如果小电机已经旋转，推力在垂直法线的分量会被削弱，需通过除以 cos 放大主推力进行补偿
    // 为避免 cos(90度)=0 死结，将最大补偿角度钳位于 60 度 (cos60 = 0.5)
    float clamped_angle = current_angle_deg;
    if (clamped_angle > 60.0f) clamped_angle = 60.0f;
    if (clamped_angle < -60.0f) clamped_angle = -60.0f;

    // TODO: 注意根据两侧偏角如果是独立控制的，可能需要分别补偿，但我们这里以同向平衡假设为例
    float cos_alpha = cosf(clamped_angle * M_PI / 180.0f);
    // 当 cos_alpha 为 0.5 时，final_tau 放大 2 倍
    if (cos_alpha > 0.05f) {
        final_tau = tau_total / cos_alpha;
    }

    // 转为 PWM 变化量
    float pwm_adjust = final_tau * THRUST_SCALE;
    
    // 安全钳位 +/- 500 (对应 1000 到 2000 的最大区间)
    if (pwm_adjust > 500.0f) pwm_adjust = 500.0f;
    if (pwm_adjust < -500.0f) pwm_adjust = -500.0f;

    // 左边电机加力，右边电机减力产生力偶矩 (假设翻滚响应为：左强右弱往右翻)
    uint32_t thrust_L = (uint32_t)(1500.0f + pwm_adjust);
    uint32_t thrust_R = (uint32_t)(1500.0f - pwm_adjust);

#if CURRENT_RUN_MODE == MODE_TEST_SENSORS || CURRENT_RUN_MODE == MODE_TEST_STEERING_ONLY
    // 如果系统不需要开主推进，强制闭锁推力处于中立！
    thrust_L = 1500;
    thrust_R = 1500;
#endif

    // 限制脉宽范围
    thrust_L = (thrust_L < 1000) ? 1000 : (thrust_L > 2000) ? 2000 : thrust_L;
    thrust_R = (thrust_R < 1000) ? 1000 : (thrust_R > 2000) ? 2000 : thrust_R;
    
    // 直接操作 LEDC
    uint32_t duty_L = us_to_duty(thrust_L);
    uint32_t duty_R = us_to_duty(thrust_R);
    
    ledc_set_duty(LEDC_MODE, THRUST_LEFT_CHANNEL, duty_L);
    ledc_update_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
    ledc_set_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL, duty_R);
    ledc_update_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
    
    vTaskDelay(pdMS_TO_TICKS(30));
}

void motor_control_emergency_stop(void) {
    static uint32_t last_print_time = 0;
    if (xTaskGetTickCount() - last_print_time > pdMS_TO_TICKS(1000)) {
        ESP_LOGW(TAG, "EMERGENCY THRUST STOP TRIGGERED! (Testing Mode or Safety Limit)");
        last_print_time = xTaskGetTickCount();
    }
    // 紧急状态下，仅停下大电机即主推进器！！
    // 千万不要重置小电机PWM（STEER_XX_CHANNEL），否则会导致无法单独测试转向系统
    uint32_t duty_neutral = us_to_duty(1500);
    
    ledc_set_duty(LEDC_MODE, THRUST_LEFT_CHANNEL, duty_neutral);
    ledc_update_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
    ledc_set_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL, duty_neutral);
    ledc_update_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
    
    vTaskDelay(pdMS_TO_TICKS(30));
}

// ======================= 大电机 (ESC) 校准专有序列 =======================

// MODE_CALIBRATE_ESC: PWM脉宽扫描测试
// 校准模式不使用互斥锁，直接操作LEDC，因为校准任务独占通道
void motor_control_esc_calibrate_task(void *pvParameters) {
    ESP_LOGW(TAG, "=========== ESC (大电机) 校准模式启动 ===========");
    ESP_LOGW(TAG, "配置: 50Hz, 14-bit分辨率, GPIO 18(LEFT) & GPIO 19(RIGHT)");
    
    // 验证系统状态
    ESP_LOGI(TAG, "\n=== 系统诊断 ===");
    ESP_LOGI(TAG, "使用GPIO: 左=%d (CH%d), 右=%d (CH%d)", 
             PIN_THRUST_LEFT, THRUST_LEFT_CHANNEL, PIN_THRUST_RIGHT, THRUST_RIGHT_CHANNEL);
    
    uint32_t freq = ledc_get_freq(LEDC_MODE, LEDC_TIMER);
    ESP_LOGI(TAG, "定时器频率: %u Hz (期望: 50)", freq);
    
    // 测试占空比计算
    uint32_t duty_1000 = us_to_duty(1000);
    uint32_t duty_1500 = us_to_duty(1500);
    uint32_t duty_2000 = us_to_duty(2000);
    ESP_LOGI(TAG, "占空比映射: 1000us=%u, 1500us=%u, 2000us=%u", duty_1000, duty_1500, duty_2000);
    
    if (duty_1000 == duty_1500 || duty_1500 == duty_2000) {
        ESP_LOGE(TAG, "❌ 错误: 占空比计算异常!");
        return;
    }
    
    // 初始化 - 设为 1500 中立
    ESP_LOGI(TAG, "\n初始化: 设置为 1500 us (中立)");
    ledc_set_duty(LEDC_MODE, THRUST_LEFT_CHANNEL, duty_1500);
    ledc_update_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
    ledc_set_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL, duty_1500);
    ledc_update_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // 验证初始占空比
    uint32_t actual = ledc_get_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
    ESP_LOGI(TAG, "验证: 设置1500us后的实际duty = %u (期望%u) %s", 
             actual, duty_1500, (actual == duty_1500) ? "✓" : "✗ WARNING");
    
    vTaskDelay(pdMS_TO_TICKS(1500));
    
    uint32_t loop_count = 0;
    const uint32_t pwm_min = 1000;
    const uint32_t pwm_max = 2000;
    const uint32_t pwm_step = 100;
    const uint32_t step_hold_ms = 1500;
    
    ESP_LOGI(TAG, "✓ 系统正常，开始PWM扫描\n");
    
    while(1) {
        loop_count++;
        ESP_LOGW(TAG, "\n========== 扫描循环 #%u 开始 ==========", loop_count);
        
        // 上升阶段：1000 -> 2000 (100us/step)
        ESP_LOGI(TAG, "--- 上升阶段 ---");
        for (uint32_t pwm = pwm_min; pwm <= pwm_max; pwm += pwm_step) {
            uint32_t expected = us_to_duty(pwm);
            
            // 直接操作LEDC，不使用互斥锁
            ledc_set_duty(LEDC_MODE, THRUST_LEFT_CHANNEL, expected);
            ledc_update_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
            ledc_set_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL, expected);
            ledc_update_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
            
            // 等待硬件生效
            vTaskDelay(pdMS_TO_TICKS(30));
            
            // 读取实际值验证
            uint32_t actual_L = ledc_get_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
            uint32_t actual_R = ledc_get_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
            
            ESP_LOGI(TAG, "[UP  ] PWM=%4u us -> L:%u R:%u (expect:%u) %s", 
                     pwm, actual_L, actual_R, expected, 
                     (actual_L == expected && actual_R == expected) ? "✓" : "✗");
            vTaskDelay(pdMS_TO_TICKS(step_hold_ms - 30));
        }
        
        // 最高值保持3秒
        ESP_LOGI(TAG, "[MAX] 保持 2000 us (3s)");
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        // 下降阶段：2000 -> 1000 (100us/step)
        ESP_LOGI(TAG, "--- 下降阶段 ---");
        for (uint32_t pwm = pwm_max; pwm >= pwm_min; pwm -= pwm_step) {
            uint32_t expected = us_to_duty(pwm);
            
            // 直接操作LEDC，不使用互斥锁
            ledc_set_duty(LEDC_MODE, THRUST_LEFT_CHANNEL, expected);
            ledc_update_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
            ledc_set_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL, expected);
            ledc_update_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
            
            // 等待硬件生效
            vTaskDelay(pdMS_TO_TICKS(30));
            
            // 读取实际值验证
            uint32_t actual_L = ledc_get_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
            uint32_t actual_R = ledc_get_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
            
            ESP_LOGI(TAG, "[DOWN] PWM=%4u us -> L:%u R:%u (expect:%u) %s", 
                     pwm, actual_L, actual_R, expected, 
                     (actual_L == expected && actual_R == expected) ? "✓" : "✗");
            vTaskDelay(pdMS_TO_TICKS(step_hold_ms - 30));
        }
        
        // 回到中立位置
        ESP_LOGI(TAG, "[MID] 回到中立位置 1500 us (3s 休息)");
        ledc_set_duty(LEDC_MODE, THRUST_LEFT_CHANNEL, duty_1500);
        ledc_update_duty(LEDC_MODE, THRUST_LEFT_CHANNEL);
        ledc_set_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL, duty_1500);
        ledc_update_duty(LEDC_MODE, THRUST_RIGHT_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(3000));
        
        ESP_LOGW(TAG, "========== 扫描循环 #%u 完成 ==========\n", loop_count);
    }
}

