#include "motor_control.h"
#include "system_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
static inline uint32_t us_to_duty(uint32_t us) {
    // formula: us * ((2^14)/(1000000/50))
    // us * (16384 / 20000) = us * 16384 / 20000
    return (us * 16384) / 20000;
}

// 设置特定通道的 PWM 脉冲 (us)
static void set_pwm_us(ledc_channel_t channel, uint32_t us) {
    if (us < 1000) us = 1000;
    if (us > 2000) us = 2000;
    ledc_set_duty(LEDC_MODE, channel, us_to_duty(us));
    ledc_update_duty(LEDC_MODE, channel);
}

void motor_control_init(void) {
    // 1. 初始化定时器 (Timer 0)
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // 2. 初始化 4 个硬件通道
    ledc_channel_config_t ch_config[4] = {
        { .gpio_num = PIN_STEER_LEFT,   .speed_mode = LEDC_MODE, .channel = STEER_LEFT_CHANNEL,   .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 },
        { .gpio_num = PIN_STEER_RIGHT,  .speed_mode = LEDC_MODE, .channel = STEER_RIGHT_CHANNEL,  .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 },
        { .gpio_num = PIN_THRUST_LEFT,  .speed_mode = LEDC_MODE, .channel = THRUST_LEFT_CHANNEL,  .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 },
        { .gpio_num = PIN_THRUST_RIGHT, .speed_mode = LEDC_MODE, .channel = THRUST_RIGHT_CHANNEL, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER, .duty = us_to_duty(1500), .hpoint = 0 }
    };

    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ch_config[i]));
    }
    
    ESP_LOGI(TAG, "Motor Control Initialized. All pulses set to 1500us!");
}

void motor_control_set_steering_pwm(uint32_t pwm_left_us, uint32_t pwm_right_us) {
#if CURRENT_RUN_MODE == MODE_TEST_SENSORS || CURRENT_RUN_MODE == MODE_TEST_BALANCE_ONLY
    // 如果系统不需要开转向功能，锁定为 1500 保持正冲下方
    set_pwm_us(STEER_LEFT_CHANNEL, 1500);
    set_pwm_us(STEER_RIGHT_CHANNEL, 1500);
#else
    set_pwm_us(STEER_LEFT_CHANNEL, pwm_left_us);
    set_pwm_us(STEER_RIGHT_CHANNEL, pwm_right_us);
#endif
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

    set_pwm_us(THRUST_LEFT_CHANNEL, thrust_L);
    set_pwm_us(THRUST_RIGHT_CHANNEL, thrust_R);
}

void motor_control_emergency_stop(void) {
    static uint32_t last_print_time = 0;
    if (xTaskGetTickCount() - last_print_time > pdMS_TO_TICKS(1000)) {
        ESP_LOGW(TAG, "EMERGENCY THRUST STOP TRIGGERED! (Testing Mode or Safety Limit)");
        last_print_time = xTaskGetTickCount();
    }
    // 紧急状态下，仅停下大电机即主推进器！！
    // 千万不要重置小电机PWM（STEER_XX_CHANNEL），否则会导致无法单独测试转向系统
    set_pwm_us(THRUST_LEFT_CHANNEL, 1500);
    set_pwm_us(THRUST_RIGHT_CHANNEL, 1500);
}

// ======================= 大电机 (ESC) 校准专有序列 =======================
// 仅仅在 MODE_CALIBRATE_ESC 模式下触发执行
void motor_control_esc_calibrate_task(void *pvParameters) {
    ESP_LOGW(TAG, "=========== ESC (大电机) 校准模式启动 ===========");
    ESP_LOGW(TAG, "请确保大电机电调已经上电，如果电调需要全油门上电再降最低油门校准，请按电调要求操作。");
    ESP_LOGW(TAG, "我们将开始一个慢速模拟量清扫: 1000us -> 2000us -> 1000us");

    // 初始位置 1000
    set_pwm_us(THRUST_LEFT_CHANNEL, 1000);
    set_pwm_us(THRUST_RIGHT_CHANNEL, 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 逐渐上升到 2000
    ESP_LOGI(TAG, ">> 推力渐增 (1000 -> 2000)");
    for (int pwm = 1000; pwm <= 2000; pwm += 10) {
        set_pwm_us(THRUST_LEFT_CHANNEL, pwm);
        set_pwm_us(THRUST_RIGHT_CHANNEL, pwm);
        printf("\r[校准中] 主推力 PWM: %d us", pwm);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
    printf("\n");
    
    // 在最高点保持一会儿
    ESP_LOGI(TAG, ">> 暂留最高油门 2000us (时长 2s)");
    vTaskDelay(pdMS_TO_TICKS(2000));

    // 逐渐下降到 1000
    ESP_LOGI(TAG, ">> 推力渐降 (2000 -> 1000)");
    for (int pwm = 2000; pwm >= 1000; pwm -= 10) {
        set_pwm_us(THRUST_LEFT_CHANNEL, pwm);
        set_pwm_us(THRUST_RIGHT_CHANNEL, pwm);
        printf("\r[校准中] 主推力 PWM: %d us", pwm);
        fflush(stdout);
        vTaskDelay(pdMS_TO_TICKS(50)); 
    }
    printf("\n");
    
    ESP_LOGW(TAG, "=========== ESC 校准序列完成，推力保持 1000us ===========");

    while(1) {
        // 挂起自身，死循环保护
        set_pwm_us(THRUST_LEFT_CHANNEL, 1000);
        set_pwm_us(THRUST_RIGHT_CHANNEL, 1000);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

