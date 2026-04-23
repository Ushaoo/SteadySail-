#include "steering_control.h"
#include "motor_control.h"
#include "system_config.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "STEERING";

// --- 编码器硬件捕获 (MT6826S PWM 模式) ---
typedef struct {
    uint32_t pin;
    volatile int64_t last_edge_time;
    volatile uint32_t high_us;
    volatile uint32_t period_us;
    volatile bool valid;
} encoder_state_t;

static encoder_state_t enc_left  = { .pin = PIN_ENC_LEFT, .valid = false };
static encoder_state_t enc_right = { .pin = PIN_ENC_RIGHT, .valid = false };

static float target_left = 180.0f;
static float target_right = 180.0f;

// 编码器初始校准偏移值
static float offset_left = 0.0f;
static float offset_right = 0.0f;

// 编码器滤波缓存（简单直通）
static float filtered_angle_left = 180.0f;
static float filtered_angle_right = 180.0f;

// --- ISR 外部中断处理函数 ---
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    encoder_state_t* st = (encoder_state_t*) arg;
    int level = gpio_get_level(st->pin);
    int64_t now = esp_timer_get_time();
    int64_t delta = now - st->last_edge_time;

    if (delta < 5) return; // 5us 短期毛刺滤波（放开原先的限制）

    if (level == 0) {
        // 下降沿：此时的 delta 是高电平时间
        st->high_us = (uint32_t)delta;
    } else {
        // 上升沿：此时的 delta 是低电平时间（这里利用 period_us 变量存放 low_us）
        st->period_us = (uint32_t)delta;
        
        // 校验整个 PWM 周期（高电平 + 低电平）以确认是一次有效读取
        // MT6826 的 PWM 周期通常在 1000us 左右，放宽为 100~50000us 防止特定角度的正常占空比被误拦截
        uint32_t total = st->high_us + st->period_us;
        if (total >= 100 && total <= 50000) { 
            st->valid = true;
        } else {
            st->valid = false;
        }
    }
    st->last_edge_time = now;
}

// 占空比转角度 (0~360)
// MT6826S 编码器：占空比范围 5-95% 对应 0-360°
static float compute_angle(volatile uint32_t high_us, volatile uint32_t period_us, volatile bool valid) {
    if (!valid || period_us == 0) return 0.0f;
    
    uint32_t total_period = high_us + period_us;
    float duty_cycle = (float)high_us / (float)total_period;
    
    // MT6826 占空比范围映射: 5% -> 0°, 95% -> 360°
    // angle = (duty - 0.05) / 0.90 * 360 = (duty - 0.05) * 400
    const float DC_MIN = 0.05f;  // 5%
    const float DC_MAX = 0.95f;  // 95%
    const float RANGE = DC_MAX - DC_MIN;  // 90%
    
    float angle = (duty_cycle - DC_MIN) / RANGE * 360.0f;
    
    // 角度范围限制到 [0, 360)
    while (angle < 0.0f) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    
    return angle;
}

// 最短路径环形误差计算
static float shortest_angle_error(float target, float current) {
    float err = target - current;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

void steering_control_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_ENC_LEFT) | (1ULL << PIN_ENC_RIGHT),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(PIN_ENC_LEFT, encoder_isr_handler, (void*)&enc_left);
    gpio_isr_handler_add(PIN_ENC_RIGHT, encoder_isr_handler, (void*)&enc_right);
    
    ESP_LOGI(TAG, "✓ Steering Encoder Interrupts Initialized.");
    ESP_LOGI(TAG, "  - Left Encoder GPIO: %d | Right Encoder GPIO: %d", PIN_ENC_LEFT, PIN_ENC_RIGHT);
    ESP_LOGI(TAG, "  - 启动后自动校准编码器（保持舵机竖直向下）");
}

void steering_control_set_target(float target_left_deg, float target_right_deg) {
    target_left = 360.0f - target_left_deg;
    target_right = target_right_deg;
}

void steering_control_calibrate_encoders(void) {
    // 读取当前编码器的竖直状态值作为校准基准
    float cal_left = compute_angle(enc_left.high_us, enc_left.period_us, enc_left.valid);
    float cal_right = compute_angle(enc_right.high_us, enc_right.period_us, enc_right.valid);
    
    // 设置偏移使得初始竖直状态对应 180°（避免 0/360 边界抖动）
    offset_left = cal_left - 180.0f;
    offset_right = cal_right - 180.0f;
    
    // 初始化滤波缓存
    filtered_angle_left = 180.0f;
    filtered_angle_right = 180.0f;
    
    ESP_LOGI(TAG, "Encoder Calibration Complete. Offset Left: %.1f°, Offset Right: %.1f°", offset_left, offset_right);
}

void steering_control_get_current_angles(float *left_deg, float *right_deg) {
    float raw_left = compute_angle(enc_left.high_us, enc_left.period_us, enc_left.valid);
    float raw_right = compute_angle(enc_right.high_us, enc_right.period_us, enc_right.valid);
    
    // 相对于初始校准点的角度
    raw_left = raw_left - offset_left;
    raw_right = raw_right - offset_right;
    
    // 将原始角度归一化到 [0, 360) 范围
    while (raw_left < 0.0f) raw_left += 360.0f;
    while (raw_left >= 360.0f) raw_left -= 360.0f;
    while (raw_right < 0.0f) raw_right += 360.0f;
    while (raw_right >= 360.0f) raw_right -= 360.0f;
    
    *left_deg = raw_left;
    *right_deg = raw_right;
}

// 获取编码器健康状态
void steering_control_get_encoder_status(bool *left_ok, bool *right_ok) {
    *left_ok = enc_left.valid;
    *right_ok = enc_right.valid;
}

// 提取单边 PID 计算
static float calculate_pid(float error, float *integral, float *prev_error, float *out_filt) {
    const float kp = 5.0f, ki = 1.0f, kd = 0.49f, dt = 0.01f;
    const float deadband = 3.0f, blend = 3.0f;
    const float integral_max = 80.0f;

    // 死区与平滑
    float abs_err = fabsf(error);
    float err_smooth = error;
    if (abs_err <= deadband) {
        err_smooth = 0.0f;
        *integral = 0.0f;
    } else if (abs_err < deadband + blend) {
        float t = (abs_err - deadband) / blend;
        float factor = t * t * (3.0f - 2.0f * t);
        err_smooth = (error > 0 ? 1.0f : -1.0f) * (abs_err - deadband) * factor;
    }

    // P 项
    float p_out = kp * err_smooth;

    // I 项
    float i_candidate = *integral + err_smooth * dt * ki;
    if (i_candidate > integral_max) {
        i_candidate = integral_max;
    } else if (i_candidate < -integral_max) {
        i_candidate = -integral_max;
    }
    *integral = i_candidate;
    float i_out = *integral;

    // D 项
    float derivative = 0.0f;
    if (dt > 0) {
        derivative = (error - *prev_error) / dt;
        if (derivative > 200.0f) derivative = 200.0f;
        if (derivative < -200.0f) derivative = -200.0f;
    }
    *prev_error = error;
    float d_out = kd * derivative;

    float raw_out = p_out + i_out + d_out;

    // 低通滤波
    float alpha = dt / (0.12f + dt);
    *out_filt = *out_filt + alpha * (raw_out - *out_filt);

    float final_out = *out_filt;
    if (final_out > 150.0f) final_out = 150.0f;
    if (final_out < -150.0f) final_out = -150.0f;
    
    // 输出阈值：避免小于25的信号导致电机微弱运转
    if (fabsf(final_out) < 20.0f) {
        final_out = 0.0f;
    }else if (fabsf(final_out) < 30.0f && fabsf(final_out) >= 20.0f) {
        if (final_out > 0) {
            final_out = 30.0f;
        } else {
            final_out = -30.0f;
        }
    }
    

    return final_out;
}

// PID 状态
static float integral_left = 0.0f, integral_right = 0.0f;
static float prev_err_left = 0.0f, prev_err_right = 0.0f;
static float out_filt_left = 0.0f, out_filt_right = 0.0f;

void steering_control_update(void) {
    float cur_left, cur_right;
    steering_control_get_current_angles(&cur_left, &cur_right);

    float err_L = shortest_angle_error(target_left, cur_left);
    float err_R = shortest_angle_error(target_right, cur_right);

    const float DEADZONE = 3.0f;
    float adjust_L = 0.0f, adjust_R = 0.0f;

#if STEERING_CONTROL_MODE == STEERING_MODE_PID
    // PID 模式
    if (fabsf(err_L) > DEADZONE) {
        adjust_L = calculate_pid(err_L, &integral_left, &prev_err_left, &out_filt_left);
    } else {
        adjust_L = 0.0f;
        integral_left = 0.0f;
    }
    
    if (fabsf(err_R) > DEADZONE) {
        adjust_R = calculate_pid(err_R, &integral_right, &prev_err_right, &out_filt_right);
    } else {
        adjust_R = 0.0f;
        integral_right = 0.0f;
    }
#else
    // 直接映射模式（中等响应速度）
    if (fabsf(err_L) > DEADZONE) {
        // 降低比例增益与最大转速限幅
        // PWM 范围限制在 1500 ± 150 (即 1200 到 1800)
        adjust_L = err_L * 5.0f; 
        if (adjust_L > 100.0f) adjust_L = 100.0f;
        if (adjust_L < -100.0f) adjust_L = -100.0f;
    }
    
    if (fabsf(err_R) > DEADZONE) {
        adjust_R = err_R * 5.0f;
        if (adjust_R > 100.0f) adjust_R = 100.0f;
        if (adjust_R < -100.0f) adjust_R = -100.0f;
    }
#endif

    uint32_t pwm_L = (uint32_t)(1500.0f - adjust_L);
    uint32_t pwm_R = (uint32_t)(1500.0f - adjust_R);

    motor_control_set_steering_pwm(pwm_L, pwm_R);
}