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

// PID 状态
static float integral_left = 0.0f, integral_right = 0.0f;
static float prev_err_left = 0.0f, prev_err_right = 0.0f;
static float out_filt_left = 0.0f, out_filt_right = 0.0f;

// --- ISR 外部中断处理函数 ---
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    encoder_state_t* st = (encoder_state_t*) arg;
    int level = gpio_get_level(st->pin);
    int64_t now = esp_timer_get_time();
    int64_t delta = now - st->last_edge_time;

    if (delta < 10) return; // 10us 毛刺滤波

    if (level == 0) {
        // 下降沿：记录高电平时间
        if (delta >= 10) {
            st->high_us = (uint32_t)delta;
        }
    } else {
        // 上升沿：记录整个周期时间
        if (delta >= 200 && delta <= 200000) { 
            st->period_us = (uint32_t)delta;
            st->valid = true;
        } else {
            st->valid = false;
        }
    }
    st->last_edge_time = now;
}

// 占空比转角度 (0~360)
// 注意：period_us 存储的是低电平时间，周期 = high_us + low_us
static float compute_angle(volatile uint32_t high_us, volatile uint32_t period_us, volatile bool valid) {
    if (!valid || period_us == 0) return 0.0f;
    
    // 正确计算占空比：高电平 / (高电平 + 低电平)
    uint32_t total_period = high_us + period_us;
    float duty = ((float)high_us / (float)total_period) * 4119.0f - 16.0f;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 4095.0f) duty = 4095.0f;
    return duty * (360.0f / 4095.0f);
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
    
    ESP_LOGI(TAG, "Steering Encoder Interrupts Initialized.");
}

void steering_control_set_target(float target_left_deg, float target_right_deg) {
    target_left = target_left_deg;
    target_right = target_right_deg;
}

void steering_control_calibrate_encoders(void) {
    // 读取当前编码器的竖直状态值作为校准基准
    float cal_left = compute_angle(enc_left.high_us, enc_left.period_us, enc_left.valid);
    float cal_right = compute_angle(enc_right.high_us, enc_right.period_us, enc_right.valid);
    
    // 设置偏移使得初始竖直状态对应 180°（避免 0/360 边界抖动）
    offset_left = cal_left - 180.0f;
    offset_right = cal_right - 180.0f;
    
    ESP_LOGI(TAG, "Encoder Calibration Complete. Offset Left: %.1f°, Offset Right: %.1f°", offset_left, offset_right);
}

void steering_control_get_current_angles(float *left_deg, float *right_deg) {
    float raw_left = compute_angle(enc_left.high_us, enc_left.period_us, enc_left.valid);
    float raw_right = compute_angle(enc_right.high_us, enc_right.period_us, enc_right.valid);
    
    // 相对于初始校准点的角度
    *left_deg = raw_left - offset_left;
    *right_deg = raw_right - offset_right;
    
    // 将角度归一化到 [0, 360) 范围（保证所有值为正数）
    while (*left_deg < 0.0f) *left_deg += 360.0f;
    while (*left_deg >= 360.0f) *left_deg -= 360.0f;
    while (*right_deg < 0.0f) *right_deg += 360.0f;
    while (*right_deg >= 360.0f) *right_deg -= 360.0f;
}

// 提取单边 PID 计算
static float calculate_pid(float error, float *integral, float *prev_error, float *out_filt) {
    const float kp = 6.8f, ki = 1.0f, kd = 0.49f, dt = 0.01f;
    const float deadband = 3.0f, blend = 3.0f;
    const float integral_max = 120.0f;

    // 死区与平滑
    float abs_err = fabsf(error);
    float err_smooth = error;
    if (abs_err <= deadband) {
        err_smooth = 0.0f;
        // 只在非常接近目标时清零积分，避免启动缓慢
        if (abs_err < 1.0f) {
            *integral = 0.0f;
        }
    } else if (abs_err < deadband + blend) {
        float t = (abs_err - deadband) / blend;
        float factor = t * t * (3.0f - 2.0f * t);
        err_smooth = (error > 0 ? 1.0f : -1.0f) * (abs_err - deadband) * factor;
    }

    // P 项
    float p_out = kp * err_smooth;

    // I 项（积分前限幅，避免饱和后不能回退）
    float i_candidate = *integral + err_smooth * dt * ki;
    if (i_candidate > integral_max) {
        i_candidate = integral_max;
    } else if (i_candidate < -integral_max) {
        i_candidate = -integral_max;
    }
    *integral = i_candidate;
    float i_out = *integral;

    // D 项（使用原始误差而不是平滑后的误差，以便更快响应）
    // 但第一次调用时，prev_error可能为0，会导致尖刺，所以用有限差分
    float derivative = 0.0f;
    if (dt > 0) {
        derivative = (error - *prev_error) / dt;
        // 对微分项进行限幅，避免尖刺
        if (derivative > 200.0f) derivative = 200.0f;
        if (derivative < -200.0f) derivative = -200.0f;
    }
    *prev_error = error;
    float d_out = kd * derivative;

    float raw_out = p_out + i_out + d_out;

    // 低通滤波 0.12s
    float alpha = dt / (0.12f + dt);
    *out_filt = *out_filt + alpha * (raw_out - *out_filt);

    // 限幅控制量 +/- 80 (对应中立1500 +/-80)
    float final_out = *out_filt;
    if (final_out > 80.0f) final_out = 80.0f;
    if (final_out < -80.0f) final_out = -80.0f;

    return final_out;
}

void steering_control_update(void) {
    float cur_left, cur_right;
    steering_control_get_current_angles(&cur_left, &cur_right);

    float err_L = shortest_angle_error(target_left, cur_left);
    float err_R = shortest_angle_error(target_right, cur_right);

    float adjust_L = calculate_pid(err_L, &integral_left, &prev_err_left, &out_filt_left);
    float adjust_R = calculate_pid(err_R, &integral_right, &prev_err_right, &out_filt_right);

    // 如果方向相反（代码中左+1右-1映射）
    uint32_t pwm_L = (uint32_t)(1500.0f + adjust_L);
    uint32_t pwm_R = (uint32_t)(1500.0f - adjust_R);

    motor_control_set_steering_pwm(pwm_L, pwm_R);
}