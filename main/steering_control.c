#include "steering_control.h"
#include "motor_control.h"
#include "system_config.h"
#include "control_params.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>

static const char *TAG = "STEERING";

// NVS namespace / key
#define STEER_NVS_NS       "steering"
#define STEER_NVS_KEY_OFFL "off_l"
#define STEER_NVS_KEY_OFFR "off_r"

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

// 是否已经完成"竖直 → 180°"校准（NVS 加载成功 或 用户手动触发过）
static bool s_calibrated = false;

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
// MT6826S 编码器：占空比 ~1%~99% 对应 0~360°（数据手册端点附近留有最小高/低脉宽保护）
static float compute_angle(volatile uint32_t high_us, volatile uint32_t period_us, volatile bool valid) {
    if (!valid || period_us == 0) return 0.0f;
    
    uint32_t total_period = high_us + period_us;
    float duty_cycle = (float)high_us / (float)total_period;
    
    // 端点放宽到 MT6826S 真实范围：1% -> 0°, 99% -> 360°
    const float DC_MIN = 0.01f;
    const float DC_MAX = 0.99f;
    const float RANGE = DC_MAX - DC_MIN;  // 98%
    
    float angle = (duty_cycle - DC_MIN) / RANGE * 360.0f;
    
    // 端点饱和（clamp，不再 wrap）：避免边缘抖动跨 0/360 翻转
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 360.0f) angle = 360.0f;
    
    return angle;
}

// 最短路径环形误差计算
static float shortest_angle_error(float target, float current) {
    float err = target - current;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

// ---------- NVS 校准存取 ----------
static esp_err_t nvs_load_offsets(float *off_l, float *off_r) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(STEER_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    union { uint32_t u; float f; } cvt_l, cvt_r;
    err = nvs_get_u32(h, STEER_NVS_KEY_OFFL, &cvt_l.u);
    if (err == ESP_OK) {
        err = nvs_get_u32(h, STEER_NVS_KEY_OFFR, &cvt_r.u);
    }
    nvs_close(h);
    if (err != ESP_OK) return err;

    *off_l = cvt_l.f;
    *off_r = cvt_r.f;
    return ESP_OK;
}

static esp_err_t nvs_save_offsets(float off_l, float off_r) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(STEER_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    union { uint32_t u; float f; } cvt_l = { .f = off_l }, cvt_r = { .f = off_r };
    err = nvs_set_u32(h, STEER_NVS_KEY_OFFL, cvt_l.u);
    if (err == ESP_OK) err = nvs_set_u32(h, STEER_NVS_KEY_OFFR, cvt_r.u);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
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

    // 尝试从 NVS 加载上次保存的零点偏移
    float loaded_l = 0.0f, loaded_r = 0.0f;
    if (nvs_load_offsets(&loaded_l, &loaded_r) == ESP_OK) {
        offset_left  = loaded_l;
        offset_right = loaded_r;
        filtered_angle_left  = 180.0f;
        filtered_angle_right = 180.0f;
        s_calibrated = true;
        ESP_LOGI(TAG, "✓ 已从 NVS 加载校准: offset_L=%.2f° offset_R=%.2f°", offset_left, offset_right);
    } else {
        s_calibrated = false;
        ESP_LOGW(TAG, "⚠ 未发现校准数据，舵机已锁定 PWM=1500。");
        ESP_LOGW(TAG, "  请把舵机摆到正下方（180° 竖直）并发送 'cal' 命令完成首次校准。");
    }
}

void steering_control_set_target(float target_left_deg, float target_right_deg) {
    target_left = 360.0f - target_left_deg;
    target_right = 360.0f - target_right_deg;
}

void steering_control_calibrate_and_save(void) {
    // 读取当前编码器的竖直状态值作为校准基准
    float cal_left  = compute_angle(enc_left.high_us,  enc_left.period_us,  enc_left.valid);
    float cal_right = compute_angle(enc_right.high_us, enc_right.period_us, enc_right.valid);

    // 设置偏移使得初始竖直状态对应 180°（避免 0/360 边界抖动）
    offset_left  = cal_left  - 180.0f;
    offset_right = cal_right - 180.0f;

    // 初始化滤波缓存
    filtered_angle_left  = 180.0f;
    filtered_angle_right = 180.0f;

    s_calibrated = true;
    ESP_LOGI(TAG, "Encoder Calibration Complete. Offset Left: %.2f°, Offset Right: %.2f°", offset_left, offset_right);

    esp_err_t err = nvs_save_offsets(offset_left, offset_right);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✓ 校准已保存到 NVS（重启后自动加载，无需再次校准）");
    } else {
        ESP_LOGE(TAG, "✗ 保存校准到 NVS 失败: %s", esp_err_to_name(err));
    }
}

bool steering_control_is_calibrated(void) {
    return s_calibrated;
}

void steering_control_get_current_angles(float *left_deg, float *right_deg) {
    float raw_left = compute_angle(enc_left.high_us, enc_left.period_us, enc_left.valid);
    float raw_right = compute_angle(enc_right.high_us, enc_right.period_us, enc_right.valid);
    
    // 相对于初始校准点的角度
    raw_left = raw_left - offset_left;
    raw_right = raw_right - offset_right;

    // 应用编码器方向反转开关（由 system_config.h 集中配置）
    // 以 180° 为镜像中心，源于校准后初始位是 180°
#if ENC_LEFT_REVERSE
    raw_left = 360.0f - raw_left;
#endif
#if ENC_RIGHT_REVERSE
    raw_right = 360.0f - raw_right;
#endif

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
    // PID 增益从全局变量读取（可通过 Blinker 实时调参）
    const float kp = g_steer_kp, ki = g_steer_ki, kd = g_steer_kd, dt = 0.01f;
    const float integral_max = 80.0f;

    // 注：外层 steering_control_update() 已经用 DEADZONE=3° 做了死区门控，
    // 这里不再叠加 smoothstep，避免 4°~6° 小误差被双重削弱后落到 0。
    float err_smooth = error;

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
    if (final_out > 500.0f) final_out = 500.0f;
    if (final_out < -500.0f) final_out = -500.0f;
    
    // 输出阈值：避免小于阈值的信号导致电机微弱运转
    if (fabsf(final_out) < 30.0f) {
        final_out = 0.0f;
    } else if (fabsf(final_out) < 60.0f) {        // 提高到 60
        final_out = (final_out > 0) ? 60.0f : -60.0f;
    }
    

    return final_out;
}

// PID 状态
static float integral_left = 0.0f, integral_right = 0.0f;
static float prev_err_left = 0.0f, prev_err_right = 0.0f;
static float out_filt_left = 0.0f, out_filt_right = 0.0f;

void steering_control_update(void) {
    // 未校准 → 强制中立位 PWM=1500（最安全：360° 连续舵机此时不旋转）
    if (!s_calibrated) {
        motor_control_set_steering_pwm(1500, 1500);
        return;
    }

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