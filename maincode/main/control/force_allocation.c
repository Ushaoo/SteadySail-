/**
 * @file force_allocation.c
 * @brief 力分配算法实现
 */

#include "force_allocation.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "FORCE_ALLOC";

#define PI 3.14159265359f
#define DEG_TO_RAD(x) ((x) * PI / 180.0f)

/* ========== 初始化函数 ========== */

esp_err_t force_allocator_init(force_allocator_t *allocator, 
                               float base_pulse, float min_pulse, float max_pulse,
                               float rate_limit, float dt)
{
    if (allocator == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(allocator, 0, sizeof(force_allocator_t));

    allocator->current_angle = 90.0f;      // 初始竖直向下
    allocator->target_angle = 90.0f;
    allocator->base_pulse = base_pulse;
    allocator->min_pulse = min_pulse;
    allocator->max_pulse = max_pulse;
    allocator->angle_rate_limit = rate_limit;
    allocator->dt = dt;
    allocator->correction_factor = 0.0f;   // 默认无补偿

    ESP_LOGI(TAG, "Force allocator initialized: base=%.0f, min=%.0f, max=%.0f, rate_limit=%.1f deg/s",
             base_pulse, min_pulse, max_pulse, rate_limit);

    return ESP_OK;
}

/* ========== 目标角度设置 ========== */

esp_err_t force_allocator_set_target_angle(force_allocator_t *allocator, float target_angle)
{
    if (allocator == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 限制在 [0°, 180°]
    if (target_angle < 0.0f) target_angle = 0.0f;
    if (target_angle > 180.0f) target_angle = 180.0f;

    allocator->target_angle = target_angle;

    ESP_LOGI(TAG, "Target angle set to %.1f°", target_angle);

    return ESP_OK;
}

/* ========== 角度速率限制 ========== */

static void apply_angle_rate_limit(force_allocator_t *allocator)
{
    if (allocator == NULL) {
        return;
    }

    float angle_diff = allocator->target_angle - allocator->current_angle;
    float max_angle_delta = allocator->angle_rate_limit * allocator->dt;

    // 限制角度变化速率
    if (angle_diff > max_angle_delta) {
        allocator->current_angle += max_angle_delta;
    } else if (angle_diff < -max_angle_delta) {
        allocator->current_angle -= max_angle_delta;
    } else {
        allocator->current_angle = allocator->target_angle;
    }

    // 确保在有效范围内
    if (allocator->current_angle < 0.0f) allocator->current_angle = 0.0f;
    if (allocator->current_angle > 180.0f) allocator->current_angle = 180.0f;
}

/* ========== 力分配计算 ========== */

esp_err_t force_allocator_compute(force_allocator_t *allocator, 
                                  float pid_output,
                                  uint16_t *pulse_left, uint16_t *pulse_right)
{
    if (allocator == NULL || pulse_left == NULL || pulse_right == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 应用角度速率限制（平滑过渡）
    apply_angle_rate_limit(allocator);

    float theta = allocator->current_angle;

    // 转换到标准坐标系（相对于水平面）
    // 用户定义：90°=向下，0°=向前，180°=向后
    // 转换为：cos(θ-90°) = sin(θ), sin(θ-90°) = -cos(θ)
    float theta_rad = DEG_TO_RAD(theta);
    float cos_component = sinf(theta_rad);      // cos(θ-90°) = sin(θ)
    float sin_component = -cosf(theta_rad);     // sin(θ-90°) = -cos(θ)

    // 力分配方程（选项1 - 简单补偿）
    // pulse_left  = base + pid_output * cos_component + correction * sin_component
    // pulse_right = base - pid_output * cos_component - correction * sin_component
    
    float correction = pid_output * allocator->correction_factor * sin_component;

    float pulse_left_f = allocator->base_pulse + pid_output * cos_component + correction;
    float pulse_right_f = allocator->base_pulse - pid_output * cos_component - correction;

    // 限制脉宽范围
    if (pulse_left_f < allocator->min_pulse) pulse_left_f = allocator->min_pulse;
    if (pulse_left_f > allocator->max_pulse) pulse_left_f = allocator->max_pulse;
    if (pulse_right_f < allocator->min_pulse) pulse_right_f = allocator->min_pulse;
    if (pulse_right_f > allocator->max_pulse) pulse_right_f = allocator->max_pulse;

    *pulse_left = (uint16_t)(pulse_left_f + 0.5f);
    *pulse_right = (uint16_t)(pulse_right_f + 0.5f);

    return ESP_OK;
}

/* ========== 查询函数 ========== */

float force_allocator_get_current_angle(force_allocator_t *allocator)
{
    if (allocator == NULL) {
        return 90.0f;
    }
    return allocator->current_angle;
}

float force_allocator_get_target_angle(force_allocator_t *allocator)
{
    if (allocator == NULL) {
        return 90.0f;
    }
    return allocator->target_angle;
}

/* ========== 补偿因子设置 ========== */

void force_allocator_set_correction_factor(force_allocator_t *allocator, float factor)
{
    if (allocator == NULL) {
        return;
    }

    if (factor < 0.0f) factor = 0.0f;
    if (factor > 1.0f) factor = 1.0f;

    allocator->correction_factor = factor;

    ESP_LOGI(TAG, "Correction factor set to %.2f", factor);
}

/* ========== 复位函数 ========== */

void force_allocator_reset(force_allocator_t *allocator)
{
    if (allocator == NULL) {
        return;
    }

    allocator->current_angle = 90.0f;
    allocator->target_angle = 90.0f;

    ESP_LOGI(TAG, "Force allocator reset to 90°");
}
