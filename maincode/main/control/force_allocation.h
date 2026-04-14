/**
 * @file force_allocation.h
 * @brief 力分配算法 - 根据推进器角度分配控制力
 * 
 * 当推进器旋转到角度 θ 时，将 PID 输出的控制力分配到
 * 左右推进器，以维持平衡。
 * 
 * 坐标系定义：
 * - θ = 0°：推进器水平向前  
 * - θ = 90°：推进器竖直向下
 * - θ = 180°：推进器水平向后
 */

#ifndef FORCE_ALLOCATION_H
#define FORCE_ALLOCATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "esp_err.h"

/* ========== 数据结构体 ========== */

/**
 * @brief 力分配计算器
 */
typedef struct {
    float current_angle;           ///< 当前推进器角度 (度)
    float target_angle;            ///< 目标推进器角度 (度)
    float base_pulse;              ///< 基础脉宽 (μs)
    float max_pulse;               ///< 最大脉宽 (μs)
    float min_pulse;               ///< 最小脉宽 (μs)
    float angle_rate_limit;        ///< 角度变化速率限制 (deg/s)
    float dt;                      ///< 时间步长 (秒)
    float correction_factor;       ///< 垂直力补偿因子 (0.0 - 1.0)
} force_allocator_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化力分配计算器
 * 
 * @param allocator 力分配器实例指针
 * @param base_pulse 基础脉宽 (μs)
 * @param min_pulse 最小脉宽 (μs)
 * @param max_pulse 最大脉宽 (μs)
 * @param rate_limit 角度变化速率限制 (deg/s)
 * @param dt 时间步长 (秒)
 * @return ESP_OK 成功，否则失败
 */
esp_err_t force_allocator_init(force_allocator_t *allocator, 
                               float base_pulse, float min_pulse, float max_pulse,
                               float rate_limit, float dt);

/**
 * @brief 设置目标推进器角度
 * 
 * 角度范围限制在 [0°, 180°]
 * 实际角度变化受速率限制
 * 
 * @param allocator 力分配器实例指针
 * @param target_angle 目标角度 (度)
 * @return ESP_OK 成功，否则失败
 */
esp_err_t force_allocator_set_target_angle(force_allocator_t *allocator, float target_angle);

/**
 * @brief 计算左右推进器脉宽
 * 
 * 根据当前推进器角度和PID输出，计算左右推进器所需的脉宽。
 * 自动处理角度速率限制。
 * 
 * 力分配方程：
 * - pulse_left  = base + pid_force * cos(θ) + correction * sin(θ)
 * - pulse_right = base - pid_force * cos(θ) - correction * sin(θ)
 * 
 * 其中 θ 相对于水平面 (θ - 90° 在标准圆周坐标中)
 * 
 * @param allocator 力分配器实例指针
 * @param pid_output PID 计算输出 (控制力)
 * @param[out] pulse_left 计算得到的左推进器脉宽 (μs)
 * @param[out] pulse_right 计算得到的右推进器脉宽 (μs)
 * @return ESP_OK 成功，否则失败
 */
esp_err_t force_allocator_compute(force_allocator_t *allocator, 
                                  float pid_output,
                                  uint16_t *pulse_left, uint16_t *pulse_right);

/**
 * @brief 获取当前推进器角度
 * 
 * @param allocator 力分配器实例指针
 * @return 当前推进器角度 (度)
 */
float force_allocator_get_current_angle(force_allocator_t *allocator);

/**
 * @brief 获取目标推进器角度
 * 
 * @param allocator 力分配器实例指针
 * @return 目标推进器角度 (度)
 */
float force_allocator_get_target_angle(force_allocator_t *allocator);

/**
 * @brief 设置垂直力补偿因子
 * 
 * 补偿因子用于调整推进器旋转时垂直力的变化。
 * 值域 [0.0, 1.0]，默认 0.0 表示不补偿。
 * 
 * @param allocator 力分配器实例指针
 * @param factor 补偿因子 [0.0, 1.0]
 */
void force_allocator_set_correction_factor(force_allocator_t *allocator, float factor);

/**
 * @brief 复位力分配器
 * 
 * 将当前角度重置为 90°（初始竖直向下状态）
 * 
 * @param allocator 力分配器实例指针
 */
void force_allocator_reset(force_allocator_t *allocator);

#ifdef __cplusplus
}
#endif

#endif // FORCE_ALLOCATION_H
