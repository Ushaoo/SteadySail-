/**
 * @file pid_controller.h
 * @brief 2自由度 PID 控制器
 * 
 * 参考树莓派版本: feedforward_dual_imu.py - PID2DOF 类
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

/* ========== 数据结构体 ========== */

/**
 * @brief 2DOF PID 控制器
 */
typedef struct {
    // PID 增益
    float Kp;       ///< 比例增益
    float Ki;       ///< 积分增益
    float Kd;       ///< 微分增益
    
    // 2DOF 权重
    float b;        ///< 比例权重 (setpoint weighting)
    float c;        ///< 微分权重 (derivative filtering)
    
    float dt;       ///< 时间步长 (秒)
    float integral; ///< 积分累计值
    
} pid_controller_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 PID 控制器
 * 
 * @param pid PID 实例指针
 * @param Kp 比例增益
 * @param Ki 积分增益
 * @param Kd 微分增益
 * @param b 比例权重 (通常 0.8)
 * @param c 微分权重 (通常 0.0)
 * @param dt 时间步长 (秒)
 */
void pid_init(pid_controller_t *pid, float Kp, float Ki, float Kd, float b, float c, float dt);

/**
 * @brief 更新 PID 控制器，计算输出
 * 
 * 2自由度 PID 算法:
 * - 比例项: Kp * (b*setpoint - measured)
 * - 积分项: Ki * integral(error)
 * - 微分项: Kd * (c*setpoint_derivative - measured_derivative)
 * 
 * @param pid PID 实例指针
 * @param setpoint 目标值 (度)
 * @param measured 测量值 (度)
 * @param omega_filtered 角速度 (deg/s，用于微分项)
 * @return PID 输出 (控制力矩，单位: N·m 或无量纲)
 */
float pid_update(pid_controller_t *pid, float setpoint, float measured, float omega_filtered);

/**
 * @brief 动态更新 PID 增益
 * 
 * @param pid PID 实例指针
 * @param Kp 新的比例增益
 * @param Ki 新的积分增益
 * @param Kd 新的微分增益
 */
void pid_update_gains(pid_controller_t *pid, float Kp, float Ki, float Kd);

/**
 * @brief 复位 PID 控制器 (清除积分累计)
 * 
 * @param pid PID 实例指针
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief 获取积分项 (用于调试)
 * 
 * @param pid PID 实例指针
 * @return 当前积分累计值
 */
float pid_get_integral(pid_controller_t *pid);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
