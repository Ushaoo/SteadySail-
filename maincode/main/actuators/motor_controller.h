/**
 * @file motor_controller.h
 * @brief 电机控制逻辑
 * 
 * 包括:
 * - 推进器控制 (差分推进)
 * - 旋转舵机控制
 * - 安全约束
 */

#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdint.h>
#include "pca9685_driver.h"

/* ========== 数据结构体 ========== */

/**
 * @brief 电机控制器实例
 */
typedef struct {
    pca9685_t *pwm_driver;          ///< PCA9685 PWM 驱动指针
    
    // 推进器通道配置
    uint8_t left_thruster_ch;       ///< 左推进器通道
    uint8_t right_thruster_ch;      ///< 右推进器通道
    int left_thruster_invert;       ///< 左推进器脉宽是否反转
    int right_thruster_invert;      ///< 右推进器脉宽是否反转
    
    // 旋转舵机通道配置
    uint8_t left_rotation_ch;       ///< 左旋转舵机通道
    uint8_t right_rotation_ch;      ///< 右旋转舵机通道
    
    // 当前状态
    uint16_t last_pulse_left;       ///< 最后设置的左推进器脉宽
    uint16_t last_pulse_right;      ///< 最后设置的右推进器脉宽
    float last_rot_angle_left;      ///< 最后设置的左旋转角
    float last_rot_angle_right;     ///< 最后设置的右旋转角
    
    // 错误计数
    uint32_t error_count;           ///< I2C 错误计数
    
} motor_controller_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化电机控制器
 * 
 * @param motor 电机控制器实例指针
 * @param pwm_driver PCA9685 PWM 驱动指针
 * @param left_thruster_ch 左推进器通道
 * @param right_thruster_ch 右推进器通道
 * @param left_rotation_ch 左旋转舵机通道
 * @param right_rotation_ch 右旋转舵机通道
 * @return ESP_OK 表示成功
 */
esp_err_t motor_init(motor_controller_t *motor, pca9685_t *pwm_driver,
                     uint8_t left_thruster_ch, uint8_t right_thruster_ch,
                     uint8_t left_rotation_ch, uint8_t right_rotation_ch);

/**
 * @brief 设置推进器脉宽 (同时控制左右)
 * 
 * @param motor 电机控制器实例指针
 * @param pulse_left 左推进器脉宽 (1000-2000 μs)
 * @param pulse_right 右推进器脉宽 (1000-2000 μs)
 * @return ESP_OK 表示成功
 */
esp_err_t motor_set_thrusters(motor_controller_t *motor, uint16_t pulse_left, uint16_t pulse_right);

/**
 * @brief 设置推进器旋转角度
 * 
 * @param motor 电机控制器实例指针
 * @param angle_left 左旋转舵机角度 (-45 ~ 45 度)
 * @param angle_right 右旋转舵机角度 (-45 ~ 45 度)
 * @return ESP_OK 表示成功
 */
esp_err_t motor_set_rotation_angles(motor_controller_t *motor, float angle_left, float angle_right);

/**
 * @brief 紧急停止 (设置所有电机为中立)
 * 
 * @param motor 电机控制器实例指针
 * @return ESP_OK 表示成功
 */
esp_err_t motor_emergency_stop(motor_controller_t *motor);

/**
 * @brief 获取控制器状态
 * 
 * @param motor 电机控制器实例指针
 * @return 错误计数
 */
uint32_t motor_get_error_count(motor_controller_t *motor);

/**
 * @brief 角度转脉宽 (用于旋转舵机)
 * 
 * 将旋转角度转换为 PWM 脉宽
 * 
 * @param angle 旋转角度 (度，-45 ~ 45)
 * @param min_pulse 对应 -45° 的脉宽
 * @param max_pulse 对应 +45° 的脉宽
 * @param center_pulse 对应 0° 的脉宽
 * @return 对应的脉宽 (微秒)
 */
uint16_t angle_to_pulse(float angle, uint16_t min_pulse, uint16_t max_pulse, uint16_t center_pulse);

/**
 * @brief 脉宽转角度 (用于旋转舵机)
 * 
 * @param pulse 脉宽 (微秒)
 * @param min_pulse 对应 -45° 的脉宽
 * @param max_pulse 对应 +45° 的脉宽
 * @param center_pulse 对应 0° 的脉宽
 * @return 对应的旋转角度 (度)
 */
float pulse_to_angle(uint16_t pulse, uint16_t min_pulse, uint16_t max_pulse, uint16_t center_pulse);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_CONTROLLER_H
