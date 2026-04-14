/**
 * @file deadzone.h
 * @brief 死区处理函数
 * 
 * 参考树莓派版本: apply_deadzone_smooth()
 */

#ifndef DEADZONE_H
#define DEADZONE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 非线性死区插值函数
 * 
 * 用于防止小的扰动导致过度响应
 * 
 * @param value 输入值
 * @param deadzone_core 核心死区阈值
 * @param deadzone_soft 软死区阈值
 * @return 处理后的输出值
 * 
 * 逻辑:
 * - 如果 |value| < deadzone_core: 返回 0
 * - 如果 |value| >= deadzone_soft: 返回 value
 * - 否则: 在两者之间平滑过渡
 */
float apply_deadzone_smooth(float value, float deadzone_core, float deadzone_soft);

#ifdef __cplusplus
}
#endif

#endif // DEADZONE_H
