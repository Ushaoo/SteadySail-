/**
 * @file deadzone.c
 * @brief 死区处理实现
 */

#include "deadzone.h"
#include <math.h>

float apply_deadzone_smooth(float value, float deadzone_core, float deadzone_soft)
{
    float abs_val = fabsf(value);

    if (abs_val < deadzone_core) {
        return 0.0f;
    } else if (abs_val >= deadzone_soft) {
        return value;
    } else {
        // 平滑过渡 (使用三次 Hermite 插值)
        float t = (abs_val - deadzone_core) / (deadzone_soft - deadzone_core);
        float smooth_factor = t * t * (3.0f - 2.0f * t);
        return (value > 0.0f ? 1.0f : -1.0f) * abs_val * smooth_factor;
    }
}
