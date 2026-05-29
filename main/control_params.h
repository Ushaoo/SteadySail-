#ifndef CONTROL_PARAMS_H
#define CONTROL_PARAMS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// 运行时可调参数（通过 Blinker / 串口动态修改）
// 默认值在 system_config.h 中定义，初始化时拷贝到全局变量
// ============================================================

// --- 平衡控制 PID（用于 balance_controller.c）---
extern volatile float g_balance_kp;
extern volatile float g_balance_ki;
extern volatile float g_balance_kd;

// --- 转向控制 PID（用于 steering_control.c）---
extern volatile float g_steer_kp;
extern volatile float g_steer_ki;
extern volatile float g_steer_kd;

// 初始化全局参数（从 system_config.h 默认值）
// 必须在 app_main 早期调用
void control_params_init(void);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_PARAMS_H
