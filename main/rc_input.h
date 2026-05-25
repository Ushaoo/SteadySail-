#pragma once
// rc_input.h — RC 接收机单通道 PWM 输入驱动
// 用 GPIO 中断 + esp_timer 测量脉宽，适配标准 1000~2000μs RC PWM
// 用途：将遥感前后轴映射为 g_forward_thrust（-100% ~ +100%）

#include "esp_err.h"
#include <stdbool.h>

// 初始化：配置 gpio_num 为双边沿中断输入，安装 ISR
esp_err_t rc_input_init(int gpio_num);

// 获取油门值：-100.0 ~ +100.0（中位死区内返回 0.0）
// 超过 RC_SIGNAL_TIMEOUT_MS 无信号时返回 0.0（安全归零）
float rc_input_get_throttle(void);

// 信号有效性：最近 RC_SIGNAL_TIMEOUT_MS 内收到过合法脉冲
bool rc_input_is_valid(void);

// 是否正处于定速巡航状态（锁定后向零点回撤 20 点触发定速）
bool rc_input_is_cruising(void);

// 外部主动取消定速（如急停、Blinker操作等）
void rc_input_cancel_cruise(void);

// 获取最近一次捕获的原始脉宽（μs），无信号时返回 0
uint32_t rc_input_get_raw_pulse_us(void);

// 打印诊断信息：上升沿/下降沿计数、丢弃次数、最近原始脉宽
void rc_input_print_diag(void);

// 获取最近一次合法 RC 脉宽（μs，1000~2000），无信号时返回 0
uint32_t rc_input_get_raw_pwm(void);
