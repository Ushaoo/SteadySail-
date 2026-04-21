#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// 初始化转向编码器捕获中断
void steering_control_init(void);

// 校准编码器：将当前竖直状态设定为 0° 基准点
// 需在 steering_control_init() 后立即调用
void steering_control_calibrate_encoders(void);

// 设置两个小电机的目标旋转角度 (0~360度)
void steering_control_set_target(float target_left_deg, float target_right_deg);

// 获取目前的实时物理角度
void steering_control_get_current_angles(float *left_deg, float *right_deg);

// 执行一次 PID 计算并输出到 PWM。需放置于 100Hz 定时任务中！
void steering_control_update(void);

#endif // STEERING_CONTROL_H