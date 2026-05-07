#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

// 初始化转向编码器捕获中断
// 内部会尝试从 NVS 读取上次保存的零点偏移；若读取成功则进入“已校准”状态，
// 否则保持“未校准”——此时 steering_control_update() 会强制下发 PWM=1500，
// 直到用户调用 steering_control_calibrate_and_save() 完成首次校准。
void steering_control_init(void);

// 把舵机摆到竖直后调用：读取当前编码器原始角作为 180° 基准，写入 NVS 持久化。
// 调用后立即进入“已校准”状态。
void steering_control_calibrate_and_save(void);

// 是否已经校准 (NVS 中有有效 offset 或当前会话内已 calibrate_and_save)
bool steering_control_is_calibrated(void);

// 设置两个小电机的目标旋转角度 (0~360度)
void steering_control_set_target(float target_left_deg, float target_right_deg);

// 获取当前下发的目标角度（与编码器同参考系）
void steering_control_get_target(float *left_deg, float *right_deg);

// 获取目前的实时物理角度
void steering_control_get_current_angles(float *left_deg, float *right_deg);

// 获取编码器健康状态 (true = 正常, false = 故障/断连)
void steering_control_get_encoder_status(bool *left_ok, bool *right_ok);

// 执行一次 PID 计算并输出到 PWM。需放置于 100Hz 定时任务中！
void steering_control_update(void);

#endif // STEERING_CONTROL_H