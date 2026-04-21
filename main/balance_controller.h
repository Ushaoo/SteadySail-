#ifndef BALANCE_CONTROLLER_H
#define BALANCE_CONTROLLER_H

#include "imu_driver.h"

// 系统的实时状态输出
typedef struct {
    float roll_deg;          // 横滚角 (设右倾为正)
    float pitch_deg;         // 俯仰角
    float yaw_deg;           // 偏航角
    
    float omega_filtered;    // 滤波后的角速度
    float alpha;             // 角加速度
    
    float tau_ff;            // 前馈输出力矩
    float tau_pid;           // PID 反馈输出力矩
    float tau_total;         // 最终分配总力矩
} balance_state_t;

// 初始化平衡控制器 (复位四元数与缓存)
void balance_controller_init(void);

// 执行一次平衡控制计算，放在 100Hz 定时任务中，传入读取到的姿态，输出计算好的状态
void balance_controller_update(dual_imu_data_t *imu_data, balance_state_t *state);

#endif // BALANCE_CONTROLLER_H
