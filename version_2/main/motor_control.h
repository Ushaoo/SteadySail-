#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

// 初始化所有电机与引脚外设 (LEDC)
void motor_control_init(void);

// 设置小电机转向 PWM 输出 (单位: 微秒 us, 通常 1000~2000)
// 传入左右转向大舵机（小电机）的设定脉宽
void motor_control_set_steering_pwm(uint32_t pwm_left_us, uint32_t pwm_right_us);

// 核心补偿逻辑：设置主推进器推力
// 输入：tau_total (平衡算法输出的垂直恢复力矩), current_angle_deg (当前小电机偏角)
void motor_control_set_thrust_with_compensation(float tau_total, float current_angle_deg);

// 紧急停止所有输出 (切回 1500 停转)
void motor_control_emergency_stop(void);

// 大电机 ESC 校准序列任务
void motor_control_esc_calibrate_task(void *pvParameters);

#endif // MOTOR_CONTROL_H
