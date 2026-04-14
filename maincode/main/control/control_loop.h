/**
 * @file control_loop.h
 * @brief 主控制循环
 * 
 * 100 Hz 控制循环，集成 IMU 读取、融合、PID 计算、电机控制
 */

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050_driver.h"
#include "imu_fusion.h"
#include "pid_controller.h"
#include "motor_controller.h"
#include "force_allocation.h"
#include "uart_command.h"

/* ========== 数据结构体 ========== */

/**
 * @brief 控制循环上下文
 */
typedef struct {
    // IMU 实例
    mpu6050_t imu1;
    mpu6050_t imu2;
    
    // 融合实例
    imu_fusion_t fusion;
    
    // PID 控制器
    pid_controller_t pid;
    
    // 电机控制器
    motor_controller_t motor;
    
    // 力分配器（根据推进器角度分配控制力）
    force_allocator_t force_allocator;
    
    // UART 命令接收器（接收推进器角度指令）
    uart_command_t uart_receiver;
    
    // 控制参数
    float setpoint_angle;           ///< 目标倾斜角 (度，通常 0)
    float max_control_output;       ///< 最大控制输出限制
    
    // 系统状态
    uint32_t loop_count;            ///< 循环计数
    uint32_t last_imu_error;        ///< 最后的 IMU 错误计数
    
    // 角度校准
    float angle_offset;             ///< 角度零点偏差 (度)
    int is_calibrated;              ///< 是否已校准 (1=是, 0=否)
    uint32_t calibration_count;     ///< 校准计数器
    float calibration_sum;          ///< 校准累计值
    
    // FreeRTOS 任务
    TaskHandle_t task_handle;       ///< 控制循环任务句柄
    
} control_loop_t;

/**
 * @brief 控制循环统计数据
 */
typedef struct {
    uint32_t total_iterations;      ///< 总迭代次数
    uint32_t imu_read_errors;       ///< IMU 读取错误数
    uint32_t motor_write_errors;    ///< 电机写入错误数
    float avg_loop_time_ms;         ///< 平均循环时间 (ms)
    float max_loop_time_ms;         ///< 最大循环时间 (ms)
} control_loop_stats_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化控制循环
 * 
 * @param loop 控制循环实例指针
 * @return ESP_OK 表示成功
 */
esp_err_t control_loop_init(control_loop_t *loop);

/**
 * @brief 启动控制循环 (创建 FreeRTOS 任务)
 * 
 * @param loop 控制循环实例指针
 * @param priority 任务优先级 (通常 20-24)
 * @param stack_size 栈大小 (字节，通常 4096 或 8192)
 * @return ESP_OK 表示成功
 */
esp_err_t control_loop_start(control_loop_t *loop, uint32_t priority, uint32_t stack_size);

/**
 * @brief 停止控制循环
 * 
 * @param loop 控制循环实例指针
 * @return ESP_OK 表示成功
 */
esp_err_t control_loop_stop(control_loop_t *loop);

/**
 * @brief 设置目标角度
 * 
 * @param loop 控制循环实例指针
 * @param setpoint 目标角度 (度)
 */
void control_loop_set_setpoint(control_loop_t *loop, float setpoint);

/**
 * @brief 获取控制循环统计数据
 * 
 * @param loop 控制循环实例指针
 * @param stats 返回的统计数据指针
 */
void control_loop_get_stats(control_loop_t *loop, control_loop_stats_t *stats);

/**
 * @brief 重置控制循环统计数据
 * 
 * @param loop 控制循环实例指针
 */
void control_loop_reset_stats(control_loop_t *loop);

/**
 * @brief 获取当前角度偏差
 * 
 * @param loop 控制循环实例指针
 * @return 角度偏差值 (度)
 */
float control_loop_get_angle_offset(control_loop_t *loop);

/**
 * @brief 设置角度偏差（手动校准）
 * 
 * @param loop 控制循环实例指针
 * @param offset 角度偏差值 (度)
 */
void control_loop_set_angle_offset(control_loop_t *loop, float offset);

/**
 * @brief 获取校准状态
 * 
 * @param loop 控制循环实例指针
 * @return 1 = 已校准, 0 = 未校准
 */
int control_loop_is_calibrated(control_loop_t *loop);

/**
 * @brief 应用角度偏差补偿（内部使用）
 * 
 * 从原始欧拉角中减去偏差
 * 
 * @param euler 欧拉角指针
 * @param offset 角度偏差值 (度)
 */
void control_loop_apply_angle_offset(euler_angle_t *euler, float offset);

#ifdef __cplusplus
}
#endif

#endif // CONTROL_LOOP_H
