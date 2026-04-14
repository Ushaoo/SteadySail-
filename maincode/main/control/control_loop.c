/**
 * @file control_loop.c
 * @brief 主控制循环实现
 * 
 * 100 Hz 控制闭环，集成：
 * 1. IMU 数据读取
 * 2. 双 IMU 融合
 * 3. PID 控制计算
 * 4. 死区处理
 * 5. 力分配（根据推进器角度调整控制力）
 * 6. 电机输出
 */

#include "control_loop.h"
#include "config.h"
#include "deadzone.h"
#include "mpu6050_driver.h"
#include "imu_fusion.h"
#include "pid_controller.h"
#include "motor_controller.h"
#include "pca9685_driver.h"
#include "force_allocation.h"
#include "uart_command.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "i2c_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>
#include <limits.h>

static const char *TAG = "CONTROL_LOOP";

static control_loop_t *g_loop_instance = NULL;

// ==============================================================================
// UART 回调函数
// ==============================================================================

/**
 * @brief UART 接收到推进器角度命令的回调函数
 */
static void on_thruster_angle_command(float angle)
{
    if (g_loop_instance != NULL) {
        ESP_LOGI(TAG, "Thruster angle command received: %.1f°", angle);
        force_allocator_set_target_angle(&g_loop_instance->force_allocator, angle);
    }
}

// ==============================================================================
// 辅助函数
// ==============================================================================

static float clamp_value(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// ==============================================================================
// 主控制循环任务
// ==============================================================================

/**
 * @brief 100 Hz 控制循环 FreeRTOS 任务
 * 
 * 六阶段管道：
 * 1. 读取两个 IMU 传感器数据
 * 2. 应用轴反演校正
 * 3. 双 IMU 融合（四元数+Mahony）
 * 4. 死区处理
 * 5. PID 控制计算
 * 6. 电机差分驱动输出
 */
static void control_loop_task(void *pvParameters)
{
    control_loop_t *loop = (control_loop_t *)pvParameters;
    
    if (loop == NULL) {
        ESP_LOGE(TAG, "Control loop task: NULL parameter");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Control loop task started at 100 Hz");

    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t loop_period = pdMS_TO_TICKS(10);  // 100 Hz
    
    euler_angle_t euler = {0};
    float_data_t accel1, gyro1, accel2, gyro2;
    float temp1, temp2;
    esp_err_t ret;

    while (1) {
        // ===== 步骤 1 & 2: 读取 IMU 并应用轴反演 =====
        ret = mpu6050_read_all(&loop->imu1, &accel1, &gyro1, &temp1);
        if (ret != ESP_OK) {
            loop->last_imu_error = 1;
        } else {
            mpu6050_apply_axis_invert(&accel1, IMU1_INVERT_X, IMU1_INVERT_Y, IMU1_INVERT_Z);
            mpu6050_apply_axis_invert(&gyro1, IMU1_INVERT_X, IMU1_INVERT_Y, IMU1_INVERT_Z);
        }
        
        ret = mpu6050_read_all(&loop->imu2, &accel2, &gyro2, &temp2);
        if (ret != ESP_OK) {
            loop->last_imu_error = 2;
        } else {
            mpu6050_apply_axis_invert(&accel2, IMU2_INVERT_X, IMU2_INVERT_Y, IMU2_INVERT_Z);
            mpu6050_apply_axis_invert(&gyro2, IMU2_INVERT_X, IMU2_INVERT_Y, IMU2_INVERT_Z);
        }
        
        // ===== 步骤 3: 双 IMU 融合 =====
        imu_fusion_update(&loop->fusion,
                         accel1.x, accel1.y, accel1.z,
                         gyro1.x, gyro1.y, gyro1.z,
                         accel2.x, accel2.y, accel2.z,
                         gyro2.x, gyro2.y, gyro2.z,
                         1.0f / CONTROL_FREQ_HZ, &euler);
        
        // ===== 步骤 3.5: 角度校准 =====
        // 如果尚未校准，首先采集样本计算偏差
        if (!loop->is_calibrated && ENABLE_ANGLE_CALIBRATION) {
            loop->calibration_sum += euler.roll;
            loop->calibration_count++;
            
            if (loop->calibration_count >= CALIBRATION_SAMPLES_COUNT) {
                // 校准完成：计算平均值作为偏差
                loop->angle_offset = loop->calibration_sum / CALIBRATION_SAMPLES_COUNT;
                loop->is_calibrated = 1;
                ESP_LOGI(TAG, "Angle calibration completed! Offset = %.2f°", loop->angle_offset);
            } else if ((loop->calibration_count % 10) == 0) {
                ESP_LOGI(TAG, "Calibrating... %d/%d samples", loop->calibration_count, CALIBRATION_SAMPLES_COUNT);
            }
        }
        
        // 应用角度偏差补偿
        control_loop_apply_angle_offset(&euler, loop->angle_offset);
        
        // ===== 步骤 4: 死区处理 =====
        float roll_filtered = apply_deadzone_smooth(euler.roll, ANGLE_DEADZONE, ANGLE_DEADZONE_SOFT);
        
        // ===== 步骤 5: PID 控制计算 =====
        float setpoint = loop->setpoint_angle;  // 通常为 0（保持水平）
        float pid_output = pid_update(&loop->pid, setpoint, roll_filtered, gyro1.x);
        pid_output = clamp_value(pid_output, -loop->max_control_output, loop->max_control_output);
        
        // ===== 步骤 6: 力分配与电机输出 =====
        uint16_t pulse_left, pulse_right;
        float current_thruster_angle;

#if ENABLE_FORCE_ALLOCATION
        // 启用力分配：根据推进器角度和 PID 输出计算左右推进器脉宽
        ret = force_allocator_compute(&loop->force_allocator, pid_output, 
                                      &pulse_left, &pulse_right);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Force allocation failed");
        }
        current_thruster_angle = force_allocator_get_current_angle(&loop->force_allocator);
#else
        // 禁用力分配：固定推进器竖直向下 (90°)，PID 输出直接转换为差分推进
        current_thruster_angle = 90.0f;  // 固定推进器方向
        float base = BASE_PULSE;
        pulse_left = (uint16_t)(base + pid_output);
        pulse_right = (uint16_t)(base - pid_output);
#endif
        
        // 输出到电机
        ret = motor_set_thrusters(&loop->motor, pulse_left, pulse_right);
        if (ret != ESP_OK) {
            loop->last_imu_error++;
        }
        
        // 将推进器角度转换为舵机角度（将 0-180° 映射到 -45° 到 +45°）
        // 舵机角度 = (推进器角度 - 90°) / 2
        float servo_angle = (current_thruster_angle - 90.0f) / 2.0f;
        loop->loop_count++;
        
        // 每 500 次循环打印一次统计
        if ((loop->loop_count % 500) == 0) {
            ESP_LOGI(TAG, "Loops=%lu | Attitude: roll=%.1f° | PID: Δ=%.1f | Thruster: %.1f° | Servo: %.1f°",
                    loop->loop_count,
                    euler.roll, pid_output, current_thruster_angle, servo_angle);
        }
        
        // 等待下一个周期 (10 ms for 100 Hz)
        xTaskDelayUntil(&last_wake_time, loop_period);
    }
}

// ==============================================================================
// 初始化和管理函数
// ==============================================================================

esp_err_t control_loop_init(control_loop_t *loop)
{
    if (loop == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing control loop...");
    ESP_LOGI(TAG, "DEBUG SWITCHES:");
    ESP_LOGI(TAG, "  - ENABLE_SERVO_ROTATION: %d", ENABLE_SERVO_ROTATION);
    ESP_LOGI(TAG, "  - ENABLE_FORCE_ALLOCATION: %d", ENABLE_FORCE_ALLOCATION);
    
    memset(loop, 0, sizeof(control_loop_t));

    // ===== 初始化 I2C =====
    ESP_LOGI(TAG, "Initializing I2C driver...");
    esp_err_t ret = i2c_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    i2c_scan_devices();

    // ===== 初始化 IMU1 =====
    ESP_LOGI(TAG, "Initializing IMU1 at address 0x%02x...", IMU1_ADDRESS);
    ret = mpu6050_init(&loop->imu1, IMU1_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU1 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    float_data_t gyro_bias1, accel_bias1;
    ret = mpu6050_calibrate(&loop->imu1, 10, &gyro_bias1, &accel_bias1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU1 calibration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ===== 初始化 IMU2 =====
    ESP_LOGI(TAG, "Initializing IMU2 at address 0x%02x...", IMU2_ADDRESS);
    ret = mpu6050_init(&loop->imu2, IMU2_ADDRESS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU2 init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    float_data_t gyro_bias2, accel_bias2;
    ret = mpu6050_calibrate(&loop->imu2, 10, &gyro_bias2, &accel_bias2);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU2 calibration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ===== 初始化 IMU 融合 =====
    ESP_LOGI(TAG, "Initializing IMU fusion...");
    imu_fusion_init(&loop->fusion);

    // ===== 初始化 PID 控制器 =====
    ESP_LOGI(TAG, "Initializing PID controller...");
    pid_init(&loop->pid, PID_KP, PID_KI, PID_KD,
             PID_B, PID_C, 1.0f / CONTROL_FREQ_HZ);

    // ===== 初始化 PCA9685 PWM 驱动 =====
    ESP_LOGI(TAG, "Initializing PCA9685 at address 0x%02x...", PCA9685_ADDRESS);
    pca9685_t pca9685 = {0};
    ret = pca9685_init(&pca9685, PCA9685_ADDRESS, PWM_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PCA9685 init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ===== 初始化电机控制器 =====
    ESP_LOGI(TAG, "Initializing motor controller...");
    ret = motor_init(&loop->motor, &pca9685, LEFT_THRUSTER, RIGHT_THRUSTER, 
                    LEFT_ROTATION_CH, RIGHT_ROTATION_CH);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    motor_set_thrusters(&loop->motor, BASE_PULSE, BASE_PULSE);

    // ===== 初始化力分配器 =====
    ESP_LOGI(TAG, "Initializing force allocator...");
    ret = force_allocator_init(&loop->force_allocator, 
                               BASE_PULSE, MIN_PULSE, MAX_PULSE, 
                               ROTATION_RATE_LIMIT, 1.0f / CONTROL_FREQ_HZ);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Force allocator init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ===== 初始化 UART 命令接收 =====
#if ENABLE_SERVO_ROTATION
    ESP_LOGI(TAG, "Servo rotation ENABLED - Initializing UART command receiver...");
    ret = uart_command_init(&loop->uart_receiver, on_thruster_angle_command);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART command init failed: %s (continuing anyway)", esp_err_to_name(ret));
        // 不返回错误，允许系统在没有UART的情况下继续运行
    }

    // ===== 启动 UART 接收任务 =====
    ret = uart_command_start(&loop->uart_receiver, 10, 2048);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "UART command task start failed: %s", esp_err_to_name(ret));
        // 不返回错误，允许系统继续运行
    }
#else
    ESP_LOGI(TAG, "Servo rotation DISABLED - UART command receiver not started");
#endif

    // ===== 初始化参数 =====
    loop->setpoint_angle = 0.0f;
    loop->max_control_output = 50.0f;
    loop->loop_count = 0;
    loop->last_imu_error = 0;
    
    // ===== 初始化角度校准 =====
#if ENABLE_ANGLE_CALIBRATION
    loop->angle_offset = 0.0f;
    loop->is_calibrated = 0;
    loop->calibration_count = 0;
    loop->calibration_sum = 0.0f;
    ESP_LOGI(TAG, "Angle calibration ENABLED - waiting 3 seconds for user to set vertical position...");
    vTaskDelay(pdMS_TO_TICKS(CALIBRATION_WAIT_TIME));
    ESP_LOGI(TAG, "Starting angle calibration - sampling %d readings...", CALIBRATION_SAMPLES_COUNT);
#else
    loop->angle_offset = 0.0f;
    loop->is_calibrated = 1;
    ESP_LOGI(TAG, "Angle calibration DISABLED");
#endif

    g_loop_instance = loop;

    ESP_LOGI(TAG, "Control loop initialized successfully!");
    return ESP_OK;
}

esp_err_t control_loop_start(control_loop_t *loop, uint32_t priority, uint32_t stack_size)
{
    if (loop == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    BaseType_t ret = xTaskCreate(
        control_loop_task,
        "CONTROL_LOOP",
        stack_size,
        loop,
        priority,
        &loop->task_handle
    );

    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control loop task");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "Control loop task created (priority=%lu, stack=%lu bytes)", priority, stack_size);
    return ESP_OK;
}

esp_err_t control_loop_stop(control_loop_t *loop)
{
    if (loop == NULL || loop->task_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    vTaskDelete(loop->task_handle);
    loop->task_handle = NULL;

    ESP_LOGI(TAG, "Control loop stopped");
    return ESP_OK;
}

void control_loop_set_setpoint(control_loop_t *loop, float setpoint)
{
    if (loop == NULL) {
        return;
    }

    loop->setpoint_angle = setpoint;
    ESP_LOGI(TAG, "Setpoint updated: %.2f°", setpoint);
}

void control_loop_get_stats(control_loop_t *loop, control_loop_stats_t *stats)
{
    if (loop == NULL || stats == NULL) {
        return;
    }

    stats->total_iterations = loop->loop_count;
    stats->imu_read_errors = loop->last_imu_error;
}

void control_loop_reset_stats(control_loop_t *loop)
{
    if (loop == NULL) {
        return;
    }

    loop->loop_count = 0;
    loop->last_imu_error = 0;
}

/**
 * @brief 获取当前角度偏差
 */
float control_loop_get_angle_offset(control_loop_t *loop)
{
    if (loop == NULL) {
        return 0.0f;
    }
    return loop->angle_offset;
}

/**
 * @brief 设置角度偏差（手动校准）
 */
void control_loop_set_angle_offset(control_loop_t *loop, float offset)
{
    if (loop == NULL) {
        return;
    }
    
    loop->angle_offset = offset;
    loop->is_calibrated = 1;
    ESP_LOGI(TAG, "Angle offset manually set to: %.2f°", offset);
}

/**
 * @brief 获取校准状态
 */
int control_loop_is_calibrated(control_loop_t *loop)
{
    if (loop == NULL) {
        return 0;
    }
    return loop->is_calibrated;
}

/**
 * @brief 应用角度偏差补偿
 * 
 * 从原始欧拉角中减去偏差
 * 示例：
 *   - 原始读数: 95°
 *   - 偏差: 5°
 *   - 校准后: 95° - 5° = 90°
 */
void control_loop_apply_angle_offset(euler_angle_t *euler, float offset)
{
    if (euler == NULL) {
        return;
    }
    
    // 只校准 roll 角（横滚角，船的平衡）
    euler->roll -= offset;
}
