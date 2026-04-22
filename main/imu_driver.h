#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include "esp_err.h"
#include "system_config.h"

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_data_t;

#if USE_DUAL_IMU
// 双 IMU 模式：融合两个独立的传感器
typedef struct {
    imu_data_t imu1;
    imu_data_t imu2;
} dual_imu_data_t;
#else
// 单 IMU 模式：仅使用一个传感器（为兼容性，仍用 dual_imu_data_t 结构体，但只填充 imu1）
typedef struct {
    imu_data_t imu1;    // 实际使用的唯一传感器
    imu_data_t imu2;    // 未使用（预留）
} dual_imu_data_t;
#endif

// 初始化 IMU 驱动
// 单IMU模式: 初始化 I2C0 (GPIO 8/9)
// 双IMU模式: 初始化 I2C0 (GPIO 8/9) + I2C1 (GPIO 10/11)
esp_err_t imu_driver_init(void);

// 读取 IMU 原始数据
esp_err_t imu_driver_read(dual_imu_data_t *data);

#endif // IMU_DRIVER_H
