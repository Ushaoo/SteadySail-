#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

typedef struct {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} imu_data_t;

typedef struct {
    imu_data_t imu1;
    imu_data_t imu2;
} dual_imu_data_t;

// 初始化双总线及两个 MPU6050
esp_err_t imu_driver_init(void);

// 读取双 IMU 的最新姿态原始数据
esp_err_t imu_driver_read(dual_imu_data_t *data);

#endif // IMU_DRIVER_H
