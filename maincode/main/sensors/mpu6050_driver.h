/**
 * @file mpu6050_driver.h
 * @brief MPU6050 9-DoF IMU 传感器驱动
 * 
 * 提供读取加速度、陀螺仪和温度的接口
 */

#ifndef MPU6050_DRIVER_H
#define MPU6050_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdint.h>
#include <math.h>

/* ========== I2C 寄存器地址 ========== */
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48

/* ========== 数据结构体 ========== */

/**
 * @brief 3 轴原始数据
 */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} raw_data_t;

/**
 * @brief 3 轴浮点数据 (已缩放)
 */
typedef struct {
    float x;
    float y;
    float z;
} float_data_t;

/**
 * @brief MPU6050 实例结构体
 */
typedef struct {
    uint8_t address;                ///< I2C 地址
    float accel_scale;              ///< 加速度缩放因子
    float gyro_scale;               ///< 陀螺仪缩放因子
} mpu6050_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 MPU6050
 * 
 * @param mpu MPU6050 实例指针
 * @param address I2C 地址 (通常 0x68 或 0x69)
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_init(mpu6050_t *mpu, uint8_t address);

/**
 * @brief 自动校准 IMU 零偏
 * 
 * 设备必须保持完全静止
 * 
 * @param mpu MPU6050 实例指针
 * @param samples 采样数 (通常 200-500)
 * @param gyro_bias 返回的陀螺仪零偏指针
 * @param accel_bias 返回的加速度零偏指针
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_calibrate(mpu6050_t *mpu, uint16_t samples, 
                            float_data_t *gyro_bias, float_data_t *accel_bias);

/**
 * @brief 读取加速度数据 (原始值)
 * 
 * @param mpu MPU6050 实例指针
 * @param accel 返回的加速度数据指针
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_read_accel_raw(mpu6050_t *mpu, raw_data_t *accel);

/**
 * @brief 读取陀螺仪数据 (原始值)
 * 
 * @param mpu MPU6050 实例指针
 * @param gyro 返回的陀螺仪数据指针
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_read_gyro_raw(mpu6050_t *mpu, raw_data_t *gyro);

/**
 * @brief 读取加速度和陀螺仪数据 (原始值)
 * 
 * @param mpu MPU6050 实例指针
 * @param accel 返回的加速度数据指针
 * @param gyro 返回的陀螺仪数据指针
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_read_all_raw(mpu6050_t *mpu, raw_data_t *accel, raw_data_t *gyro);

/**
 * @brief 读取加速度数据 (已缩放，单位 g)
 * 
 * @param mpu MPU6050 实例指针
 * @param accel 返回的加速度数据指针
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_read_accel_g(mpu6050_t *mpu, float_data_t *accel);

/**
 * @brief 读取陀螺仪数据 (已缩放，单位 deg/s)
 * 
 * @param mpu MPU6050 实例指针
 * @param gyro 返回的陀螺仪数据指针
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_read_gyro_dps(mpu6050_t *mpu, float_data_t *gyro);

/**
 * @brief 读取温度
 * 
 * @param mpu MPU6050 实例指针
 * @param temp 返回的温度指针 (单位 °C)
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_read_temp(mpu6050_t *mpu, float *temp);

/**
 * @brief 读取完整数据 (包括温度，已缩放)
 * 
 * @param mpu MPU6050 实例指针
 * @param accel 加速度数据指针
 * @param gyro 陀螺仪数据指针
 * @param temp 温度指针
 * @return ESP_OK 表示成功
 */
esp_err_t mpu6050_read_all(mpu6050_t *mpu, float_data_t *accel, float_data_t *gyro, float *temp);

/**
 * @brief 应用轴向反转 (根据硬件安装方位)
 * 
 * @param data 原始数据指针
 * @param invert_x X 轴是否反转 (1 = 反转, 0 = 正向)
 * @param invert_y Y 轴是否反转
 * @param invert_z Z 轴是否反转
 */
void mpu6050_apply_axis_invert(float_data_t *data, int invert_x, int invert_y, int invert_z);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_DRIVER_H
