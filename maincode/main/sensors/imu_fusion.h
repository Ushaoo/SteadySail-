/**
 * @file imu_fusion.h
 * @brief 双 IMU 融合算法 (DualIMUFusion)
 * 
 * 四元数融合、加速度互补滤波、陀螺仪加权
 * 参考树莓派版本: dual_imu.py
 */

#ifndef IMU_FUSION_H
#define IMU_FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

/* ========== 数据结构体 ========== */

/**
 * @brief 融合后的欧拉角
 */
typedef struct {
    float roll;     ///< 横滚角 (度)
    float pitch;    ///< 俯仰角 (度)
    float yaw;      ///< 偏航角 (度)
} euler_angle_t;

/**
 * @brief 四元数
 */
typedef struct {
    float q0, q1, q2, q3;
} quaternion_t;

/**
 * @brief 双 IMU 融合实例
 */
typedef struct {
    // 四元数
    quaternion_t quat;
    
    // 陀螺仪零偏 (每个 IMU)
    float bias_gx[2];
    float bias_gy[2];
    float bias_gz[2];
    
    // 方差估计 (用于动态加权)
    float var_gx[2];
    float var_gy[2];
    float var_gz[2];
    
    // 循环缓冲区，用于在线噪声估计
    float gyro_buf[2][100][3];  // [imu][sample][xyz]
    uint16_t gyro_buf_idx[2];
    uint16_t gyro_buf_filled[2];
    
} imu_fusion_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 IMU 融合
 * 
 * @param fusion 融合实例指针
 */
void imu_fusion_init(imu_fusion_t *fusion);

/**
 * @brief 设置陀螺仪零偏
 * 
 * @param fusion 融合实例指针
 * @param gyro_bias1 IMU1 陀螺仪零偏指针 (float[3] = {x, y, z})
 * @param gyro_bias2 IMU2 陀螺仪零偏指针
 */
void imu_fusion_set_gyro_bias(imu_fusion_t *fusion, const float *gyro_bias1, const float *gyro_bias2);

/**
 * @brief 更新融合算法
 * 
 * 使用两个 IMU 的加速度和陀螺仪数据进行四元数融合
 * 
 * @param fusion 融合实例指针
 * @param ax1, ay1, az1 IMU1 加速度 (g)
 * @param gx1, gy1, gz1 IMU1 陀螺仪 (deg/s)
 * @param ax2, ay2, az2 IMU2 加速度 (g)
 * @param gx2, gy2, gz2 IMU2 陀螺仪 (deg/s)
 * @param dt 时间步长 (秒)
 * @param euler 返回的欧拉角指针
 * @return 0 表示成功
 */
int imu_fusion_update(imu_fusion_t *fusion,
                     float ax1, float ay1, float az1,
                     float gx1, float gy1, float gz1,
                     float ax2, float ay2, float az2,
                     float gx2, float gy2, float gz2,
                     float dt,
                     euler_angle_t *euler);

/**
 * @brief 复位融合状态
 * 
 * @param fusion 融合实例指针
 */
void imu_fusion_reset(imu_fusion_t *fusion);

/**
 * @brief 获取当前四元数
 * 
 * @param fusion 融合实例指针
 * @return 当前四元数
 */
quaternion_t imu_fusion_get_quaternion(imu_fusion_t *fusion);

/**
 * @brief 四元数转欧拉角
 * 
 * @param quat 四元数
 * @param euler 返回的欧拉角指针
 */
void quat_to_euler(const quaternion_t *quat, euler_angle_t *euler);

#ifdef __cplusplus
}
#endif

#endif // IMU_FUSION_H
