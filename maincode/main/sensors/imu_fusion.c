/**
 * @file imu_fusion.c
 * @brief 双 IMU 融合实现
 * 
 * 基于树莓派版本转译，参考 dual_imu.py
 * 算法流程：
 * 1. 加速度互补融合
 * 2. 陀螺仪方差在线估计与动态加权
 * 3. Mahony 四元数更新
 * 4. 输出欧拉角
 */

#include "imu_fusion.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "IMU_FUSION";

void imu_fusion_init(imu_fusion_t *fusion)
{
    if (fusion == NULL) {
        return;
    }

    memset(fusion, 0, sizeof(imu_fusion_t));

    // 初始化四元数为单位四元数
    fusion->quat.q0 = 1.0f;
    fusion->quat.q1 = 0.0f;
    fusion->quat.q2 = 0.0f;
    fusion->quat.q3 = 0.0f;

    // 初始化方差
    for (int i = 0; i < 2; i++) {
        fusion->var_gx[i] = 1.0f;
        fusion->var_gy[i] = 1.0f;
        fusion->var_gz[i] = 1.0f;
    }

    ESP_LOGI(TAG, "IMU fusion initialized");
}

void imu_fusion_set_gyro_bias(imu_fusion_t *fusion, const float *gyro_bias1, const float *gyro_bias2)
{
    if (fusion == NULL || gyro_bias1 == NULL || gyro_bias2 == NULL) {
        return;
    }

    fusion->bias_gx[0] = gyro_bias1[0];
    fusion->bias_gy[0] = gyro_bias1[1];
    fusion->bias_gz[0] = gyro_bias1[2];

    fusion->bias_gx[1] = gyro_bias2[0];
    fusion->bias_gy[1] = gyro_bias2[1];
    fusion->bias_gz[1] = gyro_bias2[2];

    ESP_LOGI(TAG, "Gyro bias set: IMU1=(%.4f, %.4f, %.4f), IMU2=(%.4f, %.4f, %.4f)",
             gyro_bias1[0], gyro_bias1[1], gyro_bias1[2],
             gyro_bias2[0], gyro_bias2[1], gyro_bias2[2]);
}

/**
 * @brief 计算方差
 */
static float calculate_variance(const float *data, int count)
{
    if (count < 2) return 0.0f;

    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        sum += data[i];
    }
    float mean = sum / count;

    float var_sum = 0.0f;
    for (int i = 0; i < count; i++) {
        float diff = data[i] - mean;
        var_sum += diff * diff;
    }

    return var_sum / count;
}

int imu_fusion_update(imu_fusion_t *fusion,
                     float ax1, float ay1, float az1,
                     float gx1, float gy1, float gz1,
                     float ax2, float ay2, float az2,
                     float gx2, float gy2, float gz2,
                     float dt,
                     euler_angle_t *euler)
{
    if (fusion == NULL || euler == NULL || dt <= 0.0f) {
        return -1;
    }

    /* ========== Step 1: 加速度互补融合 ========== */
    
    // 计算加速度幅值
    float acc1_mag = sqrtf(ax1*ax1 + ay1*ay1 + az1*az1);
    float acc2_mag = sqrtf(ax2*ax2 + ay2*ay2 + az2*az2);

    // 归一化加速度
    if (acc1_mag > 0.1f) {
        ax1 /= acc1_mag; ay1 /= acc1_mag; az1 /= acc1_mag;
    }
    if (acc2_mag > 0.1f) {
        ax2 /= acc2_mag; ay2 /= acc2_mag; az2 /= acc2_mag;
    }

    // 互补融合加速度 (ALPHA_ACC = 0.98，更信任低频 IMU1)
    float ax = ALPHA_ACC * ax1 + (1.0f - ALPHA_ACC) * ax2;
    float ay = ALPHA_ACC * ay1 + (1.0f - ALPHA_ACC) * ay2;
    float az = ALPHA_ACC * az1 + (1.0f - ALPHA_ACC) * az2;

    // 重新归一化
    float mag = sqrtf(ax*ax + ay*ay + az*az);
    if (mag > 0.01f) {
        ax /= mag; ay /= mag; az /= mag;
    }

    /* ========== Step 2: 陀螺仪方差在线估计与动态加权 ========== */

    // 更新循环缓冲区
    for (int i = 0; i < 2; i++) {
        int idx = 0;
        if (i == 0) {
            // IMU1
            fusion->gyro_buf[i][fusion->gyro_buf_idx[i]][0] = gx1;
            fusion->gyro_buf[i][fusion->gyro_buf_idx[i]][1] = gy1;
            fusion->gyro_buf[i][fusion->gyro_buf_idx[i]][2] = gz1;
        } else {
            // IMU2
            fusion->gyro_buf[i][fusion->gyro_buf_idx[i]][0] = gx2;
            fusion->gyro_buf[i][fusion->gyro_buf_idx[i]][1] = gy2;
            fusion->gyro_buf[i][fusion->gyro_buf_idx[i]][2] = gz2;
        }

        fusion->gyro_buf_idx[i] = (fusion->gyro_buf_idx[i] + 1) % 100;
        if (fusion->gyro_buf_filled[i] < 100) {
            fusion->gyro_buf_filled[i]++;
        }

        // 当缓冲区满时，计算方差
        if (fusion->gyro_buf_filled[i] == 100) {
            // 提取数据
            float gx_data[100], gy_data[100], gz_data[100];
            for (int j = 0; j < 100; j++) {
                gx_data[j] = fusion->gyro_buf[i][j][0];
                gy_data[j] = fusion->gyro_buf[i][j][1];
                gz_data[j] = fusion->gyro_buf[i][j][2];
            }

            fusion->var_gx[i] = calculate_variance(gx_data, 100);
            fusion->var_gy[i] = calculate_variance(gy_data, 100);
            fusion->var_gz[i] = calculate_variance(gz_data, 100);
        }
    }

    // 计算方差倒数并加权
    float inv_var_gx0 = 1.0f / (fusion->var_gx[0] + 1e-9f);
    float inv_var_gy0 = 1.0f / (fusion->var_gy[0] + 1e-9f);
    float inv_var_gz0 = 1.0f / (fusion->var_gz[0] + 1e-9f);
    float inv_var_gx1 = 1.0f / (fusion->var_gx[1] + 1e-9f);
    float inv_var_gy1 = 1.0f / (fusion->var_gy[1] + 1e-9f);
    float inv_var_gz1 = 1.0f / (fusion->var_gz[1] + 1e-9f);

    float w1_total_inv_var = inv_var_gx0 + inv_var_gy0 + inv_var_gz0;
    float w2_total_inv_var = inv_var_gx1 + inv_var_gy1 + inv_var_gz1;
    float total_inv_var = w1_total_inv_var + w2_total_inv_var;

    float w1, w2;
    if (total_inv_var < 1e-9f) {
        w1 = 0.5f;
    } else {
        w1 = w1_total_inv_var / total_inv_var;
    }
    w2 = 1.0f - w1;

    // 动态调整权重
    float acc_error = fabsf(mag - 1.0f);
    if (acc_error > 0.3f) {  // 剧烈运动
        if (w1_total_inv_var > w2_total_inv_var) {
            w1 = WEIGHT_DYNAMIC;
        } else {
            w1 = 1.0f - WEIGHT_DYNAMIC;
        }
        w2 = 1.0f - w1;
    }

    // 陀螺仪零偏修正与加权融合
    float gx1_corrected = gx1 - fusion->bias_gx[0];
    float gy1_corrected = gy1 - fusion->bias_gy[0];
    float gz1_corrected = gz1 - fusion->bias_gz[0];

    float gx2_corrected = gx2 - fusion->bias_gx[1];
    float gy2_corrected = gy2 - fusion->bias_gy[1];
    float gz2_corrected = gz2 - fusion->bias_gz[1];

    float gx = w1 * gx1_corrected + w2 * gx2_corrected;
    float gy = w1 * gy1_corrected + w2 * gy2_corrected;
    float gz = w1 * gz1_corrected + w2 * gz2_corrected;

    /* ========== Step 3: Mahony 四元数更新 ========== */

    // 转换为弧度
    gx = gx * M_PI / 180.0f;
    gy = gy * M_PI / 180.0f;
    gz = gz * M_PI / 180.0f;

    float q0 = fusion->quat.q0;
    float q1 = fusion->quat.q1;
    float q2 = fusion->quat.q2;
    float q3 = fusion->quat.q3;

    // 重力预测与误差
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = ay*vz - az*vy;
    float ey = az*vx - ax*vz;
    float ez = ax*vy - ay*vx;

    // 动态 Kp (翻板/翻转时信任陀螺仪)
    float Kp = 2.0f + 25.0f * acc_error;
    gx += Kp * ex;
    gy += Kp * ey;
    gz += Kp * ez;

    // 四元数积分
    float dq0 = 0.5f * dt * (-q1*gx - q2*gy - q3*gz);
    float dq1 = 0.5f * dt * ( q0*gx + q2*gz - q3*gy);
    float dq2 = 0.5f * dt * ( q0*gy - q1*gz + q3*gx);
    float dq3 = 0.5f * dt * ( q0*gz + q1*gy - q2*gx);

    q0 += dq0;
    q1 += dq1;
    q2 += dq2;
    q3 += dq3;

    // 归一化
    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (norm > 1e-9f) {
        q0 /= norm;
        q1 /= norm;
        q2 /= norm;
        q3 /= norm;
    }

    fusion->quat.q0 = q0;
    fusion->quat.q1 = q1;
    fusion->quat.q2 = q2;
    fusion->quat.q3 = q3;

    // 转换为欧拉角
    quat_to_euler(&fusion->quat, euler);

    return 0;
}

void imu_fusion_reset(imu_fusion_t *fusion)
{
    if (fusion == NULL) {
        return;
    }

    imu_fusion_init(fusion);
}

quaternion_t imu_fusion_get_quaternion(imu_fusion_t *fusion)
{
    quaternion_t q = {1.0f, 0.0f, 0.0f, 0.0f};

    if (fusion != NULL) {
        q = fusion->quat;
    }

    return q;
}

void quat_to_euler(const quaternion_t *quat, euler_angle_t *euler)
{
    if (quat == NULL || euler == NULL) {
        return;
    }

    float q0 = quat->q0;
    float q1 = quat->q1;
    float q2 = quat->q2;
    float q3 = quat->q3;

    // 四元数转欧拉角 (ZYX 顺序)
    float roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;
    float pitch = asinf(sinp);
    float yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2*q2 + q3*q3));

    euler->roll = roll * 180.0f / M_PI;
    euler->pitch = pitch * 180.0f / M_PI;
    euler->yaw = yaw * 180.0f / M_PI;
}
