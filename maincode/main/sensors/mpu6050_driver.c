/**
 * @file mpu6050_driver.c
 * @brief MPU6050 驱动实现
 */

#include "mpu6050_driver.h"
#include "i2c_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "MPU6050";

/**
 * @brief 从 MPU6050 读取 16-bit 数据 (高字节在前)
 */
static int16_t mpu6050_read_word(mpu6050_t *mpu, uint8_t reg_high, uint8_t reg_low)
{
    uint8_t high, low;
    int16_t value;

    if (i2c_read_byte(mpu->address, reg_high, &high) != ESP_OK) {
        return 0;
    }
    if (i2c_read_byte(mpu->address, reg_low, &low) != ESP_OK) {
        return 0;
    }

    value = ((int16_t)high << 8) | low;
    return value;
}

esp_err_t mpu6050_init(mpu6050_t *mpu, uint8_t address)
{
    if (mpu == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    mpu->address = address;
    mpu->accel_scale = 16384.0f;  // 2g 范围
    mpu->gyro_scale = 131.0f;      // 250°/s 范围

    // 唤醒设备 (清除睡眠位)
    esp_err_t ret = i2c_write_byte(address, MPU6050_PWR_MGMT_1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize MPU6050 at 0x%02X", address);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // 设置采样率分频 (采样率 = 1000 / (1 + SMPLRT_DIV))
    i2c_write_byte(address, MPU6050_SMPLRT_DIV, 0x07);  // 125 Hz

    // 设置低通滤波
    i2c_write_byte(address, MPU6050_CONFIG, 0x06);  // 5Hz LPF

    // 设置陀螺仪量程 (±250°/s)
    i2c_write_byte(address, MPU6050_GYRO_CONFIG, 0x00);

    // 设置加速度量程 (±2g)
    i2c_write_byte(address, MPU6050_ACCEL_CONFIG, 0x00);

    ESP_LOGI(TAG, "MPU6050 @ 0x%02X initialized (accel_scale=%.0f, gyro_scale=%.0f)",
             address, mpu->accel_scale, mpu->gyro_scale);
    return ESP_OK;
}

esp_err_t mpu6050_calibrate(mpu6050_t *mpu, uint16_t samples, 
                            float_data_t *gyro_bias, float_data_t *accel_bias)
{
    if (mpu == NULL || gyro_bias == NULL || accel_bias == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Starting MPU6050 calibration with %d samples", samples);

    // 初始化累加器
    float_data_t gyro_sum = {0}, accel_sum = {0};

    // 采样
    for (uint16_t i = 0; i < samples; i++) {
        raw_data_t accel_raw, gyro_raw;

        esp_err_t ret = mpu6050_read_all_raw(mpu, &accel_raw, &gyro_raw);
        if (ret != ESP_OK) {
            continue;
        }

        gyro_sum.x += gyro_raw.x / mpu->gyro_scale;
        gyro_sum.y += gyro_raw.y / mpu->gyro_scale;
        gyro_sum.z += gyro_raw.z / mpu->gyro_scale;

        accel_sum.x += accel_raw.x / mpu->accel_scale;
        accel_sum.y += accel_raw.y / mpu->accel_scale;
        accel_sum.z += accel_raw.z / mpu->accel_scale;

        if ((i + 1) % 50 == 0) {
            ESP_LOGI(TAG, "Calibration: %d/%d samples", i + 1, samples);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 计算平均值
    gyro_bias->x = gyro_sum.x / samples;
    gyro_bias->y = gyro_sum.y / samples;
    gyro_bias->z = gyro_sum.z / samples;

    accel_bias->x = accel_sum.x / samples;
    accel_bias->y = accel_sum.y / samples;
    accel_bias->z = accel_sum.z / samples;

    ESP_LOGI(TAG, "Calibration complete!");
    ESP_LOGI(TAG, "Gyro bias: x=%.4f, y=%.4f, z=%.4f", gyro_bias->x, gyro_bias->y, gyro_bias->z);
    ESP_LOGI(TAG, "Accel bias: x=%.4f, y=%.4f, z=%.4f", accel_bias->x, accel_bias->y, accel_bias->z);

    return ESP_OK;
}

esp_err_t mpu6050_read_accel_raw(mpu6050_t *mpu, raw_data_t *accel)
{
    if (mpu == NULL || accel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    accel->x = mpu6050_read_word(mpu, MPU6050_ACCEL_XOUT_H, MPU6050_ACCEL_XOUT_L);
    accel->y = mpu6050_read_word(mpu, MPU6050_ACCEL_YOUT_H, MPU6050_ACCEL_YOUT_L);
    accel->z = mpu6050_read_word(mpu, MPU6050_ACCEL_ZOUT_H, MPU6050_ACCEL_ZOUT_L);

    return ESP_OK;
}

esp_err_t mpu6050_read_gyro_raw(mpu6050_t *mpu, raw_data_t *gyro)
{
    if (mpu == NULL || gyro == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gyro->x = mpu6050_read_word(mpu, MPU6050_GYRO_XOUT_H, MPU6050_GYRO_XOUT_L);
    gyro->y = mpu6050_read_word(mpu, MPU6050_GYRO_YOUT_H, MPU6050_GYRO_YOUT_L);
    gyro->z = mpu6050_read_word(mpu, MPU6050_GYRO_ZOUT_H, MPU6050_GYRO_ZOUT_L);

    return ESP_OK;
}

esp_err_t mpu6050_read_all_raw(mpu6050_t *mpu, raw_data_t *accel, raw_data_t *gyro)
{
    if (mpu == NULL || accel == NULL || gyro == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = mpu6050_read_accel_raw(mpu, accel);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6050_read_gyro_raw(mpu, gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

esp_err_t mpu6050_read_accel_g(mpu6050_t *mpu, float_data_t *accel)
{
    if (mpu == NULL || accel == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    raw_data_t accel_raw;
    esp_err_t ret = mpu6050_read_accel_raw(mpu, &accel_raw);
    if (ret != ESP_OK) {
        return ret;
    }

    accel->x = accel_raw.x / mpu->accel_scale;
    accel->y = accel_raw.y / mpu->accel_scale;
    accel->z = accel_raw.z / mpu->accel_scale;

    return ESP_OK;
}

esp_err_t mpu6050_read_gyro_dps(mpu6050_t *mpu, float_data_t *gyro)
{
    if (mpu == NULL || gyro == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    raw_data_t gyro_raw;
    esp_err_t ret = mpu6050_read_gyro_raw(mpu, &gyro_raw);
    if (ret != ESP_OK) {
        return ret;
    }

    gyro->x = gyro_raw.x / mpu->gyro_scale;
    gyro->y = gyro_raw.y / mpu->gyro_scale;
    gyro->z = gyro_raw.z / mpu->gyro_scale;

    return ESP_OK;
}

esp_err_t mpu6050_read_temp(mpu6050_t *mpu, float *temp)
{
    if (mpu == NULL || temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    int16_t temp_raw = mpu6050_read_word(mpu, MPU6050_TEMP_OUT_H, MPU6050_TEMP_OUT_L);

    // 温度转换: T (°C) = (TEMP_OUT / 340) + 36.53
    *temp = (temp_raw / 340.0f) + 36.53f;

    return ESP_OK;
}

esp_err_t mpu6050_read_all(mpu6050_t *mpu, float_data_t *accel, float_data_t *gyro, float *temp)
{
    if (mpu == NULL || accel == NULL || gyro == NULL || temp == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = mpu6050_read_accel_g(mpu, accel);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6050_read_gyro_dps(mpu, gyro);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = mpu6050_read_temp(mpu, temp);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

void mpu6050_apply_axis_invert(float_data_t *data, int invert_x, int invert_y, int invert_z)
{
    if (data == NULL) {
        return;
    }

    if (invert_x) data->x = -data->x;
    if (invert_y) data->y = -data->y;
    if (invert_z) data->z = -data->z;
}
