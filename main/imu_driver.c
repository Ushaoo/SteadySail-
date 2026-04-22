#include "imu_driver.h"
#include "system_config.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "IMU_DRIVER";

#define MPU6050_ADDR          0x68
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_ACCEL_XOUT_H  0x3B
#define I2C_MASTER_FREQ_HZ    400000

// 局部函数：唤醒 MPU6050
static esp_err_t mpu6050_wake_up(i2c_port_t i2c_num) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, true);
    i2c_master_write_byte(cmd, 0x00, true); // 写入0唤醒
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

esp_err_t imu_driver_init(void) {
    // 1. 初始化 I2C 0 (IMU 1) - 总是需要
    i2c_config_t conf0 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C0_SDA,
        .scl_io_num = PIN_I2C0_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_0, &conf0);
    i2c_driver_install(I2C_NUM_0, conf0.mode, 0, 0, 0);

#if USE_DUAL_IMU
    // 2. 初始化 I2C 1 (IMU 2) - 仅在双IMU模式
    i2c_config_t conf1 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C1_SDA,
        .scl_io_num = PIN_I2C1_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_1, &conf1);
    i2c_driver_install(I2C_NUM_1, conf1.mode, 0, 0, 0);

    // 3. 唤醒芯片
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t err1 = mpu6050_wake_up(I2C_NUM_0);
    esp_err_t err2 = mpu6050_wake_up(I2C_NUM_1);

    if (err1 == ESP_OK && err2 == ESP_OK) {
        ESP_LOGI(TAG, "双 MPU6050 初始化成功！");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "初始化失败 -> IMU1: %d, IMU2: %d", err1, err2);
        return ESP_FAIL;
    }
#else
    // 单IMU模式：仅初始化 I2C0
    vTaskDelay(pdMS_TO_TICKS(50));
    esp_err_t err1 = mpu6050_wake_up(I2C_NUM_0);

    if (err1 == ESP_OK) {
        ESP_LOGI(TAG, "单 MPU6050 (I2C0) 初始化成功！");
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "初始化失败 -> IMU1: %d", err1);
        return ESP_FAIL;
    }
#endif
}

// 局部函数：读取单个 IMU
static esp_err_t read_single_imu(i2c_port_t i2c_num, imu_data_t *data) {
    uint8_t raw_data[14];
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MPU6050_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, raw_data, 13, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, raw_data + 13, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 20 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / 16384.0f;
        data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / 16384.0f;
        data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / 16384.0f;
        
        data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]) / 131.0f;
        data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]) / 131.0f;
        data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]) / 131.0f;
    }
    return ret;
}

esp_err_t imu_driver_read(dual_imu_data_t *data) {
#if USE_DUAL_IMU
    // 双IMU模式：读取两个传感器
    esp_err_t err1 = read_single_imu(I2C_NUM_0, &data->imu1);
    esp_err_t err2 = read_single_imu(I2C_NUM_1, &data->imu2);
    
    if (err1 == ESP_OK && err2 == ESP_OK) {
        return ESP_OK;
    }
    return ESP_FAIL;
#else
    // 单IMU模式：仅读取 I2C0 的传感器，imu2 保持为零
    esp_err_t err1 = read_single_imu(I2C_NUM_0, &data->imu1);
    
    // imu2 清零（未使用）
    data->imu2.accel_x = 0.0f;
    data->imu2.accel_y = 0.0f;
    data->imu2.accel_z = 0.0f;
    data->imu2.gyro_x = 0.0f;
    data->imu2.gyro_y = 0.0f;
    data->imu2.gyro_z = 0.0f;
    
    return err1;
#endif
}
