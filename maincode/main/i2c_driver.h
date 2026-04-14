/**
 * @file i2c_driver.h
 * @brief ESP32 I2C 驱动程序（使用旧的 i2c API，与 ESP-IDF v6.0 兼容）
 * 
 * 提供基础的 I2C 读写操作，用于与 MPU6050 和 PCA9685 通信
 */

#ifndef I2C_DRIVER_H
#define I2C_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#include "driver/i2c.h"
#pragma GCC diagnostic pop
#include "esp_err.h"
#include <stdint.h>

/* ========== I2C 驱动结构体 ========== */

/**
 * @brief I2C 驱动配置结构体
 */
typedef struct {
    i2c_port_t port;                ///< I2C 端口 (I2C_NUM_0 或 I2C_NUM_1)
    int sda_pin;                    ///< SDA 引脚编号
    int scl_pin;                    ///< SCL 引脚编号
    uint32_t freq_hz;               ///< I2C 频率 (Hz)
    uint32_t timeout_ms;            ///< I2C 操作超时时间 (ms)
} imu_i2c_config_t;

/**
 * @brief I2C 传输结构体
 */
typedef struct {
    uint8_t device_address;     ///< 设备 I2C 地址
    uint8_t register_addr;      ///< 寄存器地址
    uint8_t *data;              ///< 数据指针
    uint16_t data_len;          ///< 数据长度
} i2c_transfer_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 I2C 驱动
 * 
 * 使用 config.h 中定义的硬编码引脚和参数
 * @return ESP_OK 表示成功，其他值表示失败
 */
esp_err_t i2c_driver_init(void);

/**
 * @brief 从指定地址的设备读取单个寄存器
 * 
 * @param device_address I2C 设备地址
 * @param register_addr 寄存器地址
 * @param data 用于存储读取数据的字节指针
 * @return ESP_OK 表示成功
 */
esp_err_t i2c_read_byte(uint8_t device_address, uint8_t register_addr, uint8_t *data);

/**
 * @brief 从指定地址的设备读取多个字节
 * 
 * @param device_address I2C 设备地址
 * @param register_addr 起始寄存器地址
 * @param data 数据缓冲区
 * @param data_len 要读取的字节数
 * @return ESP_OK 表示成功
 */
esp_err_t i2c_read_bytes(uint8_t device_address, uint8_t register_addr, uint8_t *data, uint16_t data_len);

/**
 * @brief 向指定地址的设备写入单个字节
 * 
 * @param device_address I2C 设备地址
 * @param register_addr 寄存器地址
 * @param data 要写入的字节值
 * @return ESP_OK 表示成功
 */
esp_err_t i2c_write_byte(uint8_t device_address, uint8_t register_addr, uint8_t data);

/**
 * @brief 向指定地址的设备写入多个字节
 * 
 * @param device_address I2C 设备地址
 * @param register_addr 起始寄存器地址
 * @param data 数据缓冲区
 * @param data_len 要写入的字节数
 * @return ESP_OK 表示成功
 */
esp_err_t i2c_write_bytes(uint8_t device_address, uint8_t register_addr, const uint8_t *data, uint16_t data_len);

/**
 * @brief 扫描 I2C 总线上的所有设备
 * 
 * 用于调试，检测连接到 I2C 总线上的所有设备
 * 
 * @return 找到的设备数量
 */
int i2c_scan_devices(void);

/**
 * @brief 关闭 I2C 驱动
 * 
 * @return ESP_OK 表示成功
 */
esp_err_t i2c_driver_deinit(void);

#ifdef __cplusplus
}
#endif

#endif // I2C_DRIVER_H

