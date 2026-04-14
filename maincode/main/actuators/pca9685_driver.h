/**
 * @file pca9685_driver.h
 * @brief PCA9685 16-通道 PWM 驱动
 * 
 * 用于控制伺服电机和 ESC
 */

#ifndef PCA9685_DRIVER_H
#define PCA9685_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include <stdint.h>

/* ========== PCA9685 寄存器地址 ========== */
#define PCA9685_MODE1           0x00
#define PCA9685_MODE2           0x01
#define PCA9685_SUBADR1         0x02
#define PCA9685_SUBADR2         0x03
#define PCA9685_SUBADR3         0x04
#define PCA9685_ALLLED_ON_L     0xFA
#define PCA9685_ALLLED_ON_H     0xFB
#define PCA9685_ALLLED_OFF_L    0xFC
#define PCA9685_ALLLED_OFF_H    0xFD
#define PCA9685_PRESCALE        0xFE

/* ========== 数据结构体 ========== */

/**
 * @brief PCA9685 PWM 驱动实例
 */
typedef struct {
    uint8_t address;                ///< I2C 地址 (通常 0x40)
    uint16_t frequency;             ///< PWM 频率 (Hz)
    uint16_t pulse_values[16];      ///< 当前每个通道的脉宽值
} pca9685_t;

/* ========== 函数声明 ========== */

/**
 * @brief 初始化 PCA9685
 * 
 * @param pca PCA9685 实例指针
 * @param address I2C 地址 (通常 0x40)
 * @param frequency PWM 频率 (Hz，通常 50 Hz 用于舵机/ESC)
 * @return ESP_OK 表示成功
 */
esp_err_t pca9685_init(pca9685_t *pca, uint8_t address, uint16_t frequency);

/**
 * @brief 设置 PWM 频率
 * 
 * @param pca PCA9685 实例指针
 * @param frequency 频率 (Hz，范围 24-1526)
 * @return ESP_OK 表示成功
 */
esp_err_t pca9685_set_frequency(pca9685_t *pca, uint16_t frequency);

/**
 * @brief 设置单个通道的脉宽 (微秒)
 * 
 * 用于舵机和 ESC，脉宽范围 1000-2000 μs
 * 
 * @param pca PCA9685 实例指针
 * @param channel 通道编号 (0-15)
 * @param pulse_us 脉宽 (微秒)
 * @return ESP_OK 表示成功
 */
esp_err_t pca9685_set_pulse(pca9685_t *pca, uint8_t channel, uint16_t pulse_us);

/**
 * @brief 设置单个通道的占空比
 * 
 * @param pca PCA9685 实例指针
 * @param channel 通道编号 (0-15)
 * @param duty_percent 占空比 (0.0-100.0)
 * @return ESP_OK 表示成功
 */
esp_err_t pca9685_set_duty(pca9685_t *pca, uint8_t channel, float duty_percent);

/**
 * @brief 同时设置两个电机通道的脉宽
 * 
 * @param pca PCA9685 实例指针
 * @param channel_left 左电机通道
 * @param pulse_left 左电机脉宽
 * @param channel_right 右电机通道
 * @param pulse_right 右电机脉宽
 * @param invert_left 左电机脉宽是否反转 (1 = 反转)
 * @return ESP_OK 表示成功
 */
esp_err_t pca9685_set_dual_motors(pca9685_t *pca,
                                  uint8_t channel_left, uint16_t pulse_left,
                                  uint8_t channel_right, uint16_t pulse_right,
                                  int invert_left);

/**
 * @brief 获取通道的当前脉宽值
 * 
 * @param pca PCA9685 实例指针
 * @param channel 通道编号 (0-15)
 * @return 脉宽值 (微秒)
 */
uint16_t pca9685_get_pulse(pca9685_t *pca, uint8_t channel);

/**
 * @brief 紧急停止 - 设置所有通道为中立脉宽
 * 
 * @param pca PCA9685 实例指针
 * @param neutral_pulse 中立脉宽 (通常 1500 μs)
 * @return ESP_OK 表示成功
 */
esp_err_t pca9685_emergency_stop(pca9685_t *pca, uint16_t neutral_pulse);

/**
 * @brief 重置 PCA9685 设备
 * 
 * @param pca PCA9685 实例指针
 * @return ESP_OK 表示成功
 */
esp_err_t pca9685_reset(pca9685_t *pca);

#ifdef __cplusplus
}
#endif

#endif // PCA9685_DRIVER_H
