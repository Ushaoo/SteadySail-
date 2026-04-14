/**
 * @file pca9685_driver.c
 * @brief PCA9685 PWM 驱动实现
 */

#include "pca9685_driver.h"
#include "i2c_driver.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "PCA9685";

/**
 * @brief 写入 LED 通道的 PWM 值
 */
static esp_err_t pca9685_write_pwm(pca9685_t *pca, uint8_t channel, uint16_t on, uint16_t off)
{
    if (channel >= 16) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t led_on_l = PCA9685_MODE1 + 6 + (channel * 4);
    uint8_t led_on_h = PCA9685_MODE1 + 7 + (channel * 4);
    uint8_t led_off_l = PCA9685_MODE1 + 8 + (channel * 4);
    uint8_t led_off_h = PCA9685_MODE1 + 9 + (channel * 4);

    uint8_t data[4];
    data[0] = on & 0xFF;
    data[1] = (on >> 8) & 0xFF;
    data[2] = off & 0xFF;
    data[3] = (off >> 8) & 0xFF;

    esp_err_t ret;
    ret = i2c_write_byte(pca->address, led_on_l, data[0]);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(pca->address, led_on_h, data[1]);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(pca->address, led_off_l, data[2]);
    if (ret != ESP_OK) return ret;
    ret = i2c_write_byte(pca->address, led_off_h, data[3]);

    return ret;
}

esp_err_t pca9685_init(pca9685_t *pca, uint8_t address, uint16_t frequency)
{
    if (pca == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    pca->address = address;
    pca->frequency = frequency;

    // 初始化所有通道脉宽为中立值 (1500 μs)
    for (int i = 0; i < 16; i++) {
        pca->pulse_values[i] = 1500;
    }

    // 重置设备
    esp_err_t ret = i2c_write_byte(address, PCA9685_MODE1, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize PCA9685 at 0x%02X", address);
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // 设置 PWM 频率
    ret = pca9685_set_frequency(pca, frequency);
    if (ret != ESP_OK) {
        return ret;
    }

    // 设置 MODE2 (推挽输出)
    i2c_write_byte(address, PCA9685_MODE2, 0x04);

    ESP_LOGI(TAG, "PCA9685 @ 0x%02X initialized (freq=%u Hz)", address, frequency);
    return ESP_OK;
}

esp_err_t pca9685_set_frequency(pca9685_t *pca, uint16_t frequency)
{
    if (pca == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 计算 prescale 值
    // prescale = round(25MHz / (4096 * freq)) - 1
    uint16_t prescale_value = (uint16_t)round(25000000.0f / (4096.0f * frequency)) - 1;

    // 限制范围
    if (prescale_value < 0x03) {
        prescale_value = 0x03;
    } else if (prescale_value > 0xFF) {
        prescale_value = 0xFF;
    }

    // 读取当前 MODE1
    uint8_t old_mode;
    esp_err_t ret = i2c_read_byte(pca->address, PCA9685_MODE1, &old_mode);
    if (ret != ESP_OK) {
        return ret;
    }

    // 设置睡眠位以改变 prescale
    uint8_t new_mode = (old_mode & 0x7F) | 0x10;  // 清除 RESTART，设置 SLEEP
    ret = i2c_write_byte(pca->address, PCA9685_MODE1, new_mode);
    if (ret != ESP_OK) {
        return ret;
    }

    // 写入 prescale 值
    ret = i2c_write_byte(pca->address, PCA9685_PRESCALE, (uint8_t)prescale_value);
    if (ret != ESP_OK) {
        return ret;
    }

    // 恢复 MODE1
    ret = i2c_write_byte(pca->address, PCA9685_MODE1, old_mode);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(5));

    // 设置 RESTART 位
    ret = i2c_write_byte(pca->address, PCA9685_MODE1, old_mode | 0x80);
    if (ret != ESP_OK) {
        return ret;
    }

    pca->frequency = frequency;
    ESP_LOGI(TAG, "PWM frequency set to %u Hz (prescale=%u)", frequency, prescale_value);

    return ESP_OK;
}

esp_err_t pca9685_set_pulse(pca9685_t *pca, uint8_t channel, uint16_t pulse_us)
{
    if (pca == NULL || channel >= 16) {
        return ESP_ERR_INVALID_ARG;
    }

    // 计算对应的 12-bit 分度值
    // duty = (pulse_us / 20000) * 4096 (对于 50Hz)
    // 或更通用: duty = (pulse_us * freq / 1000000) * 4096
    uint16_t duty = (uint16_t)((pulse_us * pca->frequency / 1000000.0f) * 4096.0f);

    // 限制范围
    if (duty > 4095) {
        duty = 4095;
    }

    // ON 时间通常从 0 开始，OFF 时间为 duty
    uint16_t on = 0;
    uint16_t off = duty;

    esp_err_t ret = pca9685_write_pwm(pca, channel, on, off);
    if (ret != ESP_OK) {
        return ret;
    }

    pca->pulse_values[channel] = pulse_us;
    return ESP_OK;
}

esp_err_t pca9685_set_duty(pca9685_t *pca, uint8_t channel, float duty_percent)
{
    if (pca == NULL || channel >= 16 || duty_percent < 0.0f || duty_percent > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t duty = (uint16_t)((duty_percent / 100.0f) * 4095.0f);
    uint16_t on = 0;
    uint16_t off = duty;

    return pca9685_write_pwm(pca, channel, on, off);
}

esp_err_t pca9685_set_dual_motors(pca9685_t *pca,
                                  uint8_t channel_left, uint16_t pulse_left,
                                  uint8_t channel_right, uint16_t pulse_right,
                                  int invert_left)
{
    if (pca == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 应用左电机反转
    uint16_t pulse_left_actual = invert_left ? (3000 - pulse_left) : pulse_left;

    // 设置左电机
    esp_err_t ret = pca9685_set_pulse(pca, channel_left, pulse_left_actual);
    if (ret != ESP_OK) {
        return ret;
    }

    // 设置右电机
    ret = pca9685_set_pulse(pca, channel_right, pulse_right);
    if (ret != ESP_OK) {
        return ret;
    }

    return ESP_OK;
}

uint16_t pca9685_get_pulse(pca9685_t *pca, uint8_t channel)
{
    if (pca == NULL || channel >= 16) {
        return 0;
    }
    return pca->pulse_values[channel];
}

esp_err_t pca9685_emergency_stop(pca9685_t *pca, uint16_t neutral_pulse)
{
    if (pca == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "EMERGENCY STOP - Setting all channels to neutral");

    // 设置所有通道为中立
    for (int i = 0; i < 16; i++) {
        esp_err_t ret = pca9685_set_pulse(pca, i, neutral_pulse);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to set channel %d to neutral", i);
            return ret;
        }
    }

    return ESP_OK;
}

esp_err_t pca9685_reset(pca9685_t *pca)
{
    if (pca == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 重置设备
    i2c_write_byte(pca->address, PCA9685_MODE1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    // 重新初始化
    return pca9685_init(pca, pca->address, pca->frequency);
}
