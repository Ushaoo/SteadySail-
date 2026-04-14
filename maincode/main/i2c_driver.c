/**
 * @file i2c_driver.c
 * @brief ESP32 I2C 驱动实现（使用旧的 i2c API，与 ESP-IDF v6.0 兼容）
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"

#include "i2c_driver.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

static const char *TAG = "I2C_DRIVER";

static i2c_port_t g_i2c_port = -1;

/**
 * @brief 初始化 I2C 驱动
 */
esp_err_t i2c_driver_init(void)
{
    g_i2c_port = I2C_PORT;

    // 配置 I2C 
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_FREQ_HZ,
        },
        .clk_flags = 0,
    };

    esp_err_t ret = i2c_param_config(I2C_PORT, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C parameter configuration failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "I2C initialized: port=%d, SDA=%d, SCL=%d, freq=%lu Hz",
             I2C_PORT, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

    return ESP_OK;
}

/**
 * @brief 从设备读取单个字节
 */
esp_err_t i2c_read_byte(uint8_t device_address, uint8_t register_addr, uint8_t *data)
{
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    return i2c_read_bytes(device_address, register_addr, data, 1);
}

/**
 * @brief 从设备读取多个字节
 */
esp_err_t i2c_read_bytes(uint8_t device_address, uint8_t register_addr, uint8_t *data, uint16_t data_len)
{
    if (data == NULL || data_len == 0 || g_i2c_port < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 使用标准的I2C写-读操作
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // 写：发送寄存器地址
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, register_addr, true);
    
    // 读：读取数据
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_READ, true);
    if (data_len > 1) {
        i2c_master_read(cmd, data, data_len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[data_len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(g_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C read failed from addr=0x%02X reg=0x%02X: %s",
                 device_address, register_addr, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 向设备写入单个字节
 */
esp_err_t i2c_write_byte(uint8_t device_address, uint8_t register_addr, uint8_t data)
{
    return i2c_write_bytes(device_address, register_addr, &data, 1);
}

/**
 * @brief 向设备写入多个字节
 */
esp_err_t i2c_write_bytes(uint8_t device_address, uint8_t register_addr, const uint8_t *data, uint16_t data_len)
{
    if (data == NULL || data_len == 0 || g_i2c_port < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    // 准备发送数据：寄存器地址 + 数据
    uint8_t write_buf[256];
    if (data_len >= 255) {
        return ESP_ERR_INVALID_ARG;  // 缓冲区太小
    }
    
    write_buf[0] = register_addr;
    memcpy(&write_buf[1], data, data_len);

    // 使用标准的I2C写操作
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_address << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, write_buf, data_len + 1, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(g_i2c_port, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "I2C write failed to addr=0x%02X reg=0x%02X: %s",
                 device_address, register_addr, esp_err_to_name(ret));
    }

    return ret;
}

/**
 * @brief 扫描 I2C 总线
 */
int i2c_scan_devices(void)
{
    if (g_i2c_port < 0) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return 0;
    }

    ESP_LOGI(TAG, "Scanning I2C bus for devices...");

    int device_count = 0;

    for (int i = 0; i < 128; i++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(g_i2c_port, cmd, 50 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Found device at address 0x%02X", i);
            device_count++;
        }
    }

    ESP_LOGI(TAG, "Total devices found: %d", device_count);
    return device_count;
}

/**
 * @brief 关闭 I2C 驱动
 */
esp_err_t i2c_driver_deinit(void)
{
    if (g_i2c_port < 0) {
        return ESP_OK;
    }

    esp_err_t ret = i2c_driver_delete(g_i2c_port);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver deletion failed: %s", esp_err_to_name(ret));
        return ret;
    }

    g_i2c_port = -1;
    ESP_LOGI(TAG, "I2C driver deinitialized");
    
    #pragma GCC diagnostic pop
    
    return ESP_OK;
}
