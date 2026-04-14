/**
 * @file main.c
 * @brief SteadySail ESP32-S3 主程序入口
 * 
 * 初始化系统、I2C、传感器，启动主控制循环
 */

#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "i2c_driver.h"
#include "control_loop.h"

static const char *TAG = "MAIN";

/**
 * @brief 系统初始化
 */
static esp_err_t system_init(void)
{
    ESP_LOGI(TAG, "=== SteadySail ESP32-S3 System Initialization ===");
    
    // 初始化 I2C
    ESP_LOGI(TAG, "Initializing I2C (SDA=%d, SCL=%d, Freq=%lu Hz)",
             I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    
    esp_err_t ret = i2c_driver_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 扫描 I2C 总线上的设备
    ESP_LOGI(TAG, "Scanning I2C devices...");
    i2c_scan_devices();
    
    // 初始化控制循环 (包含 IMU、PCA9685 等初始化)
    ESP_LOGI(TAG, "Initializing control loop...");
    control_loop_t loop = {0};
    
    ret = control_loop_init(&loop);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Control loop initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 启动控制循环
    ESP_LOGI(TAG, "Starting control loop (100 Hz)...");
    ret = control_loop_start(&loop, 20, 8192);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start control loop: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "=== System initialization complete ===");
    return ESP_OK;
}

/**
 * @brief 应用主函数
 */
void app_main(void)
{
    ESP_LOGI(TAG, "============================================");
    ESP_LOGI(TAG, "SteadySail - Boat Stabilization System");
    ESP_LOGI(TAG, "ESP32-S3 Firmware v1.0");
    ESP_LOGI(TAG, "============================================");
    
    esp_err_t ret = system_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "System initialization failed!");
        while (1) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "System running. Monitoring...");
    
    // 主循环 (可做日志、监控等)
    while (1) {
        // 每 5 秒打印一次统计信息
        vTaskDelay(pdMS_TO_TICKS(5000));
        
        // TODO: 在这里可以打印控制循环的统计信息、IMU 数据等
        ESP_LOGI(TAG, "System heartbeat...");
    }
}
