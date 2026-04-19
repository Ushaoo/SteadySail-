#include <inttypes.h>
#include <stdio.h>

#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ESP32S3_TEST";

static void log_chip_summary(void)
{
    esp_chip_info_t chip_info;
    uint32_t flash_size = 0;

    esp_chip_info(&chip_info);
    esp_flash_get_size(NULL, &flash_size);

    ESP_LOGI(TAG, "Starting ESP32-S3 self-test");
    ESP_LOGI(TAG, "Model: %s", CONFIG_IDF_TARGET);
    ESP_LOGI(TAG, "Cores: %d", chip_info.cores);
    ESP_LOGI(TAG, "Revision: %d", chip_info.revision);
    ESP_LOGI(TAG, "Flash size: %" PRIu32 " MB", flash_size / (1024 * 1024));
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "Minimum free heap: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

    if (chip_info.features & CHIP_FEATURE_WIFI_BGN) {
        ESP_LOGI(TAG, "Wi-Fi feature detected");
    }

    if (chip_info.features & CHIP_FEATURE_BLE) {
        ESP_LOGI(TAG, "BLE feature detected");
    }
}

void run_esp32s3_test(void)
{
    int heartbeat = 0;

    log_chip_summary();

    while (1) {
        int64_t uptime_ms = esp_timer_get_time() / 1000;

        ESP_LOGI(TAG, "Heartbeat %d, uptime=%" PRIi64 " ms, free_heap=%" PRIu32 " bytes",
                 heartbeat,
                 uptime_ms,
                 esp_get_free_heap_size());

        heartbeat++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}