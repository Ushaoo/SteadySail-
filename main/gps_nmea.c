/*
 * gps_nmea.c
 *
 * GPS NMEA-0183 driver for ESP32-S3
 * UART1, 9600 baud, 8N1
 * 解析 GPRMC / GNRMC，喂给 anchor_control 模块。
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "gps_nmea.h"
#include "anchor_control.h"
#include "fusion.h"

/* ─── Configuration ────────────────────────────────────────────────────────── */

#define GPS_UART_PORT   UART_NUM_1
#define GPS_BAUD_RATE   9600
#define GPS_RX_GPIO     GPIO_NUM_38   /* GPS TXD → ESP32-S3 GPIO38 */
#define GPS_TX_GPIO     GPIO_NUM_39   /* GPS RXD → ESP32-S3 GPIO39 */

#define GPS_UART_BUF_SIZE   1024
#define GPS_LINE_BUF_SIZE   128
#define GPS_TASK_STACK_SIZE 4096
#define GPS_TASK_PRIORITY   5

static const char *TAG = "GPS";

/* ─── 全局快照 ──────────────────────────────────────────────────────────────── */

static SemaphoreHandle_t s_gps_mutex = NULL;
static gps_data_t        s_latest    = {0};
static bool              s_have_any  = false;

/* ─── Helpers ───────────────────────────────────────────────────────────────── */

static bool nmea_checksum_ok(const char *sentence)
{
    const char *star = strchr(sentence, '*');
    if (star == NULL || strlen(star) < 3) {
        return false;
    }
    char hex[3] = { star[1], star[2], '\0' };
    uint8_t expected = (uint8_t)strtol(hex, NULL, 16);

    uint8_t calc = 0;
    const char *p = sentence;
    if (*p == '$') p++;
    for (; *p != '*' && *p != '\0'; p++) {
        calc ^= (uint8_t)*p;
    }
    return calc == expected;
}

static double nmea_to_decimal_degrees(const char *coord, char hemisphere)
{
    if (coord == NULL || coord[0] == '\0') return 0.0;
    double raw = atof(coord);
    int degrees = (int)(raw / 100);
    double minutes = raw - degrees * 100.0;
    double decimal = degrees + minutes / 60.0;
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    return decimal;
}

static void parse_rmc(const char *sentence)
{
    char buf[GPS_LINE_BUF_SIZE];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    static TickType_t last_no_fix_log = 0;

    const int MAX_FIELDS = 13;
    char *fields[MAX_FIELDS];
    int   nfields = 0;

    char *p = buf;
    fields[nfields++] = p;
    while (*p && nfields < MAX_FIELDS) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[nfields++] = p + 1;
        }
        p++;
    }

    if (nfields < 10) return;

    const char *utc_time = fields[1];
    const char *status   = fields[2];
    const char *lat_str  = fields[3];
    const char  lat_hem  = fields[4][0] ? fields[4][0] : '?';
    const char *lon_str  = fields[5];
    const char  lon_hem  = fields[6][0] ? fields[6][0] : '?';
    const char *speed    = fields[7];
    const char *date     = fields[9];

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    if (status[0] == 'A') {
        double lat = nmea_to_decimal_degrees(lat_str, lat_hem);
        double lon = nmea_to_decimal_degrees(lon_str, lon_hem);
        double spd_kmh = atof(speed) * 1.852;

        char time_fmt[16] = "--:--:--";
        if (strlen(utc_time) >= 6) {
            snprintf(time_fmt, sizeof(time_fmt), "%c%c:%c%c:%c%c",
                     utc_time[0], utc_time[1],
                     utc_time[2], utc_time[3],
                     utc_time[4], utc_time[5]);
        }
        char date_fmt[16] = "20--/--/--";
        if (strlen(date) >= 6) {
            snprintf(date_fmt, sizeof(date_fmt), "20%c%c-%c%c-%c%c",
                     date[4], date[5],
                     date[2], date[3],
                     date[0], date[1]);
        }

        ESP_LOGI(TAG, "Fix: %s %s  Lat: %.6f  Lon: %.6f  Speed: %.1f km/h",
                 date_fmt, time_fmt, lat, lon, spd_kmh);

        // ===== 写入全局快照 =====
        gps_data_t snap = {
            .lat_deg          = lat,
            .lon_deg          = lon,
            .speed_kmh        = (float)spd_kmh,
            .fix_valid        = true,
            .fix_timestamp_ms = now_ms,
        };
        if (s_gps_mutex) {
            xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
            s_latest   = snap;
            s_have_any = true;
            xSemaphoreGive(s_gps_mutex);
        }

        // ===== 喂给虚拟锚点模块 =====
        anchor_gps_sample_t a = {
            .lat          = lat,
            .lon          = lon,
            .speed_kmh    = (float)spd_kmh,
            .fix_valid    = true,
            .timestamp_ms = now_ms,
        };
        anchor_feed_gps(&a);
        // ===== 喂给传感器融合模块 =====
        fusion_gps_in_t f = {
            .lat          = lat,
            .lon          = lon,
            .speed_mps    = (float)(spd_kmh / 3.6),
            .fix_valid    = true,
            .timestamp_ms = now_ms,
        };
        fusion_feed_gps(&f);    } else {
        TickType_t now = xTaskGetTickCount();
        if ((now - last_no_fix_log) * portTICK_PERIOD_MS >= 5000) {
            ESP_LOGW(TAG, "No fix – searching for satellites (move to open sky)");
            last_no_fix_log = now;
        }
        // 即使无 fix 也通知 anchor，让它的看门狗按"无 fix"判断超时
        anchor_gps_sample_t a = {
            .fix_valid    = false,
            .timestamp_ms = now_ms,
        };
        anchor_feed_gps(&a);

        fusion_gps_in_t f = {
            .fix_valid    = false,
            .timestamp_ms = now_ms,
        };
        fusion_feed_gps(&f);
    }
}

/* ─── 解析 GGA：提取卫星数 / 定位质量 / HDOP，仅用于打印 ─────────────── */
static void parse_gga(const char *sentence)
{
    char buf[GPS_LINE_BUF_SIZE];
    strncpy(buf, sentence, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    const int MAX_FIELDS = 16;
    char *fields[MAX_FIELDS];
    int   nfields = 0;

    char *p = buf;
    fields[nfields++] = p;
    while (*p && nfields < MAX_FIELDS) {
        if (*p == ',' || *p == '*') {
            *p = '\0';
            fields[nfields++] = p + 1;
        }
        p++;
    }
    if (nfields < 10) return;

    // GGA 字段: 0=$GxGGA 1=UTC 2=lat 3=N/S 4=lon 5=E/W
    //          6=fixQuality(0=无,1=GPS,2=DGPS) 7=satsUsed 8=HDOP 9=alt
    int   fix_quality = atoi(fields[6]);
    int   sats_used   = atoi(fields[7]);
    float hdop        = (float)atof(fields[8]);

    static TickType_t last_log = 0;
    TickType_t now = xTaskGetTickCount();
    if ((now - last_log) * portTICK_PERIOD_MS >= 1000) {  // 1Hz 打印
        ESP_LOGI(TAG, "Sats used: %d   FixQuality: %d   HDOP: %.2f",
                 sats_used, fix_quality, hdop);
        last_log = now;
    }
}

/* ─── UART init ─────────────────────────────────────────────────────────────── */

static void gps_uart_init(void)
{
    const uart_config_t uart_cfg = {
        .baud_rate  = GPS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_param_config(GPS_UART_PORT, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_PORT,
                                 GPS_TX_GPIO,
                                 GPS_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_PORT,
                                        GPS_UART_BUF_SIZE * 2,
                                        0, 0, NULL, 0));

    ESP_LOGI(TAG, "UART%d initialized: %d baud, RX=GPIO%d, TX=GPIO%d",
             GPS_UART_PORT, GPS_BAUD_RATE, GPS_RX_GPIO, GPS_TX_GPIO);
}

/* ─── Reader task ───────────────────────────────────────────────────────────── */

static void gps_read_task(void *arg)
{
    char    line[GPS_LINE_BUF_SIZE];
    int     line_pos    = 0;
    bool    in_sentence = false;
    uint8_t buf[256];

    ESP_LOGI(TAG, "GPS reader task started");

    for (;;) {
        int len = uart_read_bytes(GPS_UART_PORT, buf, sizeof(buf),
                                  pdMS_TO_TICKS(100));
        if (len <= 0) continue;

        for (int i = 0; i < len; i++) {
            char c = (char)buf[i];

            if (c == '$') {
                line[0]     = '$';
                line_pos    = 1;
                in_sentence = true;
            } else if (c == '\r' || c == '\n') {
                if (in_sentence && line_pos > 1) {
                    line[line_pos] = '\0';
                    if (nmea_checksum_ok(line)) {
                        if (strncmp(line, "$GNRMC", 6) == 0 ||
                            strncmp(line, "$GPRMC", 6) == 0) {
                            parse_rmc(line);
                        } else if (strncmp(line, "$GNGGA", 6) == 0 ||
                                   strncmp(line, "$GPGGA", 6) == 0) {
                            parse_gga(line);
                        }
                    }
                }
                line_pos    = 0;
                in_sentence = false;
            } else if (in_sentence) {
                if (line_pos < GPS_LINE_BUF_SIZE - 1) {
                    line[line_pos++] = c;
                } else {
                    in_sentence = false;
                    line_pos    = 0;
                }
            }
        }
    }
}

/* ─── Public API ────────────────────────────────────────────────────────────── */

bool gps_get_latest(gps_data_t *out)
{
    if (!out || !s_gps_mutex) return false;
    bool ok = false;
    xSemaphoreTake(s_gps_mutex, portMAX_DELAY);
    if (s_have_any) { *out = s_latest; ok = true; }
    xSemaphoreGive(s_gps_mutex);
    return ok;
}

void gps_nmea_start(void)
{
    if (!s_gps_mutex) s_gps_mutex = xSemaphoreCreateMutex();
    ESP_LOGI(TAG, "Starting GPS NMEA driver");
    gps_uart_init();
    xTaskCreate(gps_read_task, "gps_read", GPS_TASK_STACK_SIZE, NULL,
                GPS_TASK_PRIORITY, NULL);
}
