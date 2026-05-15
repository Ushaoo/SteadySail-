#ifndef GPS_NMEA_H
#define GPS_NMEA_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 最近一次解析到的 GPS 数据快照
typedef struct {
    double   lat_deg;          // 纬度，度（北正南负）
    double   lon_deg;          // 经度，度（东正西负）
    float    speed_kmh;        // 地速 km/h
    bool     fix_valid;        // 是否 A 状态（有效定位）
    uint32_t fix_timestamp_ms; // esp_timer_get_time()/1000 时刻
} gps_data_t;

// 启动 GPS 驱动（初始化 UART + 创建读任务）
void gps_nmea_start(void);

// 拷贝最近一次 GPS 数据。返回 false 表示从未收到过有效帧。
bool gps_get_latest(gps_data_t *out);

#ifdef __cplusplus
}
#endif

#endif // GPS_NMEA_H
