#ifndef ANCHOR_CONTROL_H
#define ANCHOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ANCHOR_DISABLED = 0,   // 未启用，正常手动/自动控制
    ANCHOR_SETTING,        // 抛锚瞬间：采样 N 个 GPS fix 求平均
    ANCHOR_HOLDING,        // 在内圈内，怠速漂浮
    ANCHOR_RETURNING,      // 在外圈外，主动驶回
    ANCHOR_LOST_GPS        // GPS 失效，安全停机
} anchor_state_t;

// 一次 GPS 数据样本（由 GPS 驱动模块在 1Hz 解析后调用 anchor_feed_gps 喂进来）
typedef struct {
    double   lat;          // 纬度，度
    double   lon;          // 经度，度
    float    speed_kmh;    // 地速
    bool     fix_valid;    // 至少 2D fix
    uint32_t timestamp_ms; // esp_timer_get_time()/1000 时刻
} anchor_gps_sample_t;

// 初始化（一次性）。在 app_main 里调用。
void anchor_control_init(void);

// GPS 驱动每收到一帧 RMC 后调用，喂入最新位置（无 fix 也调用，仅刷新时间戳即可）
void anchor_feed_gps(const anchor_gps_sample_t *s);

// 开关 / 按钮 / 串口触发
void anchor_set_here(void);   // "抛锚"
void anchor_release(void);    // "起锚"

// 在 100Hz 主控制循环里调用一次。
// 内部会改写 g_forward_thrust，并通过 g_target_heading + g_heading_hold_active 接管转向。
void anchor_control_update(void);

// 状态查询
anchor_state_t anchor_get_state(void);
float          anchor_get_distance_m(void);    // 当前到锚点距离
float          anchor_get_bearing_deg(void);   // 船→锚点 方位角 (0=北, 90=东)

#ifdef __cplusplus
}
#endif

#endif // ANCHOR_CONTROL_H
