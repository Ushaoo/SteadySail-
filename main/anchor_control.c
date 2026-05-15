/*
 * anchor_control.c
 *
 * 虚拟锚点（GPS Anchor Hold）控制模块
 *
 * 思路：
 *   - 抛锚时采样 N 个 GPS fix 求平均，确定锚点
 *   - HOLDING：距离 < R_INNER，怠速漂浮
 *   - RETURNING：距离 > R_OUTER，自动选择"船头/船尾朝向锚点"（最短转角）
 *     再用现成的 HEADING_KP/KI 差速 PID 把船转过去；航向对准后给前进/倒车油门
 *   - GPS 5s 无新帧 → LOST_GPS，主推力强制 0
 */

#include "anchor_control.h"
#include "system_config.h"
#include "bno055_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>

static const char *TAG = "ANCHOR";

// ====== 与 main.c 共享的全局量 ======
extern volatile float g_forward_thrust;
extern volatile int   g_turn_state;
extern volatile bool  g_heading_hold_active;
extern volatile float g_target_heading;
extern volatile bool  g_estop_active;

// ====== 模块内部状态 ======
static anchor_state_t s_state = ANCHOR_DISABLED;

// 锚点位置（采样平均后写入）
static double s_anchor_lat = 0.0;
static double s_anchor_lon = 0.0;
static double s_anchor_cos_lat = 1.0;   // 缓存 cos(纬度) 用于经度米换算

// 抛锚时的样本累积
static double  s_set_lat_sum = 0.0;
static double  s_set_lon_sum = 0.0;
static int     s_set_count   = 0;

// 最新 GPS（由 anchor_feed_gps 写入；anchor_control_update 在 100Hz 用）
static double  s_last_lat = 0.0;
static double  s_last_lon = 0.0;
static bool    s_last_valid = false;
static uint32_t s_last_gps_ms = 0;

// 距离/方位（GPS 帧到达时刷新一次，100Hz 循环只读）
static float   s_dist_m_filt    = 0.0f;
static float   s_bearing_deg    = 0.0f;
static bool    s_have_solution  = false;

// 抛锚前保存的用户/控制状态，起锚时还原
static bool    s_saved_heading_active = false;
static float   s_saved_target_heading = 0.0f;

// FORWARD/REVERSE 选择滞回（避免临界角度抖动）
static bool    s_reverse_mode = false;

// ====== 工具函数 ======
#define DEG2RAD   0.01745329251994329577
#define RAD2DEG   57.29577951308232

static inline float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

// 把角度归一化到 (-180, +180]
static inline float wrap180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

// 把角度归一化到 [0, 360)
static inline float wrap360(float a) {
    while (a >= 360.0f) a -= 360.0f;
    while (a <    0.0f) a += 360.0f;
    return a;
}

// 局部 ENU 坐标差（米）。原点 = 锚点。
static void enu_from_anchor(double lat, double lon,
                            float *east_m, float *north_m)
{
    const double R = 6371000.0;
    *east_m  = (float)((lon - s_anchor_lon) * DEG2RAD * R * s_anchor_cos_lat);
    *north_m = (float)((lat - s_anchor_lat) * DEG2RAD * R);
}

// ====== 抛锚 / 起锚 ======
void anchor_control_init(void)
{
    s_state = ANCHOR_DISABLED;
    s_have_solution = false;
    ESP_LOGI(TAG, "anchor module ready");
}

void anchor_set_here(void)
{
    if (s_state != ANCHOR_DISABLED) {
        ESP_LOGW(TAG, "已在锚定状态(%d)，忽略重复抛锚", (int)s_state);
        return;
    }
    if (!s_last_valid) {
        ESP_LOGW(TAG, "拒绝抛锚：当前无有效 GPS fix");
        return;
    }
    s_set_lat_sum = 0.0;
    s_set_lon_sum = 0.0;
    s_set_count   = 0;

    // 保存用户当前的航向锁状态，起锚时恢复
    s_saved_heading_active = g_heading_hold_active;
    s_saved_target_heading = g_target_heading;

    s_state = ANCHOR_SETTING;
    ESP_LOGI(TAG, ">>> 进入 SETTING：开始采样 %d 个 GPS fix 平均",
             ANCHOR_SETTLE_SAMPLES);
}

void anchor_release(void)
{
    if (s_state == ANCHOR_DISABLED) return;
    s_state = ANCHOR_DISABLED;
    s_have_solution = false;
    s_reverse_mode = false;

    // 还原用户的航向锁/差速
    g_heading_hold_active = s_saved_heading_active;
    g_target_heading      = s_saved_target_heading;
    g_forward_thrust      = 0.0f;
    g_turn_state          = 0;
    ESP_LOGI(TAG, "<<< 起锚：已释放控制权");
}

// ====== GPS 输入回调（由 GPS 驱动 1Hz 调用） ======
void anchor_feed_gps(const anchor_gps_sample_t *s)
{
    if (!s) return;
    s_last_gps_ms = s->timestamp_ms;
    if (!s->fix_valid) {
        // 不更新坐标，只刷新时间戳——让 LOST_GPS 看门狗按"无 fix"判断
        return;
    }
    s_last_valid  = true;
    s_last_lat    = s->lat;
    s_last_lon    = s->lon;

    // ---- SETTING：累加平均 ----
    if (s_state == ANCHOR_SETTING) {
        s_set_lat_sum += s->lat;
        s_set_lon_sum += s->lon;
        s_set_count++;
        ESP_LOGI(TAG, "  采样 %d/%d  lat=%.7f lon=%.7f",
                 s_set_count, ANCHOR_SETTLE_SAMPLES, s->lat, s->lon);

        if (s_set_count >= ANCHOR_SETTLE_SAMPLES) {
            s_anchor_lat = s_set_lat_sum / s_set_count;
            s_anchor_lon = s_set_lon_sum / s_set_count;
            s_anchor_cos_lat = cos(s_anchor_lat * DEG2RAD);
            s_state = ANCHOR_HOLDING;
            s_have_solution = false;   // 等下一帧 GPS 算
            ESP_LOGI(TAG, "锚点确定: lat=%.7f lon=%.7f → 进入 HOLDING",
                     s_anchor_lat, s_anchor_lon);
        }
        return;
    }

    // ---- HOLDING / RETURNING / LOST_GPS：刷新距离与方位 ----
    if (s_state == ANCHOR_HOLDING || s_state == ANCHOR_RETURNING ||
        s_state == ANCHOR_LOST_GPS) {

        float east_m, north_m;
        enu_from_anchor(s->lat, s->lon, &east_m, &north_m);

        float dist_now = sqrtf(east_m * east_m + north_m * north_m);

        // 一阶低通，过滤 GPS 抖动 (~0.1m)
        if (!s_have_solution) {
            s_dist_m_filt = dist_now;
        } else {
            s_dist_m_filt = 0.7f * s_dist_m_filt + 0.3f * dist_now;
        }

        // 船→锚点 的方位角：锚点相对船的方向 = (-east, -north)
        // atan2(east, north) 标准方位角约定 (0=N, 90=E)
        s_bearing_deg = wrap360(atan2f(-east_m, -north_m) * RAD2DEG);
        s_have_solution = true;

        if (s_state == ANCHOR_LOST_GPS) {
            ESP_LOGI(TAG, "GPS 已恢复，重新进入 HOLDING");
            s_state = ANCHOR_HOLDING;
        }
    }
}

// ====== 100Hz 主循环调用 ======
void anchor_control_update(void)
{
    if (s_state == ANCHOR_DISABLED || s_state == ANCHOR_SETTING) {
        return; // 不接管控制
    }

    // 急停时让外层先处理
    if (g_estop_active) {
        g_forward_thrust = 0.0f;
        return;
    }

    // ---- GPS 看门狗 ----
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (now_ms - s_last_gps_ms > ANCHOR_GPS_TIMEOUT_MS) {
        if (s_state != ANCHOR_LOST_GPS) {
            ESP_LOGW(TAG, "GPS 超时 %lu ms → LOST_GPS, 停机",
                     (unsigned long)(now_ms - s_last_gps_ms));
        }
        s_state = ANCHOR_LOST_GPS;
        g_forward_thrust      = 0.0f;
        g_heading_hold_active = false;
        g_turn_state          = 0;
        return;
    }

    if (!s_have_solution) {
        g_forward_thrust = 0.0f;
        return;
    }

    // ---- 状态机：HOLDING <-> RETURNING（带滞回） ----
    if (s_state == ANCHOR_HOLDING && s_dist_m_filt > ANCHOR_RADIUS_OUTER_M) {
        s_state = ANCHOR_RETURNING;
        ESP_LOGI(TAG, "距离 %.2fm > %.1fm → RETURNING",
                 s_dist_m_filt, (float)ANCHOR_RADIUS_OUTER_M);
    } else if (s_state == ANCHOR_RETURNING && s_dist_m_filt < ANCHOR_RADIUS_INNER_M) {
        s_state = ANCHOR_HOLDING;
        ESP_LOGI(TAG, "距离 %.2fm < %.1fm → HOLDING",
                 s_dist_m_filt, (float)ANCHOR_RADIUS_INNER_M);
        g_forward_thrust      = 0.0f;
        g_heading_hold_active = false;
        g_turn_state          = 0;
        return;
    }

    if (s_state == ANCHOR_HOLDING) {
        // 死区内，怠速漂浮
        g_forward_thrust      = 0.0f;
        g_heading_hold_active = false;
        g_turn_state          = 0;
        return;
    }

    // ============================================================
    // RETURNING：船头/船尾任意一端朝向锚点（最短转角），分别前进/倒车
    // ============================================================
    float yaw_now = 0.0f;
    if (bno055_get_heading(&yaw_now) != ESP_OK) {
        // 拿不到航向就别乱推，下一帧重试
        g_forward_thrust = 0.0f;
        return;
    }

    float hdg_bow   = s_bearing_deg;                       // 船头朝锚点
    float hdg_stern = wrap360(s_bearing_deg + 180.0f);     // 船尾朝锚点
    float err_bow   = wrap180(hdg_bow   - yaw_now);
    float err_stern = wrap180(hdg_stern - yaw_now);

    // 选择最短转角方案 + 切换滞回（避免临界 ±90° 抖动）
    const float SWITCH_HYST_DEG = 10.0f;
    bool want_reverse = (fabsf(err_stern) + SWITCH_HYST_DEG < fabsf(err_bow));
    bool want_forward = (fabsf(err_bow)   + SWITCH_HYST_DEG < fabsf(err_stern));
    if (want_reverse) s_reverse_mode = true;
    else if (want_forward) s_reverse_mode = false;

    float target_heading_sel = s_reverse_mode ? hdg_stern : hdg_bow;
    float yaw_err_sel        = s_reverse_mode ? err_stern : err_bow;

    // 锁定该航向，让现成 HEADING_KP/KI 差速 PID 把船转过去
    g_target_heading      = target_heading_sel;
    g_heading_hold_active = true;
    g_turn_state          = 0;

    // 航向基本对准后才推 / 倒；否则原地转
    if (fabsf(yaw_err_sel) > ANCHOR_HEADING_TOL_DEG) {
        g_forward_thrust = 0.0f;
    } else {
        float over = s_dist_m_filt - ANCHOR_RADIUS_INNER_M;
        if (over < 0.0f) over = 0.0f;
        float thrust_mag = ANCHOR_DIST_KP * over;
        float cap = s_reverse_mode ? ANCHOR_REVERSE_MAX_PCT : ANCHOR_MAX_THRUST_PCT;
        if (thrust_mag > cap) thrust_mag = cap;
        g_forward_thrust = s_reverse_mode ? -thrust_mag : +thrust_mag;
    }

    // 调试打印（10Hz 节流）
    static uint32_t s_last_log_ms = 0;
    if (now_ms - s_last_log_ms > 100) {
        s_last_log_ms = now_ms;
        ESP_LOGI(TAG, "[RET] dist=%.2fm brg=%.1f yaw=%.1f "
                      "err_bow=%+.1f err_stern=%+.1f -> %s yaw_err=%+.1f thr=%+.1f%%",
                 s_dist_m_filt, s_bearing_deg, yaw_now,
                 err_bow, err_stern,
                 s_reverse_mode ? "REVERSE" : "FORWARD",
                 yaw_err_sel, g_forward_thrust);
    }
}

// ====== 查询 ======
anchor_state_t anchor_get_state(void)        { return s_state; }
float          anchor_get_distance_m(void)   { return s_dist_m_filt; }
float          anchor_get_bearing_deg(void)  { return s_bearing_deg; }
