/*
 * fusion.c – GPS + BNO055 传感器融合定位
 *
 * 状态量：
 *   (s_e, s_n)        – ENU 位置（米），相对首帧 GPS fix 原点
 *   (s_vE, s_vN)      – ENU 速度向量（m/s），由 LIA 积分 + GPS 速度校正
 *   s_psi_bias_deg    – BNO055 heading 常值偏置，由 GPS COG 慢学习
 *   (s_ab_x, s_ab_y)  – 加速度计 body 系零偏，由 ZUPT 慢学习
 *
 * GPS 来时（~1 Hz）：互补滤波校正位置；用两帧位移向量校正速度和 heading bias
 * 100 Hz：读 BNO055 LIA → body→ENU 旋转 → 积分 vE/vN → 积分 e/n
 * ZUPT：GPS 速度 < 0.3 m/s 且 |a_body| < 0.15 m/s² → 清速度 + 慢学零偏
 */

#include "fusion.h"
#include "bno055_driver.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>
#include <string.h>

static const char *TAG = "FUSION";

// ====== 滤波参数 ======
#define K_POS          0.20f   // GPS 位置校正增益（每帧 GPS 吸收 20% 误差）
#define K_VEL          0.30f   // GPS 速度向量校正增益
#define K_BIAS_HDG     0.02f   // heading bias 学习率（GPS COG 校正）
#define K_BIAS_ACC     0.001f  // 加速度计零偏学习率（ZUPT 时）
#define COG_MIN_SPD    1.0f    // m/s，低于此速度不用 GPS COG 校正
#define COG_MIN_DISP   0.3f    // m，两帧 GPS 位移 < 此值时不算 COG
#define JUMP_LIMIT_M   8.0f    // GPS 跳点剔除阈值（超过则丢该帧）
#define VEL_DECAY      0.998f  // 速度积分极弱阻尼（防长时间漂移）
#define ZUPT_SPD       0.3f    // m/s，GPS 速度低于此 → 判定静止
#define ZUPT_ACC       0.15f   // m/s²，|a_body| 低于此 → 判定静止
#define DR_MAX_AGE_MS  10000   // 纯 DR 超过此时间 → fusion_is_valid() = false

#define F_DEG2RAD 0.01745329251994329577f
#define F_RAD2DEG 57.29577951308232f
#define EARTH_R   6378137.0    // WGS-84 赤道半径（米）

// ====== 融合状态 ======
static bool    s_inited  = false;

// 参考原点（首次有效 GPS fix 写入后固定）
static double  s_ref_lat = 0.0, s_ref_lon = 0.0;
static double  s_ref_cos = 1.0; // cos(ref_lat)，缓存
static bool    s_ref_set = false;

// ENU 位置（米，相对原点）
static float   s_e = 0.0f, s_n = 0.0f;

// ENU 速度向量（m/s）
static float   s_vE = 0.0f, s_vN = 0.0f;

// BNO055 heading 偏置（度）：psi_fused = yaw_bno + s_psi_bias_deg
static float   s_psi_bias_deg = 0.0f;

// 加速度计 body 系零偏（m/s²）
static float   s_ab_x = 0.0f, s_ab_y = 0.0f;

// 上次 100Hz 预测的微秒时戳
static int64_t s_last_pred_us = 0;

// GPS 输入缓存
static fusion_gps_in_t s_last_gps;
static bool    s_have_gps           = false;
static bool    s_pending_correction = false;
static uint32_t s_last_fix_ms       = 0;

// 上一帧 GPS ENU（用于 COG / 速度向量计算）
static float   s_prev_e = 0.0f, s_prev_n = 0.0f;
static bool    s_prev_have = false;

// ====== 内部工具 ======
static inline float fwrap180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

static inline float fwrap360(float a)
{
    while (a >= 360.0f) a -= 360.0f;
    while (a <    0.0f) a += 360.0f;
    return a;
}

// 经纬度 → ENU（米），relative to (ref_lat, ref_lon)
static void latlon_to_enu(double lat, double lon,
                           double ref_lat, double ref_lon, double ref_cos,
                           float *e, float *n)
{
    *e = (float)((lon - ref_lon) * F_DEG2RAD * EARTH_R * ref_cos);
    *n = (float)((lat - ref_lat) * F_DEG2RAD * EARTH_R);
}

// ENU（米）→ 经纬度，相对 s_ref_*
static void enu_to_latlon(float e, float n, double *lat, double *lon)
{
    *lat = s_ref_lat + (double)(n / (float)EARTH_R) * F_RAD2DEG;
    *lon = s_ref_lon + (double)(e / (float)(EARTH_R * s_ref_cos)) * F_RAD2DEG;
}

// ====== GPS 校正（在 fusion_update 内、100Hz 调用） ======
static void apply_gps_correction(void)
{
    if (!s_last_gps.fix_valid) return;

    // 首帧：建立参考原点
    if (!s_ref_set) {
        s_ref_lat = s_last_gps.lat;
        s_ref_lon = s_last_gps.lon;
        s_ref_cos = cos(s_last_gps.lat * F_DEG2RAD);
        s_ref_set = true;
        s_e = s_n = s_vE = s_vN = 0.0f;
        s_prev_have = false;
        ESP_LOGI(TAG, "参考原点已设定: %.7f, %.7f", s_ref_lat, s_ref_lon);
        return;
    }

    // GPS → ENU
    float e_gps, n_gps;
    latlon_to_enu(s_last_gps.lat, s_last_gps.lon,
                  s_ref_lat, s_ref_lon, s_ref_cos, &e_gps, &n_gps);

    // 跳点剔除
    float jump = hypotf(e_gps - s_e, n_gps - s_n);
    if (jump > JUMP_LIMIT_M) {
        ESP_LOGW(TAG, "GPS 跳点 %.1fm，丢弃该帧", jump);
        return;
    }

    // 位置互补滤波
    s_e += K_POS * (e_gps - s_e);
    s_n += K_POS * (n_gps - s_n);

    // 速度向量校正 + heading bias 学习（需足够快才可信）
    if (s_prev_have && s_last_gps.speed_mps > COG_MIN_SPD) {
        float de = e_gps - s_prev_e;
        float dn = n_gps - s_prev_n;
        float disp = hypotf(de, dn);
        if (disp > COG_MIN_DISP) {
            // GPS 速度向量（方向 = 位移方向，幅值 = GPS 地速）
            float inv  = s_last_gps.speed_mps / disp;
            float vE_gps = de * inv;
            float vN_gps = dn * inv;
            s_vE += K_VEL * (vE_gps - s_vE);
            s_vN += K_VEL * (vN_gps - s_vN);

            // heading bias：用 GPS COG 慢修正 BNO055 常值偏差
            float cog = fwrap360(atan2f(de, dn) * F_RAD2DEG); // 0=N, 顺时针
            float yaw_bno = 0.0f;
            if (bno055_get_heading(&yaw_bno) == ESP_OK) {
                float err = fwrap180(cog - fwrap360(yaw_bno + s_psi_bias_deg));
                s_psi_bias_deg = fwrap180(s_psi_bias_deg + K_BIAS_HDG * err);
            }
        }
    }
    s_prev_e = e_gps;
    s_prev_n = n_gps;
    s_prev_have = true;
}

// ====== 公共 API ======
void fusion_init(void)
{
    memset(&s_last_gps, 0, sizeof(s_last_gps));
    s_inited             = true;
    s_ref_set            = false;
    s_have_gps           = false;
    s_prev_have          = false;
    s_pending_correction = false;
    s_e = s_n = s_vE = s_vN = 0.0f;
    s_psi_bias_deg = 0.0f;
    s_ab_x = s_ab_y = 0.0f;
    s_last_pred_us = 0;
    s_last_fix_ms  = 0;
    ESP_LOGI(TAG, "fusion 模块初始化完成");
}

void fusion_calibrate_acc_bias(uint32_t ms)
{
    int   n  = 0;
    float sx = 0.0f, sy = 0.0f;
    uint32_t t0 = (uint32_t)(esp_timer_get_time() / 1000ULL);
    ESP_LOGI(TAG, "加速度计零偏标定中（%.1f s）…", ms / 1000.0f);
    while ((uint32_t)(esp_timer_get_time() / 1000ULL) - t0 < ms) {
        float ax, ay, az;
        if (bno055_get_linear_accel(&ax, &ay, &az) == ESP_OK) {
            sx += ax; sy += ay; n++;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (n > 10) {
        s_ab_x = sx / (float)n;
        s_ab_y = sy / (float)n;
        ESP_LOGI(TAG, "零偏标定完成: ax_bias=%.3f m/s²  ay_bias=%.3f m/s²  (n=%d)",
                 s_ab_x, s_ab_y, n);
    } else {
        ESP_LOGW(TAG, "零偏标定样本不足（n=%d），保持默认 0", n);
    }
}

void fusion_feed_gps(const fusion_gps_in_t *g)
{
    if (!g) return;
    s_last_gps = *g;
    s_have_gps = true;
    if (g->fix_valid) {
        s_last_fix_ms        = g->timestamp_ms;
        s_pending_correction = true;
    }
}

void fusion_update(void)
{
    if (!s_inited) return;

    // 1. 吸收上一帧 GPS 校正（非实时，在控制任务里统一处理）
    if (s_pending_correction) {
        apply_gps_correction();
        s_pending_correction = false;
    }
    if (!s_ref_set) return; // 参考原点尚未建立，无法 DR

    // 2. 计算 dt
    int64_t now_us = esp_timer_get_time();
    float dt = (s_last_pred_us == 0) ? 0.01f
                                     : (float)(now_us - s_last_pred_us) * 1e-6f;
    s_last_pred_us = now_us;
    if (dt > 0.1f) dt = 0.1f; // 防止卡顿后大步长

    // 3. 读 BNO055：融合航向 + 线性加速度
    float yaw_bno = 0.0f, ax_b = 0.0f, ay_b = 0.0f, az_b = 0.0f;
    if (bno055_get_heading(&yaw_bno) != ESP_OK) return;
    if (bno055_get_linear_accel(&ax_b, &ay_b, &az_b) != ESP_OK) return;

    // 4. 去零偏
    ax_b -= s_ab_x;
    ay_b -= s_ab_y;

    // 5. Body → ENU 旋转
    //    BNO055 body 系（NDOF）：X = 船头，Y = 左舷
    //    ENU: E = 东，N = 北
    //    船头单位向量 ENU = (sin psi, cos psi)
    //    左舷单位向量 ENU = (-cos psi, sin psi)
    float psi = fwrap360(yaw_bno + s_psi_bias_deg) * F_DEG2RAD;
    float cs  = cosf(psi), sn = sinf(psi);
    float aE  = ax_b * sn  + ay_b * (-cs);
    float aN  = ax_b * cs  + ay_b * ( sn);

    // 6. 积分速度，再积分位置（极弱阻尼防止漂移）
    s_vE = s_vE * VEL_DECAY + aE * dt;
    s_vN = s_vN * VEL_DECAY + aN * dt;
    s_e += s_vE * dt;
    s_n += s_vN * dt;

    // 7. 零速检测（ZUPT）：静止时清速度 + 慢学零偏
    float a_norm   = hypotf(ax_b, ay_b);
    bool  gps_slow = s_have_gps && s_last_gps.fix_valid &&
                     (s_last_gps.speed_mps < ZUPT_SPD);
    if (gps_slow && a_norm < ZUPT_ACC) {
        s_vE = 0.0f;
        s_vN = 0.0f;
        // 此时 ax_b 即残余零偏（真实静止时 LIA 应为 0）
        s_ab_x += K_BIAS_ACC * ax_b;
        s_ab_y += K_BIAS_ACC * ay_b;
    }
}

// ====== 查询接口 ======
bool fusion_get_latlon(double *lat, double *lon)
{
    if (!s_ref_set) return false;
    enu_to_latlon(s_e, s_n, lat, lon);
    return true;
}

float fusion_get_heading_deg(void)
{
    float yaw = 0.0f;
    bno055_get_heading(&yaw);
    return fwrap360(yaw + s_psi_bias_deg);
}

void fusion_get_velocity_enu(float *vE, float *vN)
{
    if (vE) *vE = s_vE;
    if (vN) *vN = s_vN;
}

float fusion_get_surge_mps(void)
{
    float psi = fusion_get_heading_deg() * F_DEG2RAD;
    return s_vE * sinf(psi) + s_vN * cosf(psi);
}

uint32_t fusion_gps_age_ms(void)
{
    if (!s_last_fix_ms) return UINT32_MAX;
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
    return now - s_last_fix_ms;
}

bool fusion_is_valid(void)
{
    return s_ref_set && (fusion_gps_age_ms() < DR_MAX_AGE_MS);
}

void fusion_get_local_enu(double ref_lat, double ref_lon,
                          float *east_m, float *north_m)
{
    if (!s_ref_set) {
        if (east_m)  *east_m  = 0.0f;
        if (north_m) *north_m = 0.0f;
        return;
    }
    // 当前融合位置 → 经纬度 → 相对 ref 的 ENU
    double lat, lon;
    enu_to_latlon(s_e, s_n, &lat, &lon);
    double cos_ref = cos(ref_lat * F_DEG2RAD);
    if (east_m)  *east_m  = (float)((lon - ref_lon) * F_DEG2RAD * EARTH_R * cos_ref);
    if (north_m) *north_m = (float)((lat - ref_lat) * F_DEG2RAD * EARTH_R);
}
