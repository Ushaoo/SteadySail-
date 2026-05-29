#ifndef FUSION_H
#define FUSION_H

/*
 * fusion.h – GPS + BNO055 传感器融合定位
 *
 * 算法：互补滤波位置校正 + LIA 向量航位推算 + GPS COG heading 偏置学习
 *
 * 数据流：
 *   gps_nmea.c  → fusion_feed_gps()          (GPS 任务, ~1 Hz)
 *   main.c      → fusion_update()             (控制任务, 100 Hz)
 *   main.c      → fusion_calibrate_acc_bias() (开机静置 1.5s 一次)
 *   anchor_control.c / 其他模块 → fusion_get_*()
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// GPS 输入结构体（由 gps_nmea 填充后调用 fusion_feed_gps）
typedef struct {
    double   lat;          // 纬度，度（北正）
    double   lon;          // 经度，度（东正）
    float    speed_mps;    // 地速 m/s（由 NMEA speed_knots × 0.5144 得到）
    bool     fix_valid;    // true = 有效 A 状态 fix
    uint32_t timestamp_ms; // esp_timer_get_time()/1000
} fusion_gps_in_t;

// ====== 初始化 / 配置 ======

// 模块初始化（app_main 中调用一次，须在 bno055_init 之后）
void fusion_init(void);

// 开机静态标定加速度计零偏（船静止时调用，持续 ms 毫秒采样）
// 建议：bno055_init 成功后立刻调用 fusion_calibrate_acc_bias(1500)
void fusion_calibrate_acc_bias(uint32_t ms);

// ====== 数据喂入 ======

// GPS 每帧 RMC 解析后调用（无 fix 也调用，fix_valid=false 即可）
void fusion_feed_gps(const fusion_gps_in_t *g);

// 100 Hz 控制循环调用：读 BNO055（heading + LIA）做航位推算，并吸收上次 GPS 校正
void fusion_update(void);

// ====== 查询接口 ======

// 融合后全局经纬度；返回 false 表示尚未收到首帧 GPS（原点未设定）
bool fusion_get_latlon(double *lat, double *lon);

// 融合后航向（度，0=北，顺时针），含 GPS-COG 慢学习偏置修正
float fusion_get_heading_deg(void);

// ENU 速度向量（m/s），向量 DR 积分结果
void fusion_get_velocity_enu(float *vE, float *vN);

// 沿船头方向的纵向速度（m/s）：>0 前进，<0 倒车
float fusion_get_surge_mps(void);

// 自上次有效 GPS fix 以来的毫秒数；未收到过任何 fix 时返回 UINT32_MAX
uint32_t fusion_gps_age_ms(void);

// 当前定位是否可信（已有参考原点 && GPS age < DR_MAX_AGE_MS）
bool fusion_is_valid(void);

// 把当前融合位置换算成以 ref_lat/ref_lon 为原点的局部 ENU（米）
// east_m > 0 = 东，north_m > 0 = 北；当 fusion 无效时输出 (0, 0)
void fusion_get_local_enu(double ref_lat, double ref_lon,
                          float *east_m, float *north_m);

#ifdef __cplusplus
}
#endif

#endif // FUSION_H
