#include "steering_control.h"
#include "motor_control.h"
#include "system_config.h"
#include "control_params.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>

static const char *TAG = "STEERING";

// NVS namespace / key
#define STEER_NVS_NS       "steering"
#define STEER_NVS_KEY_OFFL "off_l"
#define STEER_NVS_KEY_OFFR "off_r"

// --- 编码器硬件捕获 (MT6826S PWM 模式) ---
typedef struct {
    uint32_t pin;
    volatile int64_t last_edge_time;
    volatile uint32_t high_us;
    volatile uint32_t period_us;
    volatile bool valid;
    volatile uint8_t bad_streak;     // 连续坏帧计数（去抖：连续 N 次才报无效）
    // ===== 原子快照 =====
    // ISR 在每个完整周期（上升沿）结束时，将 high_us/period_us 一次性打包写入 snap_*，
    // 主任务只读 snap_* 而不直接读 high_us/period_us，避免跨周期撕裂读。
    // 用 portMUX_TYPE 保护快照的写入与读取临界区。
    volatile uint32_t snap_high;
    volatile uint32_t snap_period;
    volatile bool     snap_valid;
} encoder_state_t;

static encoder_state_t enc_left  = { .pin = PIN_ENC_LEFT,  .valid = false, .snap_valid = false };
static encoder_state_t enc_right = { .pin = PIN_ENC_RIGHT, .valid = false, .snap_valid = false };

// 编码器快照的 spinlock（ISR 与主任务共用）
static portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;

static float target_left = 180.0f;
static float target_right = 180.0f;

// 编码器初始校准偏移值
static float offset_left = 0.0f;
static float offset_right = 0.0f;
// 编码器滤波缓存（简单直通）
static float filtered_angle_left = 180.0f;
static float filtered_angle_right = 180.0f;
// 首次播种标志：true 时野值过滤暂停，第一拍合法读数直接覆盖 filtered_angle_*。
// 解决开机时舵机不在 180° 也被卡死显示 180° 的问题（diff>40° 永远被野值剔除）。
// 校准 / NVS 加载后会重置为 false，触发重新播种。
static bool filtered_left_seeded  = false;
static bool filtered_right_seeded = false;

// 是否已经完成"竖直 → 180°"校准（NVS 加载成功 或 用户手动触发过）
static bool s_calibrated = false;

// --- ISR 外部中断处理函数 ---
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    encoder_state_t* st = (encoder_state_t*) arg;
    int level = gpio_get_level(st->pin);
    int64_t now = esp_timer_get_time();

    // 首次中断： last_edge_time==0 时 delta = now-0 是巨大值，会把
    // high_us / period_us 污染成几十万 us，bad_streak 连增使 valid 反复在 false。
    // 仅记录参考边沿时间，下一次中断才开始计算 delta。
    if (st->last_edge_time == 0) {
        st->last_edge_time = now;
        return;
    }
    int64_t delta = now - st->last_edge_time;

    if (delta < 5) return; // 5us 短期毛刺滤波

    if (level == 0) {
        // 下降沿：此时的 delta 是高电平时间
        st->high_us = (uint32_t)delta;
    } else {
        // 上升沿：此时的 delta 是低电平时间（周期完成）
        st->period_us = (uint32_t)delta;

        uint32_t total = st->high_us + st->period_us;
        if (total >= 100 && total <= 50000 && st->high_us > 0 && st->period_us > 0) {
            st->valid = true;
            st->bad_streak = 0;
            // 整个周期数据就绪，原子写入快照（主任务只读 snap_*）
            portENTER_CRITICAL_ISR(&enc_mux);
            st->snap_high   = st->high_us;
            st->snap_period = st->period_us;
            st->snap_valid  = true;
            portEXIT_CRITICAL_ISR(&enc_mux);
        } else {
            // 去抖：单次坏帧不立即报无效，连续 8 次才翻 valid=false
            // 一旦下一个完整周期正常，会自动恢复 valid=true（自纠正）
            if (st->bad_streak < 255) st->bad_streak++;
            if (st->bad_streak >= 8) {
                st->valid = false;
                portENTER_CRITICAL_ISR(&enc_mux);
                st->snap_valid = false;
                portEXIT_CRITICAL_ISR(&enc_mux);
            }
        }
    }
    st->last_edge_time = now;
}

// 占空比转角度 (0~360)
// MT6826S 编码器：占空比 ~1%~99% 对应 0~360°（数据手册端点附近留有最小高/低脉宽保护）
static float compute_angle(volatile uint32_t high_us, volatile uint32_t period_us, volatile bool valid) {
    if (!valid || period_us == 0) return 0.0f;
    
    uint32_t total_period = high_us + period_us;
    float duty_cycle = (float)high_us / (float)total_period;
    
    // 端点放宽到 MT6826S 真实范围：1% -> 0°, 99% -> 360°
    const float DC_MIN = 0.01f;
    const float DC_MAX = 0.99f;
    const float RANGE = DC_MAX - DC_MIN;  // 98%
    
    float angle = (duty_cycle - DC_MIN) / RANGE * 360.0f;
    
    // 端点饱和（clamp，不再 wrap）：避免边缘抖动跨 0/360 翻转
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 360.0f) angle = 360.0f;
    
    return angle;
}

// 最短路径环形误差计算
static float shortest_angle_error(float target, float current) {
    float err = target - current;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

// ---------- NVS 校准存取 ----------
static esp_err_t nvs_load_offsets(float *off_l, float *off_r) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(STEER_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    union { uint32_t u; float f; } cvt_l, cvt_r;
    err = nvs_get_u32(h, STEER_NVS_KEY_OFFL, &cvt_l.u);
    if (err == ESP_OK) {
        err = nvs_get_u32(h, STEER_NVS_KEY_OFFR, &cvt_r.u);
    }
    nvs_close(h);
    if (err != ESP_OK) return err;

    *off_l = cvt_l.f;
    *off_r = cvt_r.f;
    return ESP_OK;
}

static esp_err_t nvs_save_offsets(float off_l, float off_r) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(STEER_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    union { uint32_t u; float f; } cvt_l = { .f = off_l }, cvt_r = { .f = off_r };
    err = nvs_set_u32(h, STEER_NVS_KEY_OFFL, cvt_l.u);
    if (err == ESP_OK) err = nvs_set_u32(h, STEER_NVS_KEY_OFFR, cvt_r.u);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void steering_control_init(void) {
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_ENC_LEFT) | (1ULL << PIN_ENC_RIGHT),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(PIN_ENC_LEFT, encoder_isr_handler, (void*)&enc_left);
    gpio_isr_handler_add(PIN_ENC_RIGHT, encoder_isr_handler, (void*)&enc_right);

    ESP_LOGI(TAG, "✓ Steering Encoder Interrupts Initialized.");
    ESP_LOGI(TAG, "  - Left Encoder GPIO: %d | Right Encoder GPIO: %d", PIN_ENC_LEFT, PIN_ENC_RIGHT);

    // ===== 等待编码器输出稳定 (防 boot 阶段误以为舵在 180°) =====
    // MT6826S 上电后需要几个 PWM 周期 (~5–20ms) 才输出稳定占空比。
    // 如果不等，enterprise 循环会看到 valid=false，raw 返回 0 、归一化后为 180°，
    // 并以为当前位置就是 180°。这里主动轮询最多 500ms，超时只告警不阻塞启动。
    {
        ESP_LOGI(TAG, "等待编码器输出稳定 (最多 500ms)...");
        TickType_t t0 = xTaskGetTickCount();
        const TickType_t TIMEOUT = pdMS_TO_TICKS(500);
        while (!enc_left.valid || !enc_right.valid) {
            if (xTaskGetTickCount() - t0 > TIMEOUT) {
                ESP_LOGW(TAG, "⚠ 编码器 500ms 内未稳定: L_valid=%d R_valid=%d。启动继续，但初始读数可能不准",
                         (int)enc_left.valid, (int)enc_right.valid);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (enc_left.valid && enc_right.valid) {
            ESP_LOGI(TAG, "✓ 编码器已稳定 (耗时 %d ms)",
                     (int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
        }
    }

    // 尝试从 NVS 加载上次保存的零点偏移
    float loaded_l = 0.0f, loaded_r = 0.0f;
    if (nvs_load_offsets(&loaded_l, &loaded_r) == ESP_OK) {
        offset_left  = loaded_l;
        offset_right = loaded_r;
        filtered_angle_left  = 180.0f;  // 临时占位，首拍 get_current_angles() 会用真实读数覆盖
        filtered_angle_right = 180.0f;
        filtered_left_seeded  = false;  // 强制首拍播种
        filtered_right_seeded = false;
        s_calibrated = true;
        ESP_LOGI(TAG, "✓ 已从 NVS 加载校准: offset_L=%.2f° offset_R=%.2f°", offset_left, offset_right);
    } else {
        s_calibrated = false;
        ESP_LOGW(TAG, "⚠ 未发现校准数据，舵机已锁定 PWM=1500。");
        ESP_LOGW(TAG, "  请把舵机摆到正下方（180° 竖直）并发送 'cal' 命令完成首次校准。");
    }
}

void steering_control_set_target(float target_left_deg, float target_right_deg) {
    // 注意：右侧做了 360- 翻转，这是历史遗留——与 main.c 中
    //   send_tgt_R = 360 - filter_target_L
    // 的镜像"双重抵消"后，PID 才能让右舵机走到正确位置。
    // 任何调用者都要意识到：写入的 R 值与读出的 R 值 (get_target) 不是同一帧。
    // 门控判据必须使用 get_target() 的返回值，而不是写入的原始 send_tgt_R。
    target_left  = target_left_deg;
    target_right = 360.0f - target_right_deg;
}
void steering_control_get_target(float *left_deg, float *right_deg) {
    if (left_deg)  *left_deg  = target_left;
    if (right_deg) *right_deg = target_right;
}
void steering_control_calibrate_and_save(void) {
    // 读取当前编码器的竖直状态值作为校准基准
    // 原子读取快照（与 get_current_angles 保持一致）
    uint32_t snap_high_L, snap_period_L; bool snap_valid_L;
    uint32_t snap_high_R, snap_period_R; bool snap_valid_R;
    portENTER_CRITICAL(&enc_mux);
    snap_high_L   = enc_left.snap_high;
    snap_period_L = enc_left.snap_period;
    snap_valid_L  = enc_left.snap_valid;
    snap_high_R   = enc_right.snap_high;
    snap_period_R = enc_right.snap_period;
    snap_valid_R  = enc_right.snap_valid;
    portEXIT_CRITICAL(&enc_mux);

    float cal_left  = compute_angle(snap_high_L, snap_period_L, snap_valid_L);
    float cal_right = compute_angle(snap_high_R, snap_period_R, snap_valid_R);

    // 设置偏移使得初始竖直状态对应 180°（避免 0/360 边界抖动）
    offset_left  = cal_left  - 180.0f;
    offset_right = cal_right - 180.0f;

    // 初始化滤波缓存（校准刚完成，舵机一定在 180° 物理位）
    filtered_angle_left  = 180.0f;
    filtered_angle_right = 180.0f;
    filtered_left_seeded  = true;
    filtered_right_seeded = true;

    s_calibrated = true;
    ESP_LOGI(TAG, "Encoder Calibration Complete. Offset Left: %.2f°, Offset Right: %.2f°", offset_left, offset_right);

    esp_err_t err = nvs_save_offsets(offset_left, offset_right);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✓ 校准已保存到 NVS（重启后自动加载，无需再次校准）");
    } else {
        ESP_LOGE(TAG, "✗ 保存校准到 NVS 失败: %s", esp_err_to_name(err));
    }
}

bool steering_control_is_calibrated(void) {
    return s_calibrated;
}

void steering_control_get_current_angles(float *left_deg, float *right_deg) {
    // ===== valid 从 false 翻为 true 时，强制重新播种 =====
    // 这个场景例如： boot 阶段编码器一直 invalid，filtered 在 180°；突然 valid=true 后
            // 真实 raw 可能距 180 远远大于 40°，会被野值剖除永久卵住。
    // 主动检测上一帧 valid 状态，发现恢复就清 seeded 让下面 seed 分支重新播种。
    static bool prev_left_valid  = false;
    static bool prev_right_valid = false;
    if (enc_left.valid && !prev_left_valid) {
        filtered_left_seeded = false;
    }
    if (enc_right.valid && !prev_right_valid) {
        filtered_right_seeded = false;
    }
    prev_left_valid  = enc_left.valid;
    prev_right_valid = enc_right.valid;

    // 通过临界区原子读取 ISR 快照，避免读到 high_us/period_us 属于不同周期的数据
    uint32_t snap_high_L, snap_period_L; bool snap_valid_L;
    uint32_t snap_high_R, snap_period_R; bool snap_valid_R;
    portENTER_CRITICAL(&enc_mux);
    snap_high_L   = enc_left.snap_high;
    snap_period_L = enc_left.snap_period;
    snap_valid_L  = enc_left.snap_valid;
    snap_high_R   = enc_right.snap_high;
    snap_period_R = enc_right.snap_period;
    snap_valid_R  = enc_right.snap_valid;
    portEXIT_CRITICAL(&enc_mux);

    float raw_left  = compute_angle(snap_high_L, snap_period_L, snap_valid_L);
    float raw_right = compute_angle(snap_high_R, snap_period_R, snap_valid_R);
    
    // 相对于初始校准点的角度
    raw_left = raw_left - offset_left;
    raw_right = raw_right - offset_right;

    // 应用编码器方向反转开关（由 system_config.h 集中配置）
    // 以 180° 为镜像中心，源于校准后初始位是 180°
#if ENC_LEFT_REVERSE
    raw_left = 360.0f - raw_left;
#endif
#if ENC_RIGHT_REVERSE
    raw_right = 360.0f - raw_right;
#endif

    // 将原始角度归一化到 [0, 360) 范围
    while (raw_left < 0.0f) raw_left += 360.0f;
    while (raw_left >= 360.0f) raw_left -= 360.0f;
    while (raw_right < 0.0f) raw_right += 360.0f;
    while (raw_right >= 360.0f) raw_right -= 360.0f;

    // ===== 方案 E：野值剔除 + 一阶 LPF =====
    // 100Hz 调用，舵机最快 ~360°/s -> 单拍 ≤ 3.6°；任何 >40° 的瞬时跳变视为 EMI 假读，
    // 保留上一拍 filtered 值，绝不让 PID 看到 0° 或瞬间翻转。
    const float OUTLIER_DEG = 40.0f;
    const float LPF_ALPHA   = 0.35f;   // 截止 ~5Hz @ 100Hz

    // 左
    {
        if (enc_left.valid && !filtered_left_seeded) {
            // 首次播种：跳过野值剔除，直接吃当前 raw 读数为初值
            filtered_angle_left = raw_left;
            filtered_left_seeded = true;
        } else {
            float prev = filtered_angle_left;
            float d = raw_left - prev;
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            if (!enc_left.valid || fabsf(d) > OUTLIER_DEG) {
                raw_left = prev;   // 保留上一拍
            } else {
                float upd = prev + LPF_ALPHA * d;
                while (upd < 0.0f)    upd += 360.0f;
                while (upd >= 360.0f) upd -= 360.0f;
                filtered_angle_left = upd;
                raw_left = upd;
            }
        }
    }
    // 右
    {
        if (enc_right.valid && !filtered_right_seeded) {
            filtered_angle_right = raw_right;
            filtered_right_seeded = true;
        } else {
            float prev = filtered_angle_right;
            float d = raw_right - prev;
            while (d >  180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            if (!enc_right.valid || fabsf(d) > OUTLIER_DEG) {
                raw_right = prev;
            } else {
                float upd = prev + LPF_ALPHA * d;
                while (upd < 0.0f)    upd += 360.0f;
                while (upd >= 360.0f) upd -= 360.0f;
                filtered_angle_right = upd;
                raw_right = upd;
            }
        }
    }

    *left_deg = raw_left;
    *right_deg = raw_right;
}

// 获取编码器健康状态
void steering_control_get_encoder_status(bool *left_ok, bool *right_ok) {
    *left_ok = enc_left.valid;
    *right_ok = enc_right.valid;
}

// 提取单边 PID 计算
static float calculate_pid(float error, float *integral, float *prev_error, float *out_filt) {
    // PID 增益从全局变量读取（可通过 Blinker 实时调参）
    const float kp = g_steer_kp, ki = g_steer_ki, kd = g_steer_kd, dt = 0.01f;
    const float integral_max = 80.0f;

    // 注：外层 steering_control_update() 已经用 DEADZONE=3° 做了死区门控，
    // 这里不再叠加 smoothstep，避免 4°~6° 小误差被双重削弱后落到 0。
    float err_smooth = error;

    // P 项
    float p_out = kp * err_smooth;

    // I 项
    float i_candidate = *integral + err_smooth * dt * ki;
    if (i_candidate > integral_max) {
        i_candidate = integral_max;
    } else if (i_candidate < -integral_max) {
        i_candidate = -integral_max;
    }
    *integral = i_candidate;
    float i_out = *integral;

    // D 项
    float derivative = 0.0f;
    if (dt > 0) {
        derivative = (error - *prev_error) / dt;
        if (derivative > 200.0f) derivative = 200.0f;
        if (derivative < -200.0f) derivative = -200.0f;
    }
    *prev_error = error;
    float d_out = kd * derivative;

    float raw_out = p_out + i_out + d_out;

    // 低通滤波（仅平滑 D 项高频噪声，不应大幅限制 P 项响应速度）
    // 原 0.12f → τ≈125ms，截止~1.3Hz，过度压慢了整体响应。
    // 改为 0.03f → τ≈30ms，截止~5.3Hz，与大电机推力滤波器一致。
    // 如果振荡，可尝试 0.05f（τ≈50ms）作为中间值。
    float alpha = dt / (0.03f + dt);
    *out_filt = *out_filt + alpha * (raw_out - *out_filt);

    float final_out = *out_filt;
    if (final_out > 500.0f) final_out = 500.0f;
    if (final_out < -500.0f) final_out = -500.0f;
    
    // 输出死区：低于 30 的微弱信号清零，防止电机连续微弱运转损耗。
    // ⚠ 注意：不再做 30→60 的强制跳变。原因：当 PID 输出在 [30,60) 时若强推到 60，
    // 会在死区边缘（误差 2°~4°）产生速度不连续，引起 60→-60 反复振荡（bang-bang 效应）。
    // 去掉后 PID 输出连续变化，由 DEADZONE=3° 的外层门控负责抑制微小误差。
    if (fabsf(final_out) < 30.0f) {
        final_out = 0.0f;
    }
    

    return final_out;
}

// PID 状态
static float integral_left = 0.0f, integral_right = 0.0f;
static float prev_err_left = 0.0f, prev_err_right = 0.0f;
static float out_filt_left = 0.0f, out_filt_right = 0.0f;

void steering_control_update(void) {
    // 未校准 → 强制中立位 PWM=1500（最安全：360° 连续舵机此时不旋转）
    if (!s_calibrated) {
        motor_control_set_steering_pwm(1500, 1500);
        return;
    }

    float cur_left, cur_right;
    steering_control_get_current_angles(&cur_left, &cur_right);

    // 【编码器失效保护】
    // get_current_angles 在 invalid 时只是"冻结上一拍滤波值"，PID 自己感知不到失效。
    // 若不在这里拦截，PID 会按冻结的角度持续喷 PWM，360° 连续舵机会
    // 一路狂转过冲一整圈（实测 270→90 多走 360° = 540°）。
    // 失效时直接 PWM=1500（舵机不转），同时复位该侧 PID 状态，避免恢复后积分爆发。
    bool enc_l_ok = enc_left.valid;
    bool enc_r_ok = enc_right.valid;

    float err_L = shortest_angle_error(target_left, cur_left);
    float err_R = shortest_angle_error(target_right, cur_right);

    const float DEADZONE = 3.0f;
    float adjust_L = 0.0f, adjust_R = 0.0f;

#if STEERING_CONTROL_MODE == STEERING_MODE_PID
    // PID 模式
    if (!enc_l_ok) {
        // 编码器失效：停转 + 清状态，等编码器恢复
        adjust_L = 0.0f;
        integral_left = 0.0f;
        prev_err_left = 0.0f;
        out_filt_left = 0.0f;
    } else if (fabsf(err_L) > DEADZONE) {
        adjust_L = calculate_pid(err_L, &integral_left, &prev_err_left, &out_filt_left);
    } else {
        adjust_L = 0.0f;
        integral_left = 0.0f;
    }

    if (!enc_r_ok) {
        adjust_R = 0.0f;
        integral_right = 0.0f;
        prev_err_right = 0.0f;
        out_filt_right = 0.0f;
    } else if (fabsf(err_R) > DEADZONE) {
        adjust_R = calculate_pid(err_R, &integral_right, &prev_err_right, &out_filt_right);
    } else {
        adjust_R = 0.0f;
        integral_right = 0.0f;
    }
#else
    // 直接映射模式（中等响应速度）
    if (enc_l_ok && fabsf(err_L) > DEADZONE) {
        // 降低比例增益与最大转速限幅
        // PWM 范围限制在 1500 ± 150 (即 1200 到 1800)
        adjust_L = err_L * 5.0f; 
        if (adjust_L > 100.0f) adjust_L = 100.0f;
        if (adjust_L < -100.0f) adjust_L = -100.0f;
    }

    if (enc_r_ok && fabsf(err_R) > DEADZONE) {
        adjust_R = err_R * 5.0f;
        if (adjust_R > 100.0f) adjust_R = 100.0f;
        if (adjust_R < -100.0f) adjust_R = -100.0f;
    }
#endif

    uint32_t pwm_L = (uint32_t)(1500.0f - adjust_L);
    uint32_t pwm_R = (uint32_t)(1500.0f - adjust_R);

    motor_control_set_steering_pwm(pwm_L, pwm_R);
}