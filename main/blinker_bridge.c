#include "blinker_bridge.h"
#include "control_params.h"
#include "steering_control.h"
#include "motor_control.h"
#include "system_config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "cJSON.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include "blinker_api.h"

static const char *TAG = "BLINKER_BR";

// 来自 main.c 的全局共享变量
extern volatile float g_forward_thrust;
extern volatile float g_last_roll_deg;
extern volatile bool  g_estop_active;
extern volatile bool  g_hard_estop;       // 硬急停：所有通道直接 1500us
extern volatile float g_demo_roll_deg;   // DEMO_MANUAL_ROLL 手动横滚角输入
extern volatile int   g_turn_state;       // 差速转向状态：-1=左/0=直/+1=右

// ============================================================
// 控件键名（与 Blinker App 面板上的控件键名一致）
// ============================================================
#define KEY_PID_IN    "pid_in"
#define KEY_THRUST    "thrust"
#define KEY_ESTOP     "estop"

// 三个差速转向按钮（Blinker App 端按钮控件键名，需对应建立）
#define KEY_TURN_L    "turn_l"   // 左转
#define KEY_TURN_F    "turn_f"   // 直行（取消转向）
#define KEY_TURN_R    "turn_r"   // 右转

// 三个推力快速测试按钮（一键设 fwd 为 +30 / 0 / -30）
#define KEY_FWD_P30   "fwd_p30"
#define KEY_FWD_0     "fwd_0"
#define KEY_FWD_N30   "fwd_n30"

// 硬急停 与 重启按钮
#define KEY_HARD_STOP "hard_stop"   // tap 立即把所有通道压到 1500us
#define KEY_REBOOT    "reboot"      // tap 立即重启 ESP

// 在线校准按钮（防误触：必须长按后抬起才生效）
#define KEY_CAL       "cal"         // 长按 -> 调 steering_control_calibrate_and_save()

#define KEY_PWM_L     "pwm_l"
#define KEY_PWM_R     "pwm_r"
#define KEY_STEER_L   "steer_l"
#define KEY_STEER_R   "steer_r"
#define KEY_ROLL      "roll"
#define KEY_STEER_PID "steer_pid"
#define KEY_BAL_PID   "bal_pid"
#define KEY_ESTOP_ST  "estop_state"

// 6 个 PID 参数独立数字显示控件
#define KEY_SP        "sp"
#define KEY_SI        "si"
#define KEY_SD        "sd"
#define KEY_BP        "bp"
#define KEY_BI        "bi"
#define KEY_BD        "bd"

// ============================================================
// 推力滑块：直接写入全局推力（-100~100）
// 注：Blinker SLIDER 默认 0~100，App 端需将范围设置为 -100~100
// ============================================================
static void on_thrust_slider(const blinker_widget_param_val_t *val)
{
    float v = (float)val->i;
    if (v > 100.0f) v = 100.0f;
    if (v < -100.0f) v = -100.0f;
    g_forward_thrust = v;
    ESP_LOGI(TAG, "[App] 推力滑块: %.1f%%", v);
}

// ============================================================
// PID 输入解析逻辑——支持两种语法，可混用：
//
//   1) 单参数语法（推荐，一次只改一个值）
//        sp <v>   si <v>   sd <v>   -> 转向 PID Kp/Ki/Kd
//        bp <v>   bi <v>   bd <v>   -> 平衡 PID Kp/Ki/Kd
//        例：“sp 4.9”、“si 1”、“bp 20  bi 1  bd 0”（可拼接）
//
//   2) 三参数整组语法（与原老格式兼容）
//        s <kp> <ki> <kd>   /   b <kp> <ki> <kd>
//        例：“s 5 1 0.49”、“b 20 1 0”
//
//   3) DEMO 手动横滚角输入（与控制台 r 命令同语义）
//        r <angle>   -> 写入 g_demo_roll_deg，范围 ±60°
//        例：“r 10”、“r -5”、“r 0”
//
//   4) 前进推力输入（与控制台数字输入等价，范围 ±100）
//        f <percent> -> 写入 g_forward_thrust
//        例：“f 30”、“f -20”、“f 0”
//
// 返回：是否至少成功更新了 1 个参数。
// ============================================================
static bool try_parse_and_apply_pid(const char *raw)
{
    if (raw == NULL) return false;

    // 优先匹配整字串命令：reboot（不区分大小写）
    {
        const char *needle = "reboot";
        size_t nlen = strlen(needle);
        for (const char *s = raw; *s; s++) {
            size_t i = 0;
            while (i < nlen && s[i] && tolower((unsigned char)s[i]) == needle[i]) i++;
            if (i == nlen) {
                ESP_LOGW(TAG, "[App] 收到 reboot 指令，1 秒后重启 ESP32...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        }
    }

    bool any_updated = false;
    const char *p = raw;

    while (*p) {
        // 跳过非字母字符（包括 JSON 引号、逗号、空格、等号等）
        if (!isalpha((unsigned char)*p)) { p++; continue; }

        // 先试单参数语法：2 个字母 + 数值
        char k1 = (char)tolower((unsigned char)p[0]);
        char k2 = (char)tolower((unsigned char)p[1]);

        // 单字母语法：r <angle> -> 手动横滚角
        if (k1 == 'r' && !isalpha((unsigned char)p[1])) {
            float v = 0;
            const char *q = p + 1;
            while (*q == ' ' || *q == '=' || *q == ':' || *q == '\t') q++;
            if (sscanf(q, "%f", &v) == 1) {
                if (v >  60.0f) v =  60.0f;
                if (v < -60.0f) v = -60.0f;
                g_demo_roll_deg = v;
                ESP_LOGI(TAG, "[App] 手动横滚角更新: r = %.2f deg", v);
                any_updated = true;
                while (*q && (isdigit((unsigned char)*q) || *q == '.' || *q == '-' || *q == '+' || *q == 'e' || *q == 'E')) q++;
                p = q;
                continue;
            }
        }

        // 单字母语法：f <percent> -> 前进推力 (-100 ~ 100)
        if (k1 == 'f' && !isalpha((unsigned char)p[1])) {
            float v = 0;
            const char *q = p + 1;
            while (*q == ' ' || *q == '=' || *q == ':' || *q == '\t') q++;
            if (sscanf(q, "%f", &v) == 1) {
                if (v >  100.0f) v =  100.0f;
                if (v < -100.0f) v = -100.0f;
                g_forward_thrust = v;
                ESP_LOGI(TAG, "[App] 前进推力更新: f = %.1f %%", v);
                any_updated = true;
                while (*q && (isdigit((unsigned char)*q) || *q == '.' || *q == '-' || *q == '+' || *q == 'e' || *q == 'E')) q++;
                p = q;
                continue;
            }
        }
        if ((k1 == 's' || k1 == 'b') && (k2 == 'p' || k2 == 'i' || k2 == 'd')
            && !isalpha((unsigned char)p[2])) {
            float v = 0;
            // 跳过 2 个字母，读一个浮点数（允许中间是空格/=/:）
            const char *q = p + 2;
            while (*q == ' ' || *q == '=' || *q == ':' || *q == '\t') q++;
            if (sscanf(q, "%f", &v) == 1) {
                if (k1 == 's') {
                    if      (k2 == 'p') { g_steer_kp = v; }
                    else if (k2 == 'i') { g_steer_ki = v; }
                    else                { g_steer_kd = v; }
                } else { // 'b'
                    if      (k2 == 'p') { g_balance_kp = v; }
                    else if (k2 == 'i') { g_balance_ki = v; }
                    else                { g_balance_kd = v; }
                }
                ESP_LOGI(TAG, "[App] PID 单参更新: %c%c = %.3f", k1, k2, v);
                any_updated = true;
                // 跳过该数值继续扫描
                while (*q && (isdigit((unsigned char)*q) || *q == '.' || *q == '-' || *q == '+' || *q == 'e' || *q == 'E')) q++;
                p = q;
                continue;
            }
        }

        // 再试三参数整组语法：<s|b> %f %f %f（字母后不跟字母）
        if ((k1 == 's' || k1 == 'b') && !isalpha((unsigned char)p[1])) {
            char tag = 0;
            float kp = 0, ki = 0, kd = 0;
            if (sscanf(p, " %c %f %f %f", &tag, &kp, &ki, &kd) == 4) {
                if (tag == 's' || tag == 'S') {
                    g_steer_kp = kp; g_steer_ki = ki; g_steer_kd = kd;
                    ESP_LOGI(TAG, "[App] 转向 PID 整组更新: Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
                } else {
                    g_balance_kp = kp; g_balance_ki = ki; g_balance_kd = kd;
                    ESP_LOGI(TAG, "[App] 平衡 PID 整组更新: Kp=%.3f Ki=%.3f Kd=%.3f", kp, ki, kd);
                }
                any_updated = true;
                // 粗略跳过后面 3 个数值
                p++;
                int nums = 0;
                while (*p && nums < 3) {
                    if (isdigit((unsigned char)*p) || *p == '-' || *p == '+' || *p == '.') {
                        while (*p && (isdigit((unsigned char)*p) || *p == '.' || *p == '-' || *p == '+' || *p == 'e' || *p == 'E')) p++;
                        nums++;
                    } else {
                        p++;
                    }
                }
                continue;
            }
        }

        // 什么都不匹配，跳过当前连续字母串
        while (*p && isalpha((unsigned char)*p)) p++;
    }
    return any_updated;
}

// ============================================================
// 方式 1："pid_in" 按键控件回调（App 端重复按键发预设字符串时走这里）
// ============================================================
static void on_pid_input(const blinker_widget_param_val_t *val)
{
    if (val == NULL || val->s == NULL) {
        ESP_LOGW(TAG, "[App] pid_in 回调 val/s 为空");
        return;
    }
    ESP_LOGI(TAG, "[App] pid_in 收到: '%s'", val->s);
    if (!try_parse_and_apply_pid(val->s)) {
        ESP_LOGW(TAG, "[App] PID 输入未能解析: '%s' (需要 s/b kp ki kd)", val->s);
    }
}

// ============================================================
// 方式 2：原始数据回调——所有未被 widget 匹配的 JSON 都会走这里
// （比如 App 调试控件/任何手动发送的文本）
// ============================================================
static void on_raw_data(const char *data)
{
    if (data == NULL) return;
    ESP_LOGI(TAG, "[App] 原始数据: %s", data);
    try_parse_and_apply_pid(data);
}

// ============================================================
// 急停按键回调
//   交互逻辑（防误触：短按急停，长按解除）：
//     "tap"            -> 短按        -> 触发急停 (STOPPED)
//     "press"          -> 长按按下    -> 忽略，等抬起再判定
//     "pressup"        -> 长按抬起    -> 解除急停 (RUNNING)
//     "stop"/"off"     -> 自定义文本  -> 急停
//     "start"/"on"     -> 自定义文本  -> 解除急停
//     其他未知字符串                 -> toggle 兜底
// ============================================================
static void on_estop(const blinker_widget_param_val_t *val)
{
    if (val == NULL || val->s == NULL) {
        ESP_LOGW(TAG, "[App] 急停按键 val 为空");
        return;
    }
    const char *s = val->s;
    ESP_LOGW(TAG, "[App] 急停按键收到: '%s'", s);

    if (strcasecmp(s, "press") == 0) {
        // 长按按下瞬间 -> 忽略，避免误触；等待 pressup 再解除
        return;
    } else if (strcasecmp(s, "tap")  == 0 ||
               strcasecmp(s, "stop") == 0 ||
               strcasecmp(s, "off")  == 0) {
        // 短按 / 明确停止语义 -> 急停
        g_estop_active = true;
    } else if (strcasecmp(s, "pressup") == 0 ||
               strcasecmp(s, "start")   == 0 ||
               strcasecmp(s, "on")      == 0) {
        // 长按抬起 / 明确启动语义 -> 解除急停
        g_estop_active = false;
    } else {
        // 未知字符串 -> 翻转兜底
        g_estop_active = !g_estop_active;
    }
    ESP_LOGW(TAG, "[App] 急停状态 -> %s",
             g_estop_active ? "STOPPED" : "RUNNING");
}

// ============================================================
// 差速转向三按钮 —— 任意 tap 直接覆盖 g_turn_state
//   左按钮  -> g_turn_state = -1
//   前按钮  -> g_turn_state =  0  (取消转向)
//   右按钮  -> g_turn_state = +1
// 仅当 |g_forward_thrust| > TURN_MIN_FWD_PCT 时才会真正生效（main.c 内判断）。
// ============================================================
static void on_turn_left(const blinker_widget_param_val_t *val)
{
    (void)val;
    g_turn_state = -1;
    ESP_LOGI(TAG, "[App] 转向 -> 左");
}
static void on_turn_forward(const blinker_widget_param_val_t *val)
{
    (void)val;
    g_turn_state = 0;
    ESP_LOGI(TAG, "[App] 转向 -> 直行（取消）");
}
static void on_turn_right(const blinker_widget_param_val_t *val)
{
    (void)val;
    g_turn_state = +1;
    ESP_LOGI(TAG, "[App] 转向 -> 右");
}

// ============================================================
// 推力快速测试三按钮 —— 任意 tap 直接覆盖 g_forward_thrust
//   +30 按钮 -> g_forward_thrust = +30.0
//    0 按钮 -> g_forward_thrust =   0.0
//   -30 按钮 -> g_forward_thrust = -30.0
// 与滑块共用同一个全局，互不冲突，最后一次操作生效。
// ============================================================
static void on_fwd_p30(const blinker_widget_param_val_t *val)
{
    (void)val;
    g_forward_thrust = 30.0f;
    ESP_LOGI(TAG, "[App] 推力快捷 -> +30%%");
}
static void on_fwd_0(const blinker_widget_param_val_t *val)
{
    (void)val;
    g_forward_thrust = 0.0f;
    ESP_LOGI(TAG, "[App] 推力快捷 -> 0%%");
}
static void on_fwd_n30(const blinker_widget_param_val_t *val)
{
    (void)val;
    g_forward_thrust = -30.0f;
    ESP_LOGI(TAG, "[App] 推力快捷 -> -30%%");
}

// ============================================================
// 硬急停 —— tap 立即把 4 个通道全部压到 1500us
//   与原 estop 的区别：原 estop 会拿舵机走到 180°（连续舵会转动），
//   硬急停是舵机 PWM = 1500us 即"不转"。需要手动下电才能恢复。
// ============================================================
static void on_hard_stop(const blinker_widget_param_val_t *val)
{
    (void)val;
    g_hard_estop = true;
    g_estop_active = true;          // 下一轮主循环看到 hard_estop 会直接 continue
    g_forward_thrust = 0.0f;        // 清推力以防下次启动跳变
    g_turn_state = 0;
    ESP_LOGW(TAG, "[App] 硬急停！所有通道已锁定为 1500us");
}

// ============================================================
// 重启 —— tap 立即 esp_restart()
//   警告：无二次确认。如果误触会丢失当前运行状态。
// ============================================================
static void on_reboot(const blinker_widget_param_val_t *val)
{
    (void)val;
    ESP_LOGW(TAG, "[App] 收到重启命令 -> esp_restart() in 100ms");
    // 先把电机压到 1500，避免重启期间 PWM 发生不可预期跳变
    motor_control_emergency_stop();
    motor_control_set_steering_pwm(1500, 1500);
    vTaskDelay(pdMS_TO_TICKS(100));  // 让 LOG 输出与 PWM 生效
    esp_restart();
}

// ============================================================
// 在线校准按钮 —— 只接受长按 (pressup)，避免误触
//   tap     -> 仅提示“请长按”，不执行
//   press   -> 忽略（按下瞬间）
//   pressup -> 执行校准：拍下当前舵机位置作为新的 180°基准，并写入 NVS
//
//   ⚠使用前提：按之前必须亲眼确认两个推进器桃子均垂直向下，
//      否则会把错误位置当成新基准，下次开机依然会错。
//   ⚠ 校准期间会临时压下硬急停，避免调用 NVS 写时 PID 还在猛推。
// ============================================================
static void on_cal(const blinker_widget_param_val_t *val)
{
    if (val == NULL || val->s == NULL) {
        ESP_LOGW(TAG, "[App] 校准按钮 val 为空");
        return;
    }
    const char *s = val->s;

    if (strcasecmp(s, "tap") == 0) {
        ESP_LOGW(TAG, "[App] 校准按钮误触？请长按 1 秒才生效");
        return;
    }
    if (strcasecmp(s, "press") == 0) {
        // 按下瞬间不动作，等 pressup
        return;
    }
    if (strcasecmp(s, "pressup") != 0) {
        ESP_LOGW(TAG, "[App] 校准按钮未知事件: '%s' （仅 pressup 生效）", s);
        return;
    }

    ESP_LOGW(TAG, "[App] 收到长按校准命令 -> 暂停推进并拍取当前位置为新基准");
    // 临时压下硬急停，锁住 4 路 PWM，避免 PID 与校准赛跑
    bool prev_hard = g_hard_estop;
    g_hard_estop = true;
    vTaskDelay(pdMS_TO_TICKS(50));   // 让主循环跳进硬急停分支下发 1500

    steering_control_calibrate_and_save();

    // 恢复屏蔽状态（一般 prev_hard 应为 false；如果之前本来就在硬急停，保留之）
    g_hard_estop = prev_hard;
    ESP_LOGW(TAG, "[App] 校准完成，系统息复运行");
}

// ============================================================
// 数据上报任务
//
// ⚠️ Blinker 免费 broker 限制：设备发布频率最高 1 条 / 秒，超出会被踢。
//   - SDK 内部有 200ms 静默窗口合并机制：连续 send 会被合并成 1 条 publish
//   - 但当 App 下发命令时，SDK 会自动回一条 ack/state，再加上我们这条上报
//     就会出现 ≥ 2 条 / 秒 → 触发限速被踢（"errno=128 / MQTT 断开"）
//   - 因此把上报周期放宽到 2.5 秒，给入站事件留出 publish 间隙。
//
// 同时为了 100% 保证一次上报只发一条 publish，所有字段在同一个 200ms
// 静默窗口内连续 send（实际只用几毫秒），由 SDK 合并成单条 MQTT 消息。
// ============================================================
#define REPORT_PERIOD_MS  2000

static void send_number(const char *key, double value)
{
    cJSON *p = cJSON_CreateObject();
    if (!p) return;
    blinker_widget_value_number(p, value);
    blinker_widget_print(key, p);
    cJSON_Delete(p);
}

static void send_string(const char *key, const char *value)
{
    cJSON *p = cJSON_CreateObject();
    if (!p) return;
    blinker_widget_value_string(p, value);
    blinker_widget_print(key, p);
    cJSON_Delete(p);
}

static void blinker_report_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(REPORT_PERIOD_MS);
    TickType_t xLast = xTaskGetTickCount();

    while (1) {
        // 读取 PWM
        uint32_t pwm_l = 0, pwm_r = 0;
        motor_control_get_last_pwm(&pwm_l, &pwm_r);

        // 读取舵机实际角度
        float steer_l = 0, steer_r = 0;
        steering_control_get_current_angles(&steer_l, &steer_r);

        // —— 以下连续 send 会被 SDK 200ms 合并窗口聚合为单条 publish ——
        send_number(KEY_PWM_L,   (double)pwm_l);
        send_number(KEY_PWM_R,   (double)pwm_r);
        send_number(KEY_STEER_L, (double)steer_l);
        send_number(KEY_STEER_R, (double)steer_r);
        send_number(KEY_ROLL,    (double)g_last_roll_deg);

        // 当前 PID 参数文本显示（便于 App 端实时查看）
        char buf[64];
        snprintf(buf, sizeof(buf), "s %.3f %.3f %.3f", g_steer_kp, g_steer_ki, g_steer_kd);
        send_string(KEY_STEER_PID, buf);
        snprintf(buf, sizeof(buf), "b %.3f %.3f %.3f", g_balance_kp, g_balance_ki, g_balance_kd);
        send_string(KEY_BAL_PID, buf);

        // 6 个 PID 参数独立数字控件
        send_number(KEY_SP, (double)g_steer_kp);
        send_number(KEY_SI, (double)g_steer_ki);
        send_number(KEY_SD, (double)g_steer_kd);
        send_number(KEY_BP, (double)g_balance_kp);
        send_number(KEY_BI, (double)g_balance_ki);
        send_number(KEY_BD, (double)g_balance_kd);

        // 急停状态文本
        send_string(KEY_ESTOP_ST, g_estop_active ? "STOPPED" : "RUNNING");

        vTaskDelayUntil(&xLast, period);
    }
}

// ============================================================
// 启动入口 —— 启动独立任务异步初始化（避免阻塞 app_main）
// ============================================================
static void blinker_init_task(void *arg)
{
    (void)arg;

    ESP_LOGW(TAG, "================ Blinker bridge starting ================");
    ESP_LOGW(TAG, "Auth Key: %s", CONFIG_BLINKER_AUTH_KEY);
    ESP_LOGW(TAG, "Server  : %s", CONFIG_BLINKER_SERVER_HOST);
    ESP_LOGW(TAG, "等待 WiFi 配网/连接 (SmartConfig)...");

    // 注册推力滑块
    blinker_widget_add(KEY_THRUST, BLINKER_SLIDER, on_thrust_slider);

    // 注册 PID 输入按钮（方式 1）
    blinker_widget_add(KEY_PID_IN, BLINKER_BUTTON, on_pid_input);

    // 注册急停按键
    blinker_widget_add(KEY_ESTOP, BLINKER_BUTTON, on_estop);

    // 注册差速转向三按钮
    blinker_widget_add(KEY_TURN_L, BLINKER_BUTTON, on_turn_left);
    blinker_widget_add(KEY_TURN_F, BLINKER_BUTTON, on_turn_forward);
    blinker_widget_add(KEY_TURN_R, BLINKER_BUTTON, on_turn_right);

    // 注册推力快速测试三按钮
    blinker_widget_add(KEY_FWD_P30, BLINKER_BUTTON, on_fwd_p30);
    blinker_widget_add(KEY_FWD_0,   BLINKER_BUTTON, on_fwd_0);
    blinker_widget_add(KEY_FWD_N30, BLINKER_BUTTON, on_fwd_n30);

    // 注册硬急停与重启按钮
    blinker_widget_add(KEY_HARD_STOP, BLINKER_BUTTON, on_hard_stop);
    blinker_widget_add(KEY_REBOOT,    BLINKER_BUTTON, on_reboot);

    // 注册在线校准按钮（长按生效）
    blinker_widget_add(KEY_CAL, BLINKER_BUTTON, on_cal);

    // 注册原始数据回调（方式 2：兼容 App 调试控件等任何文本输入）
    blinker_data_handler(on_raw_data);

    // 启动 Blinker 主流程（含 WiFi/MQTT，未配网会等待 SmartConfig，可能长时间阻塞）
    esp_err_t err = blinker_init();
    ESP_LOGW(TAG, "blinker_init 返回: %d", err);
    ESP_LOGW(TAG, "================ Blinker bridge ready ================");

    // 启动 1Hz 上报任务
    xTaskCreatePinnedToCore(blinker_report_task,
                            "blinker_report",
                            4096,
                            NULL,
                            3,
                            NULL,
                            0);

    // 初始化任务完成后退出
    vTaskDelete(NULL);
}

void blinker_bridge_start(void)
{
    // 用独立任务初始化，避免 blinker_init 内部阻塞影响 app_main
    // 优先级低于控制任务(5)，固定到 Core 0（WiFi 网络栈所在核心）
    xTaskCreatePinnedToCore(blinker_init_task,
                            "blinker_init",
                            8192,    // 较大栈，blinker_init 内部会跑 WiFi 初始化
                            NULL,
                            4,       // 略低于控制任务
                            NULL,
                            0);
}
