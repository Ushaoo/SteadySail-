#include "rc_input.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

static const char *TAG = "RC_INPUT";

// ==================== 参数配置 ====================
// RC PWM 合法脉宽范围（μs）
#define RC_PULSE_MIN_US      800    // 低于此值视为噪声
#define RC_PULSE_MAX_US      2200   // 高于此值视为噪声
// 标准中位与量程（μs）
#define RC_CENTER_US         1500
#define RC_HALF_RANGE_US     500    // 1500-500=1000, 1500+500=2000
// 中位死区（μs）：±30μs 内视为 0
#define RC_DEADBAND_US       30
// 信号超时（ms）：超时后 get_throttle() 返回 0，is_valid() 返回 false
#define RC_SIGNAL_TIMEOUT_MS 200

// ==================== 定速巡航参数 ====================
#define RC_CRUISE_SETTLE_US  2000000LL  // 拨杆稳定 2s 后触发定速
#define RC_CRUISE_TOL_PCT    5.0f       // 定速追踪容差 ±5%

// ==================== 定速状态机 ====================
typedef enum {
    RC_CRUISE_IDLE,      // 拨杆近中位，无定速
    RC_CRUISE_SETTLING,  // 拨杆非零，计时 2s
    RC_CRUISE_LATCHED,   // 2s 已到，候松手触发定速
    RC_CRUISE_ACTIVE,    // 定速激活，松手保持定速值
} rc_cruise_state_t;

static rc_cruise_state_t s_cruise_state    = RC_CRUISE_IDLE;
static float             s_cruise_value    = 0.0f;  // 锁定的定速值
static float             s_settle_value    = 0.0f;  // 正在计时的稳停值
static int64_t           s_settle_start_us = 0;     // 计时起始时刻

// ==================== 内部状态 ====================
static volatile int64_t  s_rising_us  = 0;    // 上升沿时间戳
static volatile uint32_t s_pulse_us   = 0;    // 最新合法脉宽（μs）
static volatile int64_t  s_last_valid_us = 0; // 最近一次合法脉冲的系统时间
static volatile bool     s_isr_installed = false;

// ==================== GPIO ISR ====================
// 在双边沿中断中：上升沿记时，下降沿算脉宽
static void IRAM_ATTR rc_gpio_isr(void *arg)
{
    int64_t now = esp_timer_get_time();  // μs，64位，不溢出

    if (gpio_get_level((gpio_num_t)(intptr_t)arg)) {
        // 上升沿
        s_rising_us = now;
    } else {
        // 下降沿：计算脉宽
        if (s_rising_us > 0) {
            int64_t width = now - s_rising_us;
            if (width >= RC_PULSE_MIN_US && width <= RC_PULSE_MAX_US) {
                s_pulse_us       = (uint32_t)width;
                s_last_valid_us  = now;
            }
            s_rising_us = 0;
        }
    }
}

// ========================================================
// 公共接口
// ========================================================
esp_err_t rc_input_init(int gpio_num)
{
    gpio_config_t io_conf = {
        .pin_bit_mask  = (1ULL << gpio_num),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLDOWN_DISABLE,
        .pull_down_en  = GPIO_PULLDOWN_ENABLE,  // 无信号时保持低电平，防止误触发
        .intr_type     = GPIO_INTR_ANYEDGE,
    };
    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO%d 配置失败: %s", gpio_num, esp_err_to_name(err));
        return err;
    }

    err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE 表示 ISR service 已安装（main.c 里可能已装），忽略
        ESP_LOGE(TAG, "ISR service 安装失败: %s", esp_err_to_name(err));
        return err;
    }

    err = gpio_isr_handler_add((gpio_num_t)gpio_num, rc_gpio_isr,
                               (void *)(intptr_t)gpio_num);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ISR handler 注册失败: %s", esp_err_to_name(err));
        return err;
    }

    s_isr_installed = true;
    ESP_LOGI(TAG, "RC 油门输入初始化完成 (GPIO%d), 等待信号...", gpio_num);
    return ESP_OK;
}

bool rc_input_is_valid(void)
{
    if (!s_isr_installed || s_last_valid_us == 0) return false;
    int64_t now = esp_timer_get_time();
    return (now - s_last_valid_us) < ((int64_t)RC_SIGNAL_TIMEOUT_MS * 1000LL);
}

float rc_input_get_throttle(void)
{
    // 信号丢失：取消定速并归零
    if (!rc_input_is_valid()) {
        s_cruise_state = RC_CRUISE_IDLE;
        return 0.0f;
    }

    // 读取脉宽并计算原始制动値
    uint32_t pulse = s_pulse_us;
    int32_t  offset = (int32_t)pulse - RC_CENTER_US;
    float raw = 0.0f;
    if (offset > RC_DEADBAND_US || offset < -RC_DEADBAND_US) {
        raw = ((float)offset / (float)RC_HALF_RANGE_US) * 100.0f;
        if (raw >  100.0f) raw =  100.0f;
        if (raw < -100.0f) raw = -100.0f;
  
    }

    int64_t now = esp_timer_get_time();

    switch (s_cruise_state) {

    // ---- 拨杆近中位，无定速 ----
    case RC_CRUISE_IDLE:
        if (raw != 0.0f) {
            s_cruise_state    = RC_CRUISE_SETTLING;
            s_settle_value    = raw;
            s_settle_start_us = now;
        }
        return raw;  // 通常为 0

    // ---- 拨杆非零，计旲s ----
    case RC_CRUISE_SETTLING:
        if (raw == 0.0f) {
            // 2s 前松手，不定速
            s_cruise_state = RC_CRUISE_IDLE;
            return 0.0f;
        }
        if (fabsf(raw - s_settle_value) > RC_CRUISE_TOL_PCT) {
            // 拨杆移动：重置计时
            s_settle_value    = raw;
            s_settle_start_us = now;
        }
        if ((now - s_settle_start_us) >= RC_CRUISE_SETTLE_US) {
            // 等待2s，准备定速（松手时生效）
            s_cruise_value = s_settle_value;
            s_cruise_state = RC_CRUISE_LATCHED;
            ESP_LOGI(TAG, "定速候按 (%.1f%%)，松手即生效", s_cruise_value);
        }
        return raw;  // 拨杆期间始终跟随拨杆

    // ---- 定速就绪，候松手触发 ----
    case RC_CRUISE_LATCHED:
        if (raw == 0.0f) {
            // 松手：定速生效
            s_cruise_state = RC_CRUISE_ACTIVE;
            ESP_LOGI(TAG, "定速激活: %.1f%%", s_cruise_value);
            return s_cruise_value;
        }
        // 拨杆仍在非零区（含松手过程中的瞬态值）：跟随拨杆，等待归零
        return raw;

    // ---- 定速激活，松手保持 ----
    case RC_CRUISE_ACTIVE:
        if (raw == 0.0f) {
            // 拨杆在中位：维持定速値
            return s_cruise_value;
        }
        // 任意非零输入：取消定速，回 IDLE 等待拆杆归零后重新计时
        s_cruise_state = RC_CRUISE_IDLE;
        ESP_LOGI(TAG, "定速取消，等待归中后重新计时");
        return raw;

    default:
        s_cruise_state = RC_CRUISE_IDLE;
        return raw;
    }
}

bool rc_input_is_cruising(void)
{
    return (s_cruise_state == RC_CRUISE_ACTIVE);
}

void rc_input_cancel_cruise(void)
{
    s_cruise_state = RC_CRUISE_IDLE;
}
