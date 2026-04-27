#include "control_params.h"
#include "system_config.h"
#include "esp_log.h"

static const char *TAG = "PARAMS";

// --- 平衡 PID（默认值取自 system_config.h）---
volatile float g_balance_kp = PID_KP;
volatile float g_balance_ki = PID_KI;
volatile float g_balance_kd = PID_KD;

// --- 转向 PID（默认值与原 steering_control.c 内常量一致）---
volatile float g_steer_kp = 5.0f;
volatile float g_steer_ki = 1.0f;
volatile float g_steer_kd = 0.49f;

void control_params_init(void)
{
    ESP_LOGI(TAG, "PID 默认: balance(Kp=%.2f Ki=%.2f Kd=%.2f) steer(Kp=%.2f Ki=%.2f Kd=%.2f)",
             g_balance_kp, g_balance_ki, g_balance_kd,
             g_steer_kp, g_steer_ki, g_steer_kd);
}
