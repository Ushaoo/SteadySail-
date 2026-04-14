#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"

#define PWM_INPUT_PIN GPIO_NUM_4

#define MOTOR_PWM_GPIO       2
#define MOTOR_PWM_FREQ_HZ    50
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_12_BIT
#define MOTOR_PWM_NEUTRAL    1400
#define MOTOR_PWM_MAX        1480
#define MOTOR_PWM_MIN        1320

#define CAPTURE_MIN_PERIOD_US      200U
#define CAPTURE_MAX_PERIOD_US   200000U
#define CAPTURE_MIN_HIGH_US         10U
#define CAPTURE_EDGE_GLITCH_US      10U
#define CAPTURE_STALE_US        300000U

#define PID_LOOP_MS 10
#define PID_DEADBAND_DEG 3.0f
#define PID_DEADBAND_BLEND_DEG 3.0f
#define PID_OUTPUT_FILTER_TAU_S 0.12f

static const char *TAG = "PWM_CAPTURE";

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_period_us = 0;
static volatile uint32_t s_high_us = 0;
static volatile uint32_t s_last_frame_us = 0;
static volatile bool s_new_data = false;

static inline float clampf(float x, float lo, float hi)
{
    if (x < lo) {
        return lo;
    }
    if (x > hi) {
        return hi;
    }
    return x;
}

static float normalize_angle_360(float angle)
{
    while (angle < 0.0f) {
        angle += 360.0f;
    }
    while (angle >= 360.0f) {
        angle -= 360.0f;
    }
    return angle;
}

static float shortest_angle_error(float target, float current)
{
    float err = target - current;
    while (err > 180.0f) {
        err -= 360.0f;
    }
    while (err < -180.0f) {
        err += 360.0f;
    }
    return err;
}

static float apply_smooth_deadband(float err, float deadband, float blend)
{
    float abs_err = (err >= 0.0f) ? err : -err;
    float sign = (err >= 0.0f) ? 1.0f : -1.0f;

    if (abs_err <= deadband) {
        return 0.0f;
    }

    if (blend <= 0.0f || abs_err >= (deadband + blend)) {
        return sign * (abs_err - deadband);
    }

    {
        float x = abs_err - deadband;
        float t = x / blend;
        float s = t * t * (3.0f - 2.0f * t); /* smoothstep */
        return sign * (x * s);
    }
}

static uint32_t clamp_duty(int32_t duty)
{
    if (duty < MOTOR_PWM_MIN) {
        return MOTOR_PWM_MIN;
    }
    if (duty > MOTOR_PWM_MAX) {
        return MOTOR_PWM_MAX;
    }
    return (uint32_t)duty;
}

static float compute_angle_from_pwm(uint32_t period_us, uint32_t high_us)
{
    float duty_raw = ((float)high_us / (float)period_us) * 4119.0f - 16.0f;
    float angle = duty_raw * (360.0f / 4095.0f);
    return clampf(angle, 0.0f, 360.0f);
}

static void IRAM_ATTR pwm_isr_handler(void *arg)
{
    static uint64_t last_rise_us = 0;
    static uint64_t rise_time_us = 0;
    static uint64_t last_edge_us = 0;
    static uint32_t current_period_us = 0;
    static uint32_t prev_period_us = 0;

    uint64_t now_us = (uint64_t)esp_timer_get_time();
    int level = gpio_get_level((gpio_num_t)(uintptr_t)arg);

    if (last_edge_us != 0 && (now_us - last_edge_us) < CAPTURE_EDGE_GLITCH_US) {
        return;
    }
    last_edge_us = now_us;

    if (level == 1) {
        if (last_rise_us != 0) {
            uint64_t p = now_us - last_rise_us;
            if (p >= CAPTURE_MIN_PERIOD_US && p <= CAPTURE_MAX_PERIOD_US) {
                uint32_t period = (uint32_t)p;
                if (prev_period_us == 0 ||
                    (period >= (prev_period_us / 2U) && period <= (prev_period_us * 2U))) {
                    current_period_us = period;
                    prev_period_us = period;
                }
            }
        }
        last_rise_us = now_us;
        rise_time_us = now_us;
    } else {
        if (rise_time_us != 0 && current_period_us != 0) {
            uint64_t h = now_us - rise_time_us;
            if (h >= CAPTURE_MIN_HIGH_US && h <= current_period_us) {
                portENTER_CRITICAL_ISR(&s_lock);
                s_period_us = current_period_us;
                s_high_us = (uint32_t)h;
                s_last_frame_us = (uint32_t)now_us;
                s_new_data = true;
                portEXIT_CRITICAL_ISR(&s_lock);
            }
        }
    }
}

static esp_err_t pwm_capture_init(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << PWM_INPUT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    gpio_isr_handler_remove(PWM_INPUT_PIN);
    return gpio_isr_handler_add(PWM_INPUT_PIN, pwm_isr_handler, (void *)PWM_INPUT_PIN);
}

static void motor_pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t channel_cfg = {
        .gpio_num = MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = MOTOR_PWM_NEUTRAL,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
}

static void motor_set_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static bool get_latest_angle(float *angle_deg, uint32_t *period_us, uint32_t *high_us)
{
    uint32_t local_period = 0;
    uint32_t local_high = 0;
    uint32_t local_last_frame = 0;

    portENTER_CRITICAL(&s_lock);
    if (s_period_us > 0 && s_high_us > 0) {
        local_period = s_period_us;
        local_high = s_high_us;
        local_last_frame = s_last_frame_us;
    }
    portEXIT_CRITICAL(&s_lock);

    if (local_period == 0 || local_high == 0) {
        return false;
    }

    uint32_t now_us = (uint32_t)esp_timer_get_time();
    if ((now_us - local_last_frame) > CAPTURE_STALE_US) {
        return false;
    }

    *period_us = local_period;
    *high_us = local_high;
    *angle_deg = compute_angle_from_pwm(local_period, local_high);
    return true;
}

void run_pwm_capture_angle(void)
{
    esp_err_t ret = pwm_capture_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM capture init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "PWM capture started on GPIO %d", PWM_INPUT_PIN);

    while (1) {
        float angle = 0.0f;
        uint32_t period_us = 0;
        uint32_t high_us = 0;

        if (get_latest_angle(&angle, &period_us, &high_us)) {
            ESP_LOGI(TAG, "Realtime angle: %.2f deg", angle);
            ESP_LOGI(TAG, "Period: %.2f us, High: %.2f us", (float)period_us, (float)high_us);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void run_pwm_pid_angle_control(void)
{
    const float kp = 9.8f;
    const float ki = 1.0f;
    const float kd = 0.49f;
    const float dt = (float)PID_LOOP_MS / 1000.0f;
    const float out_alpha = dt / (PID_OUTPUT_FILTER_TAU_S + dt);

    float target_deg = 0.0f;
    float prev_err = 0.0f;
    float integ = 0.0f;
    float u_filt = 0.0f;
    bool target_initialized = false;

    char input_buf[32] = {0};
    size_t input_len = 0;

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    esp_err_t ret = pwm_capture_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM capture init failed: %s", esp_err_to_name(ret));
        return;
    }
    motor_pwm_init();
    motor_set_duty(0);
    vTaskDelay(pdMS_TO_TICKS(3000));
    motor_set_duty(MOTOR_PWM_NEUTRAL);
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "PID angle control started");
    ESP_LOGI(TAG, "Input target angle (0..360) in serial monitor and press Enter");
    ESP_LOGI(TAG, "Example: 90 or 270.5");

    TickType_t last_log_tick = 0;

    while (1) {
        int ch = 0;
        while ((ch = getchar()) != EOF) {
            if (ch == '\r' || ch == '\n') {
                if (input_len > 0) {
                    input_buf[input_len] = '\0';
                    char *endptr = NULL;
                    float cmd = strtof(input_buf, &endptr);
                    if (endptr != input_buf) {
                        target_deg = normalize_angle_360(cmd);
                        target_initialized = true;
                        integ = 0.0f;
                        prev_err = 0.0f;
                        u_filt = 0.0f;
                        ESP_LOGI(TAG, "New target: %.2f deg", target_deg);
                    } else {
                        ESP_LOGW(TAG, "Invalid input: %s", input_buf);
                    }
                    input_len = 0;
                }
            } else if (input_len < sizeof(input_buf) - 1U) {
                input_buf[input_len++] = (char)ch;
            }
        }

        float angle = 0.0f;
        uint32_t period_us = 0;
        uint32_t high_us = 0;
        bool valid_angle = get_latest_angle(&angle, &period_us, &high_us);

        if (valid_angle && !target_initialized) {
            target_deg = angle;
            target_initialized = true;
            ESP_LOGI(TAG, "Initial target set to current angle: %.2f deg", target_deg);
        }

        if (valid_angle && target_initialized) {
            float err_raw = shortest_angle_error(target_deg, angle);
            float err = apply_smooth_deadband(err_raw, PID_DEADBAND_DEG, PID_DEADBAND_BLEND_DEG);

            integ += err * dt;
            integ = clampf(integ, -120.0f, 120.0f);

            float deriv = (err - prev_err) / dt;
            prev_err = err;

            float u_raw = kp * err + ki * integ + kd * deriv;
            u_filt += out_alpha * (u_raw - u_filt);

            int32_t duty_cmd = (int32_t)(MOTOR_PWM_NEUTRAL - u_filt);
            uint32_t duty = clamp_duty(duty_cmd);
            motor_set_duty(duty);

            TickType_t now_tick = xTaskGetTickCount();
            if ((now_tick - last_log_tick) >= pdMS_TO_TICKS(200)) {
                ESP_LOGI(TAG,
                         "target=%.2f angle=%.2f err_raw=%.2f err_pid=%.2f u_raw=%.2f u_filt=%.2f duty=%u period=%u high=%u",
                         target_deg, angle, err_raw, err, u_raw, u_filt, (unsigned)duty,
                         (unsigned)period_us, (unsigned)high_us);
                last_log_tick = now_tick;
            }
        } else {
            u_filt = 0.0f;
            motor_set_duty(MOTOR_PWM_NEUTRAL);
        }

        vTaskDelay(pdMS_TO_TICKS(PID_LOOP_MS));
    }
}