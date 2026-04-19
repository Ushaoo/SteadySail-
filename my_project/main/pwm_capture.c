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
#include "blinker_api.h"

#define SYSTEM_COUNT 2

static const gpio_num_t ENCODER_INPUT_PINS[SYSTEM_COUNT] = {
    GPIO_NUM_4,
    GPIO_NUM_5,
};

static const int MOTOR_PWM_GPIOS[SYSTEM_COUNT] = {
    1,
    2,
};

static const int MOTOR_DIR[SYSTEM_COUNT] = {
    1,
    -1,
};

static const ledc_channel_t MOTOR_PWM_CHANNELS[SYSTEM_COUNT] = {
    LEDC_CHANNEL_0,
    LEDC_CHANNEL_1,
};

static const int THRUSTER_PWM_GPIOS[SYSTEM_COUNT] = {
    15,
    16,
};

static const ledc_channel_t THRUSTER_PWM_CHANNELS[SYSTEM_COUNT] = {
    LEDC_CHANNEL_2,
    LEDC_CHANNEL_3,
};

#define MOTOR_PWM_FREQ_HZ    50
#define MOTOR_PWM_RESOLUTION LEDC_TIMER_12_BIT
#define MOTOR_PWM_NEUTRAL    1400
#define MOTOR_PWM_MAX        1480
#define MOTOR_PWM_MIN        1320

#define THRUSTER_PWM_FREQ_HZ    50
#define THRUSTER_PWM_RESOLUTION LEDC_TIMER_12_BIT
#define THRUSTER_PWM_NEUTRAL    1500
#define THRUSTER_PWM_MAX        1600
#define THRUSTER_PWM_MIN        1400

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

typedef struct {
    portMUX_TYPE lock;
    volatile uint32_t period_us;
    volatile uint32_t high_us;
    volatile uint32_t last_frame_us;

    volatile uint64_t last_rise_us;
    volatile uint64_t rise_time_us;
    volatile uint64_t last_edge_us;
    volatile uint32_t current_period_us;
    volatile uint32_t prev_period_us;
} capture_state_t;

typedef struct {
    int index;
    gpio_num_t pin;
} isr_ctx_t;

static capture_state_t s_capture[SYSTEM_COUNT] = {
    {
        .lock = portMUX_INITIALIZER_UNLOCKED,
    },
    {
        .lock = portMUX_INITIALIZER_UNLOCKED,
    },
};

static isr_ctx_t s_isr_ctx[SYSTEM_COUNT];
static portMUX_TYPE s_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
static volatile int8_t s_thruster_cmd[SYSTEM_COUNT] = {0, 0};
static volatile bool s_target_update_pending = false;
static volatile float s_target_from_blinker = 0.0f;
static volatile bool s_stop_requested = false;

static void blinker_data_callback(const char *data)
{
    cJSON *root = cJSON_Parse(data);
    if (root == NULL) {
        ESP_LOGW(TAG, "Invalid blinker data: %s", data);
        return;
    }

    cJSON *item = NULL;

    item = cJSON_GetObjectItem(root, "btn-lfw");
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        portENTER_CRITICAL(&s_cmd_lock);
        if (strcmp(item->valuestring, "pressup") == 0) {
            s_thruster_cmd[0] = 0;
        } else {
            s_thruster_cmd[0] = 1;
        }
        portEXIT_CRITICAL(&s_cmd_lock);
    }

    item = cJSON_GetObjectItem(root, "btn-lbk");
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        portENTER_CRITICAL(&s_cmd_lock);
        if (strcmp(item->valuestring, "pressup") == 0) {
            s_thruster_cmd[0] = 0;
        } else {
            s_thruster_cmd[0] = -1;
        }
        portEXIT_CRITICAL(&s_cmd_lock);
    }

    item = cJSON_GetObjectItem(root, "btn-rfw");
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        portENTER_CRITICAL(&s_cmd_lock);
        if (strcmp(item->valuestring, "pressup") == 0) {
            s_thruster_cmd[1] = 0;
        } else {
            s_thruster_cmd[1] = 1;
        }
        portEXIT_CRITICAL(&s_cmd_lock);
    }

    item = cJSON_GetObjectItem(root, "btn-rbk");
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        portENTER_CRITICAL(&s_cmd_lock);
        if (strcmp(item->valuestring, "pressup") == 0) {
            s_thruster_cmd[1] = 0;
        } else {
            s_thruster_cmd[1] = -1;
        }
        portEXIT_CRITICAL(&s_cmd_lock);
    }

    item = cJSON_GetObjectItem(root, "ran-ang");
    if (cJSON_IsNumber(item)) {
        portENTER_CRITICAL(&s_cmd_lock);
        s_target_from_blinker = (float)item->valuedouble;
        s_target_update_pending = true;
        portEXIT_CRITICAL(&s_cmd_lock);
    }

    item = cJSON_GetObjectItem(root, "btn-stp");
    if (cJSON_IsString(item) && item->valuestring != NULL) {
        if (strcmp(item->valuestring, "tap") == 0 || strcmp(item->valuestring, "press") == 0) {
            portENTER_CRITICAL(&s_cmd_lock);
            s_stop_requested = true;
            s_thruster_cmd[0] = 0;
            s_thruster_cmd[1] = 0;
            portEXIT_CRITICAL(&s_cmd_lock);
        }
    }

    cJSON_Delete(root);
}

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

static uint32_t clamp_thruster_duty(int32_t duty)
{
    if (duty < THRUSTER_PWM_MIN) {
        return THRUSTER_PWM_MIN;
    }
    if (duty > THRUSTER_PWM_MAX) {
        return THRUSTER_PWM_MAX;
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
    isr_ctx_t *ctx = (isr_ctx_t *)arg;
    capture_state_t *s = &s_capture[ctx->index];
    uint64_t now_us = (uint64_t)esp_timer_get_time();
    int level = gpio_get_level(ctx->pin);

    if (s->last_edge_us != 0 && (now_us - s->last_edge_us) < CAPTURE_EDGE_GLITCH_US) {
        return;
    }
    s->last_edge_us = now_us;

    if (level == 1) {
        if (s->last_rise_us != 0) {
            uint64_t p = now_us - s->last_rise_us;
            if (p >= CAPTURE_MIN_PERIOD_US && p <= CAPTURE_MAX_PERIOD_US) {
                uint32_t period = (uint32_t)p;
                if (s->prev_period_us == 0 ||
                    (period >= (s->prev_period_us / 2U) && period <= (s->prev_period_us * 2U))) {
                    s->current_period_us = period;
                    s->prev_period_us = period;
                }
            }
        }
        s->last_rise_us = now_us;
        s->rise_time_us = now_us;
    } else {
        if (s->rise_time_us != 0 && s->current_period_us != 0) {
            uint64_t h = now_us - s->rise_time_us;
            if (h >= CAPTURE_MIN_HIGH_US && h <= s->current_period_us) {
                portENTER_CRITICAL_ISR(&s->lock);
                s->period_us = s->current_period_us;
                s->high_us = (uint32_t)h;
                s->last_frame_us = (uint32_t)now_us;
                portEXIT_CRITICAL_ISR(&s->lock);
            }
        }
    }
}

static esp_err_t pwm_capture_init_all(void)
{
    gpio_config_t io_cfg = {
        .pin_bit_mask = 0,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };

    for (int i = 0; i < SYSTEM_COUNT; ++i) {
        io_cfg.pin_bit_mask |= (1ULL << ENCODER_INPUT_PINS[i]);
    }
    ESP_ERROR_CHECK(gpio_config(&io_cfg));

    esp_err_t ret = gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }

    for (int i = 0; i < SYSTEM_COUNT; ++i) {
        s_isr_ctx[i].index = i;
        s_isr_ctx[i].pin = ENCODER_INPUT_PINS[i];
        gpio_isr_handler_remove(ENCODER_INPUT_PINS[i]);
        ret = gpio_isr_handler_add(ENCODER_INPUT_PINS[i], pwm_isr_handler, &s_isr_ctx[i]);
        if (ret != ESP_OK) {
            return ret;
        }
    }

    return ESP_OK;
}

static void motor_pwm_init_all(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = MOTOR_PWM_RESOLUTION,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (int i = 0; i < SYSTEM_COUNT; ++i) {
        ledc_channel_config_t channel_cfg = {
            .gpio_num = MOTOR_PWM_GPIOS[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = MOTOR_PWM_CHANNELS[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = MOTOR_PWM_NEUTRAL,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    }
}

static void motor_set_duty(int index, uint32_t duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNELS[index], duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, MOTOR_PWM_CHANNELS[index]);
}

static void thruster_pwm_init_all(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = THRUSTER_PWM_RESOLUTION,
        .freq_hz = THRUSTER_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    for (int i = 0; i < SYSTEM_COUNT; ++i) {
        ledc_channel_config_t channel_cfg = {
            .gpio_num = THRUSTER_PWM_GPIOS[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = THRUSTER_PWM_CHANNELS[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_1,
            .duty = THRUSTER_PWM_NEUTRAL,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_cfg));
    }
}

static void thruster_set_duty(int index, uint32_t duty)
{
    uint32_t clamped = clamp_thruster_duty((int32_t)duty);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, THRUSTER_PWM_CHANNELS[index], clamped);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, THRUSTER_PWM_CHANNELS[index]);
}

static bool get_latest_angle(int index, float *angle_deg, uint32_t *period_us, uint32_t *high_us)
{
    capture_state_t *s = &s_capture[index];
    uint32_t local_period = 0;
    uint32_t local_high = 0;
    uint32_t local_last_frame = 0;

    portENTER_CRITICAL(&s->lock);
    if (s->period_us > 0 && s->high_us > 0) {
        local_period = s->period_us;
        local_high = s->high_us;
        local_last_frame = s->last_frame_us;
    }
    portEXIT_CRITICAL(&s->lock);

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
    esp_err_t ret = pwm_capture_init_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM capture init failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Dual PWM capture started on GPIO %d and GPIO %d",
             ENCODER_INPUT_PINS[0], ENCODER_INPUT_PINS[1]);

    while (1) {
        float angle[2] = {0};
        uint32_t period_us[2] = {0};
        uint32_t high_us[2] = {0};

        bool ok0 = get_latest_angle(0, &angle[0], &period_us[0], &high_us[0]);
        bool ok1 = get_latest_angle(1, &angle[1], &period_us[1], &high_us[1]);

        if (ok0 || ok1) {
            ESP_LOGI(TAG,
                     "enc0: %s angle=%.2f p=%u h=%u | enc1: %s angle=%.2f p=%u h=%u",
                     ok0 ? "ok" : "na",
                     angle[0],
                     (unsigned)period_us[0],
                     (unsigned)high_us[0],
                     ok1 ? "ok" : "na",
                     angle[1],
                     (unsigned)period_us[1],
                     (unsigned)high_us[1]);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void run_pwm_pid_angle_control(void)
{
    const float kp = 6.8f;
    const float ki = 1.0f;
    const float kd = 0.49f;
    const float dt = (float)PID_LOOP_MS / 1000.0f;
    const float out_alpha = dt / (PID_OUTPUT_FILTER_TAU_S + dt);

    float target_deg[SYSTEM_COUNT] = {0};
    float prev_err[SYSTEM_COUNT] = {0};
    float integ[SYSTEM_COUNT] = {0};
    float u_filt[SYSTEM_COUNT] = {0};
    bool target_initialized[SYSTEM_COUNT] = {false};

    char input_buf[64] = {0};
    size_t input_len = 0;

    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    esp_err_t ret = pwm_capture_init_all();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PWM capture init failed: %s", esp_err_to_name(ret));
        return;
    }

    blinker_init();
    blinker_data_handler(blinker_data_callback);

    motor_pwm_init_all();
    thruster_pwm_init_all();

    for (int i = 0; i < SYSTEM_COUNT; ++i) {
        motor_set_duty(i, 0);
        thruster_set_duty(i, THRUSTER_PWM_NEUTRAL);
    }

    ESP_LOGI(TAG, "Thruster PWM ready: pin %d and %d, neutral=%d, range=%d..%d",
             THRUSTER_PWM_GPIOS[0],
             THRUSTER_PWM_GPIOS[1],
             THRUSTER_PWM_NEUTRAL,
             THRUSTER_PWM_MIN,
             THRUSTER_PWM_MAX);

    vTaskDelay(pdMS_TO_TICKS(3000));
    for (int i = 0; i < SYSTEM_COUNT; ++i) {
        motor_set_duty(i, MOTOR_PWM_NEUTRAL);
    }
    vTaskDelay(pdMS_TO_TICKS(3000));

    ESP_LOGI(TAG, "Dual PID started: motor GPIO1<->encoder GPIO4, motor GPIO2<->encoder GPIO5");
    ESP_LOGI(TAG, "Serial input format: <target0>,<target1>  Example: 90,270");
    ESP_LOGI(TAG, "If only one value is given, it is applied to both motors");

    TickType_t last_log_tick = 0;

    while (1) {
        int8_t thruster_cmd_local[SYSTEM_COUNT] = {0};
        uint32_t thruster_duty_local[SYSTEM_COUNT] = {THRUSTER_PWM_NEUTRAL, THRUSTER_PWM_NEUTRAL};
        bool stop_requested_local = false;
        bool target_update_local = false;
        float blinker_target_local = 0.0f;

        portENTER_CRITICAL(&s_cmd_lock);
        thruster_cmd_local[0] = s_thruster_cmd[0];
        thruster_cmd_local[1] = s_thruster_cmd[1];
        stop_requested_local = s_stop_requested;
        target_update_local = s_target_update_pending;
        blinker_target_local = s_target_from_blinker;
        s_target_update_pending = false;
        portEXIT_CRITICAL(&s_cmd_lock);

        for (int i = 0; i < SYSTEM_COUNT; ++i) {
            uint32_t thruster_duty = THRUSTER_PWM_NEUTRAL;
            if (thruster_cmd_local[i] > 0) {
                thruster_duty = THRUSTER_PWM_MAX;
            } else if (thruster_cmd_local[i] < 0) {
                thruster_duty = THRUSTER_PWM_MIN;
            }
            thruster_duty_local[i] = thruster_duty;
            thruster_set_duty(i, thruster_duty);
        }

        if (target_update_local) {
            target_deg[0] = normalize_angle_360(blinker_target_local);
            target_deg[1] = normalize_angle_360(blinker_target_local);
            target_initialized[0] = true;
            target_initialized[1] = true;
            integ[0] = 0.0f;
            integ[1] = 0.0f;
            prev_err[0] = 0.0f;
            prev_err[1] = 0.0f;
            u_filt[0] = 0.0f;
            u_filt[1] = 0.0f;
            ESP_LOGI(TAG, "Blinker target angle updated: %.2f deg", blinker_target_local);
        }

        if (stop_requested_local) {
            for (int i = 0; i < SYSTEM_COUNT; ++i) {
                motor_set_duty(i, MOTOR_PWM_NEUTRAL);
                thruster_set_duty(i, THRUSTER_PWM_NEUTRAL);
            }
            sleep(10);
            for (int i = 0; i < SYSTEM_COUNT; ++i) {
                motor_set_duty(i, 0);
                thruster_set_duty(i, 0);
            }

            ESP_LOGI(TAG, "Received btn-stp, stop requested, exiting control loop");
            break;
        }

        int ch = 0;
        while ((ch = getchar()) != EOF) {
            if (ch == '\r' || ch == '\n') {
                if (input_len > 0) {
                    input_buf[input_len] = '\0';
                    float t0 = 0.0f;
                    float t1 = 0.0f;
                    int parsed = sscanf(input_buf, "%f%*[, ]%f", &t0, &t1);

                    if (parsed == 2) {
                        target_deg[0] = normalize_angle_360(t0);
                        target_deg[1] = normalize_angle_360(t1);
                        target_initialized[0] = true;
                        target_initialized[1] = true;
                        integ[0] = 0.0f;
                        integ[1] = 0.0f;
                        prev_err[0] = 0.0f;
                        prev_err[1] = 0.0f;
                        u_filt[0] = 0.0f;
                        u_filt[1] = 0.0f;
                        ESP_LOGI(TAG, "New targets: motor0=%.2f deg, motor1=%.2f deg",
                                 target_deg[0], target_deg[1]);
                    } else {
                        char *endptr = NULL;
                        float one = strtof(input_buf, &endptr);
                        if (endptr != input_buf) {
                            target_deg[0] = normalize_angle_360(one);
                            target_deg[1] = normalize_angle_360(one);
                            target_initialized[0] = true;
                            target_initialized[1] = true;
                            integ[0] = 0.0f;
                            integ[1] = 0.0f;
                            prev_err[0] = 0.0f;
                            prev_err[1] = 0.0f;
                            u_filt[0] = 0.0f;
                            u_filt[1] = 0.0f;
                            ESP_LOGI(TAG, "New shared target: %.2f deg", one);
                        } else {
                            ESP_LOGW(TAG, "Invalid input: %s", input_buf);
                        }
                    }

                    input_len = 0;
                    memset(input_buf, 0, sizeof(input_buf));
                }
            } else if (input_len < sizeof(input_buf) - 1U) {
                input_buf[input_len++] = (char)ch;
            }
        }

        float angle[SYSTEM_COUNT] = {0};
        uint32_t period_us[SYSTEM_COUNT] = {0};
        uint32_t high_us[SYSTEM_COUNT] = {0};
        bool valid[SYSTEM_COUNT] = {false};
        uint32_t duty[SYSTEM_COUNT] = {MOTOR_PWM_NEUTRAL, MOTOR_PWM_NEUTRAL};
        float err_raw[SYSTEM_COUNT] = {0};
        float err_pid[SYSTEM_COUNT] = {0};
        float u_raw[SYSTEM_COUNT] = {0};

        for (int i = 0; i < SYSTEM_COUNT; ++i) {
            valid[i] = get_latest_angle(i, &angle[i], &period_us[i], &high_us[i]);

            if (valid[i] && !target_initialized[i]) {
                target_deg[i] = angle[i];
                target_initialized[i] = true;
                ESP_LOGI(TAG, "motor%d initial target = %.2f deg", i, target_deg[i]);
            }

            if (valid[i] && target_initialized[i]) {
                err_raw[i] = shortest_angle_error(target_deg[i], angle[i]);
                err_pid[i] = apply_smooth_deadband(err_raw[i], PID_DEADBAND_DEG, PID_DEADBAND_BLEND_DEG);

                integ[i] += err_pid[i] * dt;
                integ[i] = clampf(integ[i], -120.0f, 120.0f);

                float deriv = (err_pid[i] - prev_err[i]) / dt;
                prev_err[i] = err_pid[i];

                u_raw[i] = kp * err_pid[i] + ki * integ[i] + kd * deriv;
                u_filt[i] += out_alpha * (u_raw[i] - u_filt[i]);

                int32_t duty_cmd = (int32_t)(MOTOR_PWM_NEUTRAL - (MOTOR_DIR[i] * u_filt[i]));
                duty[i] = clamp_duty(duty_cmd);
                motor_set_duty(i, duty[i]);
            } else {
                u_filt[i] = 0.0f;
                motor_set_duty(i, MOTOR_PWM_NEUTRAL);
                duty[i] = MOTOR_PWM_NEUTRAL;
            }
        }

        TickType_t now_tick = xTaskGetTickCount();
        if ((now_tick - last_log_tick) >= pdMS_TO_TICKS(200)) {
            ESP_LOGI(TAG,
                     "m0 tgt=%.1f ang=%.1f err=%.1f duty=%u val=%d | m1 tgt=%.1f ang=%.1f err=%.1f duty=%u val=%d | th0=%u th1=%u",
                     target_deg[0], angle[0], err_raw[0], (unsigned)duty[0], valid[0],
                     target_deg[1], angle[1], err_raw[1], (unsigned)duty[1], valid[1],
                     (unsigned)thruster_duty_local[0], (unsigned)thruster_duty_local[1]);
            last_log_tick = now_tick;
        }

        vTaskDelay(pdMS_TO_TICKS(PID_LOOP_MS));
    }
}


