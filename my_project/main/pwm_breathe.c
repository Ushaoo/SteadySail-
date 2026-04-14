/**
 * @brief PWM wave profile:
 *        start at duty 1500, hold for 1 second,
 *        ramp up to 2000, ramp down to 1000,
 *        then keep ramping between 1000 and 2000.
 */

#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

#define PWM_GPIO       2                    /* output pin */
#define PWM_FREQ_HZ    50                 /* carrier frequency */
#define PWM_RESOLUTION LEDC_TIMER_12_BIT   /* 12-bit => max 4095 */
#define PWM_INIT_DUTY  1400                 /* initial duty       */
#define PWM_MAX_DUTY   1900                 /* upper bound        */
#define PWM_MIN_DUTY   900                 /* lower bound        */
#define PWM_STEP       5                  /* change per tick    */
#define PWM_STEP_MS    50                   /* ms between ticks   */
#define PWM_INIT_HOLD_MS 1000               /* hold at init duty  */

static const char *TAG = "PWM_BREATHE";

void run_pwm_breathe(void)
{
    /* ---- configure LEDC timer ---- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    /* ---- configure LEDC channel ---- */
    ledc_channel_config_t channel_cfg = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = PWM_INIT_DUTY,
        .hpoint     = 0,
    };
    ledc_channel_config(&channel_cfg);

    ESP_LOGI(TAG, "PWM started: GPIO%d freq=%d Hz", PWM_GPIO, PWM_FREQ_HZ);
    ESP_LOGI(TAG, "Profile: %d (hold %d ms) -> %d -> %d -> loop",
             PWM_INIT_DUTY, PWM_INIT_HOLD_MS, PWM_MAX_DUTY, PWM_MIN_DUTY);

    uint32_t duty     = PWM_INIT_DUTY;
    int      rising   = 1;  /* 1 = duty increasing, 0 = duty decreasing */

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "duty=%u", (unsigned)duty);
    vTaskDelay(pdMS_TO_TICKS(PWM_INIT_HOLD_MS));

    duty     = PWM_MAX_DUTY;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "duty=%u", (unsigned)duty);
    vTaskDelay(pdMS_TO_TICKS(PWM_INIT_HOLD_MS));


    duty     = PWM_MIN_DUTY;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "duty=%u", (unsigned)duty);
    vTaskDelay(pdMS_TO_TICKS(PWM_INIT_HOLD_MS));

    duty     = PWM_INIT_DUTY;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "duty=%u", (unsigned)duty);
    vTaskDelay(pdMS_TO_TICKS(PWM_INIT_HOLD_MS));

    while (1) {
        if (rising) {
            if (duty + PWM_STEP >= PWM_MAX_DUTY) {
                duty = PWM_MAX_DUTY;
            } else {
                duty += PWM_STEP;
            }
            if (duty >= PWM_MAX_DUTY) {
                rising = 0;
            }
        } else {
            if (duty <= PWM_MIN_DUTY + PWM_STEP) {
                duty = PWM_MIN_DUTY;
            } else {
                duty -= PWM_STEP;
            }
            if (duty <= PWM_MIN_DUTY) {
                rising = 1;
            }
        }

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ESP_LOGI(TAG, "duty=%u", (unsigned)duty);

        vTaskDelay(pdMS_TO_TICKS(PWM_STEP_MS));
    }
}

/**
 * @brief Keyboard-controlled motor direction using arrow keys
 *        UP/DOWN arrows control motor speed and direction
 *        LEFT arrow: decrease speed, RIGHT arrow: increase speed
 */
void run_pwm_keyboard_control(void)
{
    /* UART0 is already initialized by the console system, no need to reconfigure */
    
    /* ---- configure LEDC timer ---- */
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz         = PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    /* ---- configure LEDC channel ---- */
    ledc_channel_config_t channel_cfg = {
        .gpio_num   = PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = PWM_INIT_DUTY,
        .hpoint     = 0,
    };
    ledc_channel_config(&channel_cfg);

    ESP_LOGI(TAG, "Keyboard-controlled PWM started: GPIO%d", PWM_GPIO);
    ESP_LOGI(TAG, "UP: forward, DOWN: reverse, LEFT: decrease, RIGHT: increase, 'q': quit");

    uint32_t duty = PWM_INIT_DUTY;
    int data;
    uint8_t esc_seq[3] = {0};  /* buffer for escape sequence */
    uint8_t esc_idx = 0;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_LOGI(TAG, "Initial duty=%u", (unsigned)duty);

    while (1) {
        /* Read character from console input */
        data = getchar();
        
        if (data != EOF) {
            /* Handle escape sequences for arrow keys */
            if (data == 0x1b) {  /* ESC character */
                esc_idx = 0;
                esc_seq[esc_idx++] = data;
            } else if (esc_idx > 0) {
                esc_seq[esc_idx++] = data;
                if (esc_idx >= 3) {
                    /* Complete escape sequence */
                    if (esc_seq[0] == 0x1b && esc_seq[1] == 0x5b) {
                        switch (esc_seq[2]) {
                            case 0x41:  /* UP arrow - increase duty (forward) */
                                if (duty + PWM_STEP <= PWM_MAX_DUTY) {
                                    duty += PWM_STEP;
                                } else {
                                    duty = PWM_MAX_DUTY;
                                }
                                printf("UP: duty=%u\n", (unsigned)duty);
                                break;
                            case 0x42:  /* DOWN arrow - decrease duty (reverse) */
                                if (duty >= PWM_MIN_DUTY + PWM_STEP) {
                                    duty -= PWM_STEP;
                                } else {
                                    duty = PWM_MIN_DUTY;
                                }
                                printf("DOWN: duty=%u\n", (unsigned)duty);
                                break;
                            case 0x43:  /* RIGHT arrow - increase speed */
                                if (duty + PWM_STEP*2 <= PWM_MAX_DUTY) {
                                    duty += PWM_STEP*2;
                                } else {
                                    duty = PWM_MAX_DUTY;
                                }
                                printf("RIGHT: duty=%u\n", (unsigned)duty);
                                break;
                            case 0x44:  /* LEFT arrow - decrease speed */
                                if (duty >= PWM_MIN_DUTY + PWM_STEP*2) {
                                    duty -= PWM_STEP*2;
                                } else {
                                    duty = PWM_MIN_DUTY;
                                }
                                printf("LEFT: duty=%u\n", (unsigned)duty);
                                break;
                            default:
                                break;
                        }
                    }
                    esc_idx = 0;
                }
            } else if (data == 'q' || data == 'Q') {
                /* Exit keyboard control mode */
                ESP_LOGI(TAG, "Exiting keyboard control mode");
                break;
            } else if (data == ' ') {
                /* Space: stop motor */
                duty = PWM_INIT_DUTY;
                printf("STOP: duty=%u\n", (unsigned)duty);
            }

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        }
    }
}

/**
 * @brief Menu to select between breathing mode and keyboard control mode
 */
int pwm_mode_selector(void)
{
    int data;
    
    printf("\n========== PWM Control Menu ==========\n");
    printf("Select mode:\n");
    printf("  1 - Breathing mode (automatic)\n");
    printf("  2 - Keyboard control mode\n");
    printf("=====================================\n\n");

    while (1) {
        data = getchar();
        if (data == '1') {
            printf("Selected: Breathing mode\n");
            return 1;
        } else if (data == '2') {
            printf("Selected: Keyboard control mode\n");
            return 2;
        } else if (data != '\n' && data != '\r') {
            printf("Invalid input, please press 1 or 2\n");
        }
    }
}
