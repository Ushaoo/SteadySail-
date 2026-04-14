/**
 * @file motor_controller.c
 * @brief 电机控制逻辑实现
 */

#include "motor_controller.h"
#include "config.h"
#include "esp_log.h"
#include <math.h>

static const char *TAG = "MOTOR";

esp_err_t motor_init(motor_controller_t *motor, pca9685_t *pwm_driver,
                     uint8_t left_thruster_ch, uint8_t right_thruster_ch,
                     uint8_t left_rotation_ch, uint8_t right_rotation_ch)
{
    if (motor == NULL || pwm_driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    motor->pwm_driver = pwm_driver;
    motor->left_thruster_ch = left_thruster_ch;
    motor->right_thruster_ch = right_thruster_ch;
    motor->left_rotation_ch = left_rotation_ch;
    motor->right_rotation_ch = right_rotation_ch;
    
    motor->left_thruster_invert = 1;    // 默认反转左推进器
    motor->right_thruster_invert = 0;
    
    motor->last_pulse_left = BASE_PULSE;
    motor->last_pulse_right = BASE_PULSE;
    motor->last_rot_angle_left = 0.0f;
    motor->last_rot_angle_right = 0.0f;
    motor->error_count = 0;

    // 初始化所有电机到中立位置
    pca9685_set_pulse(pwm_driver, left_thruster_ch, BASE_PULSE);
    pca9685_set_pulse(pwm_driver, right_thruster_ch, BASE_PULSE);
    pca9685_set_pulse(pwm_driver, left_rotation_ch, ROTATION_PULSE_CENTER);
    pca9685_set_pulse(pwm_driver, right_rotation_ch, ROTATION_PULSE_CENTER);

    ESP_LOGI(TAG, "Motor controller initialized");
    return ESP_OK;
}

esp_err_t motor_set_thrusters(motor_controller_t *motor, uint16_t pulse_left, uint16_t pulse_right)
{
    if (motor == NULL || motor->pwm_driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 限制范围
    if (pulse_left < MIN_PULSE) pulse_left = MIN_PULSE;
    if (pulse_left > MAX_PULSE) pulse_left = MAX_PULSE;
    if (pulse_right < MIN_PULSE) pulse_right = MIN_PULSE;
    if (pulse_right > MAX_PULSE) pulse_right = MAX_PULSE;

    // 应用反转处理
    uint16_t pulse_left_actual = motor->left_thruster_invert ? (3000 - pulse_left) : pulse_left;
    uint16_t pulse_right_actual = motor->right_thruster_invert ? (3000 - pulse_right) : pulse_right;

    // 设置左推进器
    esp_err_t ret = pca9685_set_pulse(motor->pwm_driver, motor->left_thruster_ch, pulse_left_actual);
    if (ret != ESP_OK) {
        motor->error_count++;
        ESP_LOGW(TAG, "Failed to set left thruster");
        return ret;
    }

    // 设置右推进器
    ret = pca9685_set_pulse(motor->pwm_driver, motor->right_thruster_ch, pulse_right_actual);
    if (ret != ESP_OK) {
        motor->error_count++;
        ESP_LOGW(TAG, "Failed to set right thruster");
        return ret;
    }

    motor->last_pulse_left = pulse_left;
    motor->last_pulse_right = pulse_right;

    return ESP_OK;
}

esp_err_t motor_set_rotation_angles(motor_controller_t *motor, float angle_left, float angle_right)
{
    if (motor == NULL || motor->pwm_driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // 限制角度范围
    if (angle_left < -MAX_ROTATION_ANGLE) angle_left = -MAX_ROTATION_ANGLE;
    if (angle_left > MAX_ROTATION_ANGLE) angle_left = MAX_ROTATION_ANGLE;
    if (angle_right < -MAX_ROTATION_ANGLE) angle_right = -MAX_ROTATION_ANGLE;
    if (angle_right > MAX_ROTATION_ANGLE) angle_right = MAX_ROTATION_ANGLE;

    // 转换角度为脉宽
    uint16_t pulse_left = angle_to_pulse(angle_left, ROTATION_PULSE_MIN, ROTATION_PULSE_MAX, ROTATION_PULSE_CENTER);
    uint16_t pulse_right = angle_to_pulse(angle_right, ROTATION_PULSE_MIN, ROTATION_PULSE_MAX, ROTATION_PULSE_CENTER);

    // 设置左旋转舵机
    esp_err_t ret = pca9685_set_pulse(motor->pwm_driver, motor->left_rotation_ch, pulse_left);
    if (ret != ESP_OK) {
        motor->error_count++;
        ESP_LOGW(TAG, "Failed to set left rotation servo");
        return ret;
    }

    // 设置右旋转舵机
    ret = pca9685_set_pulse(motor->pwm_driver, motor->right_rotation_ch, pulse_right);
    if (ret != ESP_OK) {
        motor->error_count++;
        ESP_LOGW(TAG, "Failed to set right rotation servo");
        return ret;
    }

    motor->last_rot_angle_left = angle_left;
    motor->last_rot_angle_right = angle_right;

    return ESP_OK;
}

esp_err_t motor_emergency_stop(motor_controller_t *motor)
{
    if (motor == NULL || motor->pwm_driver == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGW(TAG, "EMERGENCY STOP!");

    // 设置所有电机为中立
    motor_set_thrusters(motor, BASE_PULSE, BASE_PULSE);
    motor_set_rotation_angles(motor, 0.0f, 0.0f);

    return ESP_OK;
}

uint32_t motor_get_error_count(motor_controller_t *motor)
{
    if (motor == NULL) {
        return 0;
    }

    return motor->error_count;
}

uint16_t angle_to_pulse(float angle, uint16_t min_pulse, uint16_t max_pulse, uint16_t center_pulse)
{
    // 线性映射: -45° -> min_pulse, 0° -> center_pulse, +45° -> max_pulse
    float normalized_angle = angle / 45.0f;  // 归一化到 [-1, 1]

    uint16_t pulse;
    if (angle < 0.0f) {
        pulse = (uint16_t)(center_pulse + normalized_angle * (center_pulse - min_pulse));
    } else {
        pulse = (uint16_t)(center_pulse + normalized_angle * (max_pulse - center_pulse));
    }

    // 限制在范围内
    if (pulse < min_pulse) pulse = min_pulse;
    if (pulse > max_pulse) pulse = max_pulse;

    return pulse;
}

float pulse_to_angle(uint16_t pulse, uint16_t min_pulse, uint16_t max_pulse, uint16_t center_pulse)
{
    float angle;

    if (pulse < center_pulse) {
        angle = 45.0f * (float)(pulse - center_pulse) / (float)(center_pulse - min_pulse);
    } else {
        angle = 45.0f * (float)(pulse - center_pulse) / (float)(max_pulse - center_pulse);
    }

    // 限制范围
    if (angle < -45.0f) angle = -45.0f;
    if (angle > 45.0f) angle = 45.0f;

    return angle;
}
