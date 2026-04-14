/**
 * @file pid_controller.c
 * @brief 2DOF PID 控制器实现
 * 
 * 参考树莓派版本的 PID2DOF 类
 */

#include "pid_controller.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "PID";

void pid_init(pid_controller_t *pid, float Kp, float Ki, float Kd, float b, float c, float dt)
{
    if (pid == NULL) {
        return;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->b = b;
    pid->c = c;
    pid->dt = dt;
    pid->integral = 0.0f;

    ESP_LOGI(TAG, "PID initialized: Kp=%.2f, Ki=%.2f, Kd=%.2f, b=%.2f, c=%.2f, dt=%.4f",
             Kp, Ki, Kd, b, c, dt);
}

float pid_update(pid_controller_t *pid, float setpoint, float measured, float omega_filtered)
{
    if (pid == NULL) {
        return 0.0f;
    }

    float error = setpoint - measured;

    // 比例项 (2DOF: b*setpoint - measured)
    float proportional = pid->Kp * (pid->b * setpoint - measured);

    // 积分项
    pid->integral += error * pid->dt;
    float integral_term = pid->Ki * pid->integral;

    // 微分项 (2DOF: c*setpoint_derivative - measured_derivative)
    // 由于 setpoint 为常数，c*setpoint_derivative = 0
    float derivative = pid->Kd * (pid->c * 0.0f - omega_filtered);

    float output = proportional + integral_term + derivative;

    return output;
}

void pid_update_gains(pid_controller_t *pid, float Kp, float Ki, float Kd)
{
    if (pid == NULL) {
        return;
    }

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    ESP_LOGI(TAG, "PID gains updated: Kp=%.2f, Ki=%.2f, Kd=%.2f", Kp, Ki, Kd);
}

void pid_reset(pid_controller_t *pid)
{
    if (pid == NULL) {
        return;
    }

    pid->integral = 0.0f;
}

float pid_get_integral(pid_controller_t *pid)
{
    if (pid == NULL) {
        return 0.0f;
    }

    return pid->integral;
}
