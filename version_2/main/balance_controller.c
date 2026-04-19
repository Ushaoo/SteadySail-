#include "balance_controller.h"
#include "system_config.h"
#include <math.h>

// --- 双 IMU Mahony 算法全局变量 ---
static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float prev_omega_filtered = 0.0f;
static float pid_integral = 0.0f;

#define ALPHA_ACC  0.98f
#define WEIGHT_DYN 0.8f
#define ALPHA_EMA  0.15f
#define K_SELF     100.0f
#define INERTIA    (SYS_MASS * (SYS_WIDTH / 2.0f) * (SYS_WIDTH / 2.0f) / 3.0f) // 近似转动惯量 2.4

// 将弧度限制在 -PI 到 PI
static inline float constrain_rad(float x) {
    if (x > M_PI) x -= 2.0f * M_PI;
    if (x < -M_PI) x += 2.0f * M_PI;
    return x;
}

// 提取角速度方差的核心算法：滑动窗口估计 (此处为简化版实现：快速指数平均方差)
static float imu1_var = 1.0f, imu2_var = 1.0f;

// 非线性死区平滑插值 (Smoothstep)
static float apply_deadzone_smooth(float value, float core, float soft) {
    float abs_val = fabsf(value);
    if (abs_val < core) {
        return 0.0f;
    } else if (abs_val >= soft) {
        return value;
    } else {
        float t = (abs_val - core) / (soft - core);
        float smooth_factor = t * t * (3.0f - 2.0f * t);
        return (value > 0 ? 1.0f : -1.0f) * abs_val * smooth_factor;
    }
}

void balance_controller_init(void) {
    // 重置四元数和积分器
    q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
    prev_omega_filtered = 0.0f;
    pid_integral = 0.0f;
    imu1_var = 1.0f; imu2_var = 1.0f;
}

void balance_controller_update(dual_imu_data_t *imu_data, balance_state_t *state) {
    // **********************************************
    // 1. 双 IMU 数据融合 (方差加权 + Mahony 滤波)
    // **********************************************
    // 简单指数移动方差更新 (模拟 100个样本缓冲区的实时方差)
    imu1_var = 0.95f * imu1_var + 0.05f * (imu_data->imu1.gyro_x * imu_data->imu1.gyro_x);
    imu2_var = 0.95f * imu2_var + 0.05f * (imu_data->imu2.gyro_x * imu_data->imu2.gyro_x);

    // 加速度互补
    float ax = ALPHA_ACC * imu_data->imu1.accel_x + (1.0f - ALPHA_ACC) * imu_data->imu2.accel_x;
    float ay = ALPHA_ACC * imu_data->imu1.accel_y + (1.0f - ALPHA_ACC) * imu_data->imu2.accel_y;
    float az = ALPHA_ACC * imu_data->imu1.accel_z + (1.0f - ALPHA_ACC) * imu_data->imu2.accel_z;
    
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm > 0.01f) { ax /= norm; ay /= norm; az /= norm; }
    
    // 角速度最优加权
    float inv_var1 = 1.0f / (imu1_var + 1e-6f);
    float inv_var2 = 1.0f / (imu2_var + 1e-6f);
    float w1 = inv_var1 / (inv_var1 + inv_var2);
    
    float acc_error = fabsf(norm - 1.0f);
    if (acc_error > 0.3f) w1 = (inv_var1 > inv_var2) ? WEIGHT_DYN : (1.0f - WEIGHT_DYN);
    float w2 = 1.0f - w1;

    float gx = w1 * imu_data->imu1.gyro_x + w2 * imu_data->imu2.gyro_x;
    float gy = w1 * imu_data->imu1.gyro_y + w2 * imu_data->imu2.gyro_y;
    float gz = w1 * imu_data->imu1.gyro_z + w2 * imu_data->imu2.gyro_z;

    // 转弧度进行 Mahony 更新
    float gx_rad = gx * M_PI / 180.0f;
    float gy_rad = gy * M_PI / 180.0f;
    float gz_rad = gz * M_PI / 180.0f;

    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;

    float Kp_mahony = 2.0f + 25.0f * acc_error;
    gx_rad += Kp_mahony * ex;
    gy_rad += Kp_mahony * ey;
    gz_rad += Kp_mahony * ez;

    // 四元数积分计算
    q0 += 0.5f * CONTROL_DT * (-q1*gx_rad - q2*gy_rad - q3*gz_rad);
    q1 += 0.5f * CONTROL_DT * (q0*gx_rad + q2*gz_rad - q3*gy_rad);
    q2 += 0.5f * CONTROL_DT * (q0*gy_rad - q1*gz_rad + q3*gx_rad);
    q3 += 0.5f * CONTROL_DT * (q0*gz_rad + q1*gy_rad - q2*gx_rad);

    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm; q1 /= norm; q2 /= norm; q3 /= norm;

    // 解算欧拉角 (度)
    state->roll_deg = atan2f(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2)) * 180.0f / M_PI;
    state->pitch_deg = asinf(2.0f * (q0*q2 - q3*q1)) * 180.0f / M_PI;
    state->yaw_deg = atan2f(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3)) * 180.0f / M_PI;

    // **********************************************
    // 2. 前馈与 PID 控制律计算
    // **********************************************
    float theta = state->roll_deg;
    float raw_omega = gx;  // 横滚角速度

    // EMA滤波
    state->omega_filtered = ALPHA_EMA * raw_omega + (1.0f - ALPHA_EMA) * prev_omega_filtered;
    
    // 计算角加速度 alpha = d(omega)/dt
    state->alpha = (state->omega_filtered - prev_omega_filtered) / CONTROL_DT;
    prev_omega_filtered = state->omega_filtered;

    // 死区补偿：角速度太小则视为0，消除震荡
    float omega = apply_deadzone_smooth(state->omega_filtered, 3.0f, 6.0f);
    
    // 如果角速度被削弱，角加速度按比例削弱
    if (fabsf(state->omega_filtered) > 1e-6f) {
        state->alpha *= fabsf(omega / state->omega_filtered);
    } else {
        state->alpha = 0.0f;
    }

    // [前馈计算]: 抵消重力扰动与自身惯量
    float tau_disturb = SYS_MASS * GRAVITY * (SYS_WIDTH / 2.0f) * sinf(theta * M_PI / 180.0f);
    float tau_self = -K_SELF * theta; // 虚拟刚度防外翻
    state->tau_ff = -tau_disturb - INERTIA * state->alpha - tau_self;
    
    // 前馈激进预测: 如果预测将要偏移，增加打回力矩
    float theta_pred = theta + omega * 0.05f + 0.5f * state->alpha * 0.05f * 0.05f;
    if (fabsf(theta_pred) > 0.8f) {
        state->tau_ff *= 1.2f;
    }

    // [2DOF PID 反馈计算]
    float error = 0.0f - theta; // 目标 0 度
    if (fabsf(theta) < ANGLE_DEADZONE) {
        pid_integral = 0.0f;  // 核心死区清积分防积分饱和
    } else {
        pid_integral += error * CONTROL_DT;
    }
    // 积分上限钳位
    if (pid_integral > 100.0f) pid_integral = 100.0f;
    if (pid_integral < -100.0f) pid_integral = -100.0f;

    float prop = PID_KP * (0.8f * 0.0f - theta); // 2DOF: b=0.8
    float integ = PID_KI * pid_integral;
    float deriv = PID_KD * (0.0f - omega);       // 2DOF: c=0.0
    state->tau_pid = prop + integ + deriv;

    // [总力矩与系统级死区平滑衰减]
    float tau_total = FEEDFORWARD_PARAM * state->tau_ff - FEEDBACK_PARAM * state->tau_pid;

    float angle_factor = apply_deadzone_smooth(theta, ANGLE_DEADZONE, ANGLE_DEADZONE_SOFT) / (fabsf(theta) + 1e-6f);
    if (angle_factor > 1.0f) angle_factor = 1.0f; // 缩放范围 0 ~ 1
    
    tau_total *= angle_factor;
    state->tau_total = tau_total;
}

