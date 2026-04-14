/**
 * @file config.h
 * @brief SteadySail ESP32-S3 核心配置文件
 * 
 * 参考树莓派版本的参数，包含所有控制算法的关键参数
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== I2C 硬件配置 ==================== */
#define I2C_PORT            I2C_NUM_0
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22
#define I2C_FREQ_HZ         400000

/* ==================== IMU I2C 地址 ==================== */
#define IMU1_ADDRESS        0x68        // Main IMU
#define IMU2_ADDRESS        0x69        // Secondary IMU
#define PCA9685_ADDRESS     0x40

/* ==================== 控制周期配置 ==================== */
#define CONTROL_FREQ_HZ     100         // 控制频率 (Hz)
#define CONTROL_DT_MS       10          // 控制周期 (ms)
#define CONTROL_DT_S        0.01f       // 控制周期 (秒)

/* ==================== IMU 轴向配置 ==================== */
// 每个 IMU 的 XYZ 轴是否需要反向
#define IMU1_INVERT_X       1           // 1 = 反向, 0 = 正向
#define IMU1_INVERT_Y       1
#define IMU1_INVERT_Z       0

#define IMU2_INVERT_X       0
#define IMU2_INVERT_Y       0
#define IMU2_INVERT_Z       0

#define CALIBRATION_SAMPLES 500         // 校准采样数

/* ==================== 融合算法参数 ==================== */
#define ALPHA_ACC           0.98f       // 加速度互补滤波系数
#define WEIGHT_DYNAMIC      0.8f        // 动态时的权重
#define BIAS_UPDATE_RATE    0.001f      // 零偏更新速率
#define ALPHA_EMA           0.15f       // EMA 低通滤波系数

/* ==================== 物理参数 ==================== */
#define MASS                80.0f       // 质量 (kg)
#define WIDTH               0.6f        // 船宽 (m)
#define G                   9.81f       // 重力加速度 (m/s^2)

/* ==================== 电机 PWM 参数 ==================== */
#define BASE_PULSE          1500        // 中立脉宽 (μs)
#define MIN_PULSE           1000        // 最小脉宽 (μs)
#define MAX_PULSE           2000        // 最大脉宽 (μs)
#define PWM_FREQ_HZ         50          // PWM 频率 (Hz)
#define THRUST_SCALE        0.55f       // 推力到 PWM 的缩放系数

/* ==================== 电机通道定义 ==================== */
#define LEFT_THRUSTER       0           // 左推进器 (PCA9685 通道)
#define RIGHT_THRUSTER      1           // 右推进器

#define LEFT_ROTATION_CH    2           // 左旋转舵机
#define RIGHT_ROTATION_CH   3           // 右旋转舵机

/* ==================== PID 控制参数 (来自树莓派版本) ==================== */
#define PID_KP              20.0f       // 比例增益
#define PID_KI              1.0f        // 积分增益
#define PID_KD              0.0f        // 微分增益
#define PID_B               0.8f        // 比例权重 (2DOF)
#define PID_C               0.0f        // 微分权重 (2DOF)

/* ==================== 前馈 / 反馈参数 ==================== */
#define FEEDFORWARD_PARAM   0.28f       // 前馈力矩缩放
#define FEEDBACK_PARAM      0.5f        // PID 反馈力矩缩放

/* ==================== 死区参数 ==================== */
#define ANGLE_DEADZONE      1.0f        // 角度死区核心 (度)
#define ANGLE_DEADZONE_SOFT 3.0f        // 角度死区软边界 (度)
#define OMEGA_DEADZONE_SOFT 6.0f        // 角速度死区软边界 (deg/s)

/* ==================== 角度零点校准配置 ==================== */
#define ENABLE_ANGLE_CALIBRATION    1   // 1 = 启用角度校准, 0 = 禁用
#define CALIBRATION_WAIT_TIME       3000 // 等待时间 (ms)，用户需要在这时间内保持垂直
#define CALIBRATION_SAMPLES_COUNT   100  // 校准采样数，用于平均

/* ==================== 调试开关 ==================== */
#define ENABLE_SERVO_ROTATION       1   // 1 = 启用舵机旋转 (UART控制), 0 = 禁用
#define ENABLE_FORCE_ALLOCATION     1   // 1 = 启用力分配 (根据推进器角度调整平衡), 0 = 禁用

/* ==================== 旋转控制参数 ==================== */
#define MAX_ROTATION_ANGLE  45.0f       // 最大旋转角 (度)
#define ROTATION_PULSE_MIN  1000        // 旋转舵机最小脉宽 (μs)
#define ROTATION_PULSE_MAX  2000        // 旋转舵机最大脉宽 (μs)
#define ROTATION_PULSE_CENTER 1500      // 旋转舵机中立脉宽 (μs)
#define ROTATION_RATE_LIMIT 90.0f       // 旋转速率限制 (deg/s)

/* ==================== 数据边界限制 ==================== */
#define ACCEL_MAX           16384.0f    // MPU6050 加速度最大值 (2g 范围)
#define GYRO_MAX            131.0f      // MPU6050 陀螺仪最大值 (250°/s 范围)

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H
