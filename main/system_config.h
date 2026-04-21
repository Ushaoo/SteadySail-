#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// ==========================================
// 1. 系统运行模式开关 (用于分步调试)
// ==========================================
#define MODE_TEST_SENSORS        0  // 纯感知测试：仅打印 IMU 与编码器数据
#define MODE_TEST_STEERING_ONLY  1  // 纯转向测试：仅测试小电机PID，大电机锁定
#define MODE_TEST_BALANCE_ONLY   2  // 纯平衡测试：小电机锁定垂直，仅测试大电机防翻滚
#define MODE_FULL_INTEGRATION    3  // 终极联动：指定推力角 + 主推力补偿
#define MODE_CALIBRATE_ESC       4  // ✨大电机电调校准模式：1000->2000->1000 循环扫描

// 当前激活的模式 (编译前修改这里)
#define CURRENT_RUN_MODE   MODE_CALIBRATE_ESC

// ==========================================
// 2. 硬件引脚分配 (10 个核心 GPIO)
// ==========================================
// --- 双 I2C (MPU6050) ---
#define PIN_I2C0_SDA    8   // IMU 1
#define PIN_I2C0_SCL    9
#define PIN_I2C1_SDA    10  // IMU 2
#define PIN_I2C1_SCL    11

// --- 转向编码器输入 (MT6826S 脉宽捕获) ---
#define PIN_ENC_LEFT    4
#define PIN_ENC_RIGHT   5

// --- 转向小电机 PWM 输出 ---
#define PIN_STEER_LEFT  1
#define PIN_STEER_RIGHT 2

// --- 主推进器 (电调 ESC) PWM 输出 ---
// ESP32-S3 LEDC LOW_SPEED 支持的GPIO: 0-19, 21-25, 26-33
#define PIN_THRUST_LEFT  18
#define PIN_THRUST_RIGHT 19

// ==========================================
// 3. 安全保护机制开关
// ==========================================
#define ENABLE_I2C_RECOVERY    1    // 开启 I2C 错误重试与自救
#define ENABLE_EMERGENCY_STOP  1    // 开启倾角过大断电保护 (>60度)
#define I2C_RETRY_COUNT        3    // I2C 失败重试次数

// ==========================================
// 4. 物理与控制参数 (原 Python 映射)
// ==========================================
#define CONTROL_DT          0.01f   // 主控制循环时间 (s) -> 100Hz
#define SYS_MASS            80.0f   // kg
#define SYS_WIDTH           0.6f    // m
#define GRAVITY             9.81f   // m/s^2

#define PID_KP              20.0f
#define PID_KI              1.0f
#define PID_KD              0.0f
#define FEEDFORWARD_PARAM   0.28f
#define FEEDBACK_PARAM      0.5f
#define ANGLE_DEADZONE      1.0f
#define ANGLE_DEADZONE_SOFT 3.0f

#endif // SYSTEM_CONFIG_H
