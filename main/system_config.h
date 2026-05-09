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
#define MODE_TEST_IMU_ONLY       5  // 🎯IMU 数据专用测试：实时打印原始和融合数据

// 当前激活的模式 (编译前修改这里)
#define CURRENT_RUN_MODE   MODE_FULL_INTEGRATION

// ==========================================
// 1.1 FULL_INTEGRATION 演示开关
//   1 = 手动模拟 Roll 角（忽略 IMU，仅 MODE_FULL_INTEGRATION 生效）
//       串口输入 `r <角度>` 设置模拟 Roll，舱机会按该角度动作
//       供展会演示舱机转动逻辑使用（设备不在水中，IMU不动也能看到舵机响应）
//   0 = 正常使用 IMU 读数
// ==========================================
#define DEMO_MANUAL_ROLL         0
// ==========================================
// 1.5. 转向控制模式开关
// ==========================================
#define STEERING_MODE_PID        0  // 使用 PID 控制（精确但容易振荡）
#define STEERING_MODE_DIRECT     1  // 直接映射模式（简单，误差直接转 PWM）

// 当前转向模式 (改这里切换)
#define STEERING_CONTROL_MODE    STEERING_MODE_PID

// ==========================================
// 2. 硬件引脚分配 (10 个核心 GPIO)
// ==========================================
// --- 双 I2C (MPU6050) ---
#define PIN_I2C0_SDA    8   // IMU 1
#define PIN_I2C0_SCL    9
#define PIN_I2C1_SDA    10  // IMU 2
#define PIN_I2C1_SCL    11

// --- 转向编码器输入 (MT6826S 脉宽捕获) ---
#define PIN_ENC_LEFT    5
#define PIN_ENC_RIGHT   4

// --- 转向小电机 PWM 输出 ---
#define PIN_STEER_LEFT  2
#define PIN_STEER_RIGHT 1

// --- 主推进器 (电调 ESC) PWM 输出 ---
// ESP32-S3 LEDC LOW_SPEED 支持的GPIO: 0-19, 21-25, 26-33
#define PIN_THRUST_LEFT  18
#define PIN_THRUST_RIGHT 19

// --- RC 遥感 PWM 输入 (油门通道) ---
// 标准 RC PWM: 1000~2000μs, 1500μs=中位; 接收机信号线 → GPIO 7
#define PIN_RC_THROTTLE  7


// ==========================================
// 3. IMU 配置开关 (单/双 IMU 切换)
// ==========================================
#define USE_DUAL_IMU           0    // 0=单IMU测试版本, 1=双IMU融合版本
                                     // 单IMU模式使用 I2C0 (GPIO 8/9)
                                     // 后续购买支持改地址的IMU时改为1

// 0 = 使用 MPU6050 + 软件 Mahony 融合（遗留备用）
// 1 = 使用 BNO055 片上 NDOF 融合输出 Roll 角（无软件融合负担）
//     BNO055 接在 I2C0（GPIO 8/9），考接 MPU6050 原接口；不再调用 imu_driver_init()
#define USE_BNO055_FOR_ROLL    1

// ==========================================
// 4. 安全保护机制开关
// ==========================================
#define ENABLE_I2C_RECOVERY    1    // 开启 I2C 错误重试与自救
#define ENABLE_EMERGENCY_STOP  1    // 开启倾角过大断电保护 (>60度)
#define I2C_RETRY_COUNT        3    // I2C 失败重试次数

// ------------------------------------------
// ⚠️ 调试限幅：PWM 输出范围（中立 1500 us，改这里可以全局限制推力/转角）
//   - 调试期建议取小，避免涨车/损坏舵机
//   - 所有 motor_control_* 下发函数都会被强制 clamp 到该范围
// ------------------------------------------
// 主推进器（大电机 / ESC）调试限幅：1500 ± 偏移
#define THRUST_PWM_MIN_US      1200    // 默认 1500 - 50
#define THRUST_PWM_MAX_US      1800    // 默认 1500 + 50

// 转向舵机（小电机）调试限幅：1500 ± 偏移
//   舵机机械范围一般是 1000~2000；调试期可设小一点限位
#define STEER_PWM_MIN_US       1000    // 默认 1500 - 500
#define STEER_PWM_MAX_US       2000    // 默认 1500 + 500

// ------------------------------------------
// ⚙️ 方向反转开关（硬件装好后，按观察到的现象切 0/1，无需改控制逻辑）
//
//   验证方法（每个独立测试，失败就把对应开关改 1）：
//     1. THRUST_*_INVERT     -> 给正向推力，桨片应"向后吹水让船前进"。若反了 -> 设 1
//     2. STEER_*_INVERT      -> set_target(170°) 后，编码器读数应往 170 靠。若往 190 走 -> 设 1
//                               （即 PWM 增加方向与角度增加方向相反）
//     3. ENC_*_REVERSE       -> 手动转动舵机，编码器应该数值递增；若递减 -> 设 1
//
//   说明：
//     - 推力反转：在 motor_control_set_pwm_bidirectional 内做，等价于
//       把 invert_L/R 再异或一次
//     - 舵机反转：把舵机 PWM 围绕 1500 镜像（pwm = 3000 - pwm）
//     - 编码器反转：在 steering_control 里读取角度后做 angle = 360 - angle
// ------------------------------------------
#define THRUST_LEFT_INVERT     0    // 0=正常, 1=反转左推进器输出方向
#define THRUST_RIGHT_INVERT    0    // 0=正常, 1=反转右推进器输出方向

#define STEER_LEFT_INVERT      1    // 0=正常, 1=反转左舵机 PWM 方向
#define STEER_RIGHT_INVERT     0    // 0=正常, 1=反转右舵机 PWM 方向

#define ENC_LEFT_REVERSE       0    // 0=正常, 1=反转左编码器读数0

#define ENC_RIGHT_REVERSE      1    // 0=正常, 1=反转右编码器读数

// ==========================================
// 4.5 差速转向参数（仅在 g_forward_thrust > TURN_MIN_FWD_PCT 时启用）
//   原理：左右水平分量 H 取不等值 → 推力大的一侧把船头推向反向
//     H_L = H + DELTA_H/2 ,  H_R = H - DELTA_H/2  → 船头向右偏（右转）
//     H_L = H - DELTA_H/2 ,  H_R = H + DELTA_H/2  → 船头向左偏（左转）
//   通过 Blinker 三个按钮（左 / 前 / 右）控制 g_turn_state ∈ {-1, 0, +1}。
//   "前"按钮 = 取消转向，恢复 H_L = H_R 直行。
// ==========================================
#define TURN_DELTA_H        100.0f   // 转向时左右水平分量差（推力单位，量程 0~500，50 ≈ 10%）
#define TURN_MIN_FWD_PCT    5.0f    // 仅当 |g_forward_thrust| > 此值时差速生效（避免低速误转）

// ==========================================
// 4.6 航向保持 PID 参数（BNO055 绝对偏航角 → 差速修正）
//   turn_f 按钮触发：锁定当前 BNO055 偏航角，持续将偏差转化为左右差速 dH。
//   dH = Kp * yaw_err + Ki * integral
//   - Kp 越大，小偏差时响应越快，但过大会振荡（可从 0.5 开始调）
//   - Ki 消除稳态偏差（如长期受单侧风/流影响），建议从 0.05 开始
//   - 输出饱和限幅 = TURN_DELTA_H（与手动差速共用上限）
// ==========================================
#define HEADING_KP          0.8f    // 航向保持 P 增益 (dH/°)
#define HEADING_KI          0.05f   // 航向保持 I 增益 (dH/(°·s))

// ==========================================
// 5. 物理与控制参数 (原 Python 映射)
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
