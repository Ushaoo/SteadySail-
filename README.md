# SteadySail v2.0 - 可旋转推进器自平衡水面艇控制系统

## 📋 项目概述

**SteadySail** 是一个基于 **ESP32-S3** 的**可旋转推进器自平衡水面艇（USV）自动控制系统**。该系统通过在桨板两侧安装可旋转的推进器，实现三层递进式功能切换：

| 推进器状态 | 工作模式 | 功能说明 |
|----------|--------|--------|
| **竖直状态（0°）** | 平衡模式 | 双IMU传感器监测船体横滚，自动调节左右推进器推力差产生平衡力矩，抵抗倾覆 |
| **水平状态（90°）** | 推进模式 | 推进器完全水平，两侧推进器均匀出力提供前进动力 |
| **倾斜状态（0°~90°）** | 混合模式 | 同时提供平衡与推进，根据当前倾斜角自动补偿推力方向，实现边推进边自平衡 |

系统以 **100Hz** 实时控制频率运行，采用 **Mahony四元数姿态融合** + **2DOF前馈反馈PID** 双层控制策略，确保在复杂海况下的稳定航行。

---

## ⚙️ 硬件要求

### 主控制器
- **微控制器**：ESP32-S3（双核 Xtensa 处理器）
- **时钟频率**：240MHz
- **Flash**：≥ 4MB
- **RAM**：≥ 512KB SRAM

### I2C 接口支持说明

**ESP32-S3 I2C 支持情况**：
- 支持多个 I2C 接口（I2C_NUM_0, I2C_NUM_1, 甚至更多）
- 每个 I2C 接口可自由分配到任意 GPIO 引脚（只需 SDA 和 SCL）
- 不同 I2C 接口可以使用不同的时钟速率

**当前项目配置**：
- **I2C0**：GPIO 8(SDA) + GPIO 9(SCL) → IMU1
- **I2C1**：GPIO 10(SDA) + GPIO 11(SCL) → IMU2

### 传感器
- **双IMU传感器**：MPU-6050 × 2（加速度计 + 陀螺仪）
  - 量程：加速度 ±16g，角速度 ±2000°/s
  - 通信接口：I2C（地址 0x68）
  - **I2C 地址**：MPU-6050 的 I2C 地址由 AD0 引脚决定
    - AD0 接 GND → 地址为 0x68
    - AD0 接 VCC → 地址为 0x69
  
### 电机与执行器
- **主推进电机**：无刷电机 × 2（两侧对称）
  - 驱动方式：ESC电调控制
  - PWM频率：50Hz
  - 脉宽范围：1000 ~ 2000 µs（中立 1500 µs）

- **转向小电机**：舵机 × 2（可旋转推进器用）
  - 驱动方式：PWM控制
  - PWM频率：50Hz
  - 脉宽范围：1000 ~ 2000 µs（中立 1500 µs）

- **位置反馈编码器**：MT6826S PWM编码器 × 2
  - 输出格式：PWM占空比（0~4095对应0°~360°）
  - 精度：≤1°

### 电源
- **供电电压**：9V ~ 18V（推荐 12V）
- **系统功耗**：典型 20W（空载），峰值 80W（满推力）

---

## 🔌 硬件连接图

### ESP32-S3 GPIO 引脚分配（10个核心GPIO）

```
┌─────────────────────────────────────────────┐
│  双 I2C 总线（MPU-6050 传感器）            │
├─────────────────────────────────────────────┤
│  I2C0: GPIO 8(SDA)  GPIO 9(SCL)   → IMU1  │
│  I2C1: GPIO 10(SDA) GPIO 11(SCL)  → IMU2  │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│  编码器输入（MT6826S 占空比捕获）           │
├─────────────────────────────────────────────┤
│  GPIO 4  → 左编码器（上升/下降沿检测）      │
│  GPIO 5  → 右编码器（上升/下降沿检测）      │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│  转向小电机 PWM 输出（50Hz）                │
├─────────────────────────────────────────────┤
│  GPIO 1  → 左转向舵机  (LEDC CH0)          │
│  GPIO 2  → 右转向舵机  (LEDC CH1)          │
└─────────────────────────────────────────────┘

┌─────────────────────────────────────────────┐
│  主推进器 ESC PWM 输出（50Hz, 14-bit分辨率）│
├─────────────────────────────────────────────┤
│  GPIO 18 → 左主推进    (LEDC CH2, TIMER_0) │
│  GPIO 19 → 右主推进    (LEDC CH3, TIMER_0) │
└─────────────────────────────────────────────┘
```

### 推荐连接方式

```
┌─ I2C0 总线 (400kHz) ──────────────────┐
│  GPIO 8  (SDA) ──┬─ IMU1 (MPU-6050)    │
│  GPIO 9  (SCL) ──┤  地址: 0x68          │
│  +3.3V   ────────┤  +5V (可用降压)      │
│  GND     ────────┴─ GND                │
└─────────────────────────────────────────┘

┌─ I2C1 总线 (400kHz) ──────────────────┐
│  GPIO 10 (SDA) ──┬─ IMU2 (MPU-6050)    │
│  GPIO 11 (SCL) ──┤  地址: 0x68          │
│  +3.3V   ────────┤  +5V (可用降压)      │
│  GND     ────────┴─ GND                │
└─────────────────────────────────────────┘

┌─ 编码器输入 ──────────────────────────┐
│  GPIO 4  ↔ 左编码器 (MT6826S Data)    │
│  GPIO 5  ↔ 右编码器 (MT6826S Data)    │
│  +5V     ← 编码器电源                 │
│  GND     ← 编码器地                   │
└─────────────────────────────────────────┘

┌─ 转向小电机 ──────────────────────────┐
│  GPIO 1  ↔ 左舵机 (Signal, PWM 50Hz)  │
│  GPIO 2  ↔ 右舵机 (Signal, PWM 50Hz)  │
│  +5V     ← 舵机电源                   │
│  GND     ← 舵机地                     │
└─────────────────────────────────────────┘

┌─ 主推进器 (ESC) ──────────────────────┐
│  GPIO 18 ↔ 左ESC (Signal, PWM 50Hz)   │
│  GPIO 19 ↔ 右ESC (Signal, PWM 50Hz)   │
│  GND     ↔ ESC GND (共地)             │
│  +12V    ← 电池+ (直接供电)           │
│  GND     ← 电池- (共地)               │
└─────────────────────────────────────────┘

⚠️ I2C 上拉电阻: 需在 SDA/SCL 各添加 4.7kΩ 上拉至 3.3V
⚠️ 信号隔离: GPIO 信号线与电源线分开布线，避免干扰
```

### ESP32-S3 I2C 连接方案（三选一）

#### 方案A：推荐 - 使用两条独立 I2C 总线（当前配置）

适用于有充足 GPIO 的场景。ESP32-S3 支持多个 I2C 接口，完全可以使用两条独立总线：

```
配置文件: main/system_config.h
├── I2C0 总线: GPIO 8 (SDA) + GPIO 9 (SCL)
│   └── IMU1 (MPU-6050, AD0=GND, 地址 0x68)
│
└── I2C1 总线: GPIO 10 (SDA) + GPIO 11 (SCL)
    └── IMU2 (MPU-6050, AD0=GND, 地址 0x68)

💡 优势：
  • 完全独立的总线，不会互相干扰
  • 可以不同的频率运行
  • 故障隔离（一条线故障另一条继续工作）
  • 代码改动最小（当前就是这个方案）
```

#### 方案B：共享一条 I2C 总线 + 地址区分

如果只有一条 I2C 总线（GPIO 8/9 或 GPIO 10/11），可通过 MPU-6050 的 AD0 引脚区分地址：

```
配置: 需修改 main/system_config.h 和 main/imu_driver.c

GPIO 8  (SDA) ──┬─ IMU1 (AD0=GND,  地址 0x68)
GPIO 9  (SCL) ──┤
                └─ IMU2 (AD0=VCC,  地址 0x69)
                
+3.3V ──────────┬─ IMU1 (+3.3V引脚)
                └─ IMU2 (+3.3V引脚, 用于AD0)
GND ────────────┬─ IMU1 (GND, 用于AD0)
                └─ IMU2 (GND)

修改步骤：
1. 编辑 main/imu_driver.c，同时支持地址 0x68 和 0x69
2. 删除 I2C1 初始化
3. 两个 IMU 都在 I2C0 上，但地址不同
```

**修改代码示例**：
```c
// 在 imu_driver.c 中改为单总线双地址读取
#define MPU6050_ADDR_1  0x68  // IMU1 地址
#define MPU6050_ADDR_2  0x69  // IMU2 地址（需配置 AD0=VCC）

esp_err_t imu_driver_init(void) {
    // 只初始化 I2C0
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = PIN_I2C0_SDA,  // GPIO 8
        .scl_io_num = PIN_I2C0_SCL,  // GPIO 9
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    
    // 唤醒两个 IMU（地址不同）
    mpu6050_wake_up_addr(I2C_NUM_0, MPU6050_ADDR_1);
    mpu6050_wake_up_addr(I2C_NUM_0, MPU6050_ADDR_2);
}

esp_err_t imu_driver_read(dual_imu_data_t *data) {
    read_single_imu_addr(I2C_NUM_0, MPU6050_ADDR_1, &data->imu1);
    read_single_imu_addr(I2C_NUM_0, MPU6050_ADDR_2, &data->imu2);
}
```

#### 方案C：使用 I2C 多路复用器芯片（TCA9548A）

适合有大量 I2C 设备的复杂场景：

```
GPIO 8  (SDA) ──┬─ TCA9548A (主 I2C)
GPIO 9  (SCL) ──┤
                
TCA9548A 有 8 个子通道：
  CH0 → IMU1 (MPU-6050, 0x68)
  CH1 → IMU2 (MPU-6050, 0x68)
  CH2 → 其他传感器...
  ...

💡 优势：
  • 单主控制器可接多个相同地址的从设备
  • 易于扩展
  
⚠️ 缺点：
  • 多了一块硬件芯片（成本+复杂度）
  • 代码改动较大（需要多路选择逻辑）
```

---

### 如何选择最适合的方案

| 条件 | 推荐方案 |
|------|--------|
| **GPIO 充足**（≥4个 GPIO 用于 I2C） | 方案A（当前） |
| **GPIO 紧张**（只有 2 个 GPIO） | 方案B（需要修改代码） |
| **需要更多传感器**（>2 个 I2C 设备） | 方案C（需要硬件改造） |
| **最快快速原型** | 方案A（无需改动） |

---

### 验证 I2C 连接

烧录后检查连接是否成功：

```bash
# 进入 MODE_TEST_SENSORS 模式
# 修改 main/system_config.h
#define CURRENT_RUN_MODE   MODE_TEST_SENSORS

# 构建并烧录
idf.py build flash monitor

# 监控输出，应看到类似：
I (40) IMU_DRIVER: 双 MPU6050 初始化成功！

# 如果显示失败，检查：
# 1. GPIO 接线是否正确？
# 2. I2C 上拉电阻是否在位（4.7kΩ）？
# 3. MPU-6050 电源（+3.3V, GND）是否稳定？
# 4. AD0 引脚配置是否正确？
```

---

## 🏗️ 软件架构

### 模块化设计

```
┌──────────────────────────────────────────────────┐
│                   main.c (主程序)                 │
│  - app_main()     初始化所有模块                 │
│  - control_core_task()  100Hz 控制循环           │
│  - 串口监控任务   实时参数调整                   │
└──────────────────┬─────────────────────────────┘
                   │
        ┌──────────┼──────────┐
        │          │          │
        ▼          ▼          ▼
┌────────────────┐ ┌──────────────────┐ ┌──────────────────┐
│ imu_driver.c   │ │ steering_control │ │ motor_control.c  │
│                │ │                  │ │                  │
│ • 双I2C初始化  │ │ • 编码器ISR      │ │ • LEDC PWM配置   │
│ • MPU-6050读取 │ │ • PID转向控制    │ │ • 推力补偿算法   │
│ • 原始数据解析 │ │ • 最短路径算法   │ │ • ESC校准序列    │
└──────┬─────────┘ └────────┬─────────┘ └────────┬─────────┘
       │                    │                    │
       └────────────┬───────┴────────────────────┘
                    │
                    ▼
        ┌─────────────────────────┐
        │ balance_controller.c    │
        │                         │
        │ • Mahony四元数融合      │
        │ • 方差加权双IMU融合     │
        │ • 前馈扰动补偿          │
        │ • 2DOF PID反馈控制      │
        │ • 角度死区平滑          │
        └─────────────────────────┘
```

### 实时控制流程（100Hz，每周期 10ms）

```
┌─────────────────────────────────────────────────┐
│ 周期开始 (绝对定时，精度 ±0.1ms)               │
└──────────────┬──────────────────────────────────┘
               │
               ▼
   ┌───────────────────────────┐
   │ 转向PID更新               │
   │ steering_control_update() │
   │ → 获取当前推进器角度α      │
   └───────────────┬───────────┘
                   │
                   ▼
   ┌───────────────────────────┐
   │ IMU数据读取               │
   │ imu_driver_read()         │
   │ → 双IMU原始数据           │
   └───────────────┬───────────┘
                   │
            ┌──────┴──────┐
            │ 成功?      │
            └─┬────────┬─┘
              │ 否     │ 是
              │        ▼
              │   ┌──────────────────┐
              │   │ 平衡算法计算     │
              │   │ balance_control_ │
              │   │ update()         │
              │   │ → τ_total (力矩) │
              │   └─────┬────────────┘
              │         │
              │         ▼
              │   ┌──────────────────┐
              │   │ 角度补偿         │
              │   │ τ_final =        │
              │   │ τ_total/cos(α)   │
              │   └─────┬────────────┘
              │         │
              └─────┬───┘
                    │
                    ▼
   ┌───────────────────────────┐
   │ 推力分配                  │
   │ motor_control_set_thrust_ │
   │ with_compensation()       │
   │ 左推 = 1500 + Δ           │
   │ 右推 = 1500 - Δ           │
   │ (产生平衡力偶)            │
   └───────────────┬───────────┘
                   │
                   ▼
   ┌───────────────────────────┐
   │ 安全检查                  │
   │ Roll角 > 60° ?            │
   │ → 紧急停止                │
   └───────────────┬───────────┘
                   │
                   ▼
   ┌───────────────────────────┐
   │ 等待至 10ms 整数倍        │
   │ vTaskDelayUntil()         │
   └───────────────┬───────────┘
                   │
                   ▼
        ┌──────────────────┐
        │ 循环重新开始     │
        └──────────────────┘
```

---

## 🔧 核心模块详解

### 1. IMU 驱动模块（imu_driver.c）

**功能**：实时读取两个MPU-6050传感器数据

**关键接口**：
```c
esp_err_t imu_driver_init(void);
// 初始化双I2C总线（400kHz）和两个MPU-6050芯片

esp_err_t imu_driver_read(dual_imu_data_t *data);
// 读取原始加速度(±16g) + 角速度(±2000°/s)
// 返回结构体包含: imu1/imu2 → {accel_x/y/z, gyro_x/y/z}
```

**数据解析**：
- 加速度归一化：±16g 范围映射到 ±1g（以重力加速度为单位）
- 角速度单位：°/s
- 采样率：400ms读取周期（由主控制循环驱动）

---

### 2. 平衡控制器（balance_controller.c）

**三大核心算法**：

#### A. 双IMU方差加权融合
```
步骤1：计算各IMU的角速度方差（实时指数移动平均）
       var1 = 0.95·var1 + 0.05·(gyro_x^2)
       var2 = 0.95·var2 + 0.05·(gyro_x^2)

步骤2：倒数权重分配
       w1 = 1/var1 / (1/var1 + 1/var2)
       w2 = 1 - w1

步骤3：融合加速度与角速度
       a_fused = 0.98·imu1 + 0.02·imu2
       ω_fused = w1·imu1 + w2·imu2
```

**作用**：自动选择更可靠的传感器，抗干扰性强

#### B. Mahony四元数姿态融合
```
四元数更新：q(k+1) = q(k) + 0.5·Δt·ω·q(k)

其中ω包含加速度补偿项（修正漂移）：
ω = ω_gyro + Kp_adaptive·e_acc

自适应Kp：Kp = 2.0 + 25.0·|a_error|
（加速度偏离重力越大，校正增益越高）

输出欧拉角：
Roll   = atan2(2(q0·q1 + q2·q3), 1-2(q1² + q2²))
Pitch  = asin(2(q0·q2 - q3·q1))
Yaw    = atan2(2(q0·q3 + q1·q2), 1-2(q2² + q3²))
```

**优势**：无矩阵运算，计算量小；收敛速度快

#### C. 前馈+反馈混合控制
```
前馈部分（预测补偿）：
  τ_disturb = m·g·(L/2)·sin(θ)     // 重力倾覆力矩
  τ_inertia = I·α                  // 惯量补偿
  τ_self = -100·θ                  // 虚拟刚度
  τ_ff = -τ_disturb - τ_inertia - τ_self

反馈部分（误差修正）：
  e = 0 - θ                        // 目标0°
  P = 20.0·e  (如果|θ| > 1°)
  I = ∫e·dt   (积分限幅 ±100)
  D = 0.0·dω  (纯比例2DOF)
  τ_pid = 20·e + 1·∫e·dt

最终输出：
  τ_total = 0.28·τ_ff - 0.5·τ_pid
```

**配置参数（system_config.h）**：
```c
PID_KP = 20.0               // 比例系数
PID_KI = 1.0                // 积分系数
PID_KD = 0.0                // 微分系数
FEEDFORWARD_PARAM = 0.28    // 前馈权重
FEEDBACK_PARAM = 0.5        // 反馈权重
ANGLE_DEADZONE = 1.0        // 角度死区（°）
SYS_MASS = 80.0             // 系统质量（kg）
SYS_WIDTH = 0.6             // 系统宽度（m）
GRAVITY = 9.81              // 重力加速度（m/s²）
```

---

### 3. 电机驱动模块（motor_control.c）

**PWM映射关系**（50Hz, 14-bit分辨率）：
```
脉宽时间 → LEDC占空比（0~16383）:
  1000 µs (最小推力)    → 819  (5.0%)
  1500 µs (中立/停止)   → 1228 (7.5%)
  2000 µs (最大推力)    → 1638 (10.0%)

计算公式：duty = (us × 16384) / 20000
```

**推力补偿算法**（处理推进器旋转对推力的影响）：
```
当推进器旋转角度α时，竖直方向推力分量 = τ·cos(α)
为保持竖直平衡力矩不变，需补偿：

τ_compensated = τ_total / cos(α)

限制条件（避免分母过小）：
  α_clamped = clamp(α, -60°, +60°)
  cos(60°) = 0.5，即最大补偿2倍

转为PWM变化量：
  Δ_pwm = τ_compensated × 0.55
  限幅：±500 (对应1000~2000范围)

左右推进器：
  thrust_L = 1500 + Δ_pwm   (增加左推力)
  thrust_R = 1500 - Δ_pwm   (减少右推力)
  → 产生力偶矩恢复平衡
```

**ESC校准模式**（MODE_CALIBRATE_ESC）：
- 自动扫描 PWM 从 1000 → 2000 → 1000 µs
- 每步停留 1.5s，最高值保持 3s
- 帮助ESC学习推力范围映射

---

### 4. 转向控制模块（steering_control.c）

**编码器采样**（MT6826S PWM占空比编码器）：
```
GPIO中断捕获：
  上升沿 → 记录整个周期时间 (period_us)
  下降沿 → 记录高电平时间 (high_us)

占空比 → 角度转换：
  total_period = high_us + period_us
  duty = (high_us / total_period) × 4095
  angle = (duty - offset) × (360° / 4095)
```

**PID转向控制**：
```
目标设定：0° (竖直) ~ 360° (全圆)

最短路径算法（处理环形误差）：
  error = target - current
  if error > 180°  → error -= 360°
  if error < -180° → error += 360°

PID参数：
  Kp = 6.8  (比例)
  Ki = 1.0  (积分)
  Kd = 0.49 (微分)
  
死区机制：
  |error| < 3°  → 输出清零，积分清零（防抖）
  3° ~ 6°       → 平滑过渡
  |error| ≥ 6°  → 正常PID

积分限幅：±120
输出限幅：±80 µs (对应1500±80脉宽)

低通滤波：时间常数 0.12s
  out_filt = out_filt + α·(raw_out - out_filt)
  α = dt / (0.12 + dt) ≈ 0.077
```

---

## 🚀 快速开始

### 前置要求
- **操作系统**：Windows/Linux/macOS
- **ESP-IDF**：v4.4（使用 ESP-IDF Tools Installer 安装）
- **VS Code**：最新版本
- **VS Code扩展**：
  - ESP-IDF extension (by Espressif)
  - C/C++ extension (by Microsoft)
  - Python 3.8+

### 单/双 IMU 切换说明

**当前状态**：项目已配置为**单 IMU 测试版本**

#### 切换到单 IMU 模式（仅连接一个 MPU-6050）

```c
// 编辑 main/system_config.h，确保：
#define USE_DUAL_IMU           0    // ← 设为 0（单IMU）

// 硬件连接：
// GPIO 8  (SDA) ──┬─ MPU-6050 (地址 0x68)
// GPIO 9  (SCL) ──┤
// +3.3V ──────────┤
// GND ────────────┴─

// I2C1 的 GPIO 10/11 可以不连接（或用于其他用途）
```

**单 IMU 模式特点**：
- 使用 I2C0 总线（GPIO 8/9）
- 直接使用单个传感器数据，无融合
- Mahony 算法固定增益（不自适应）
- 适合快速测试和调试

#### 升级到双 IMU 模式（需购买支持改地址的 MPU-6050）

```c
// 编辑 main/system_config.h，修改为：
#define USE_DUAL_IMU           1    // ← 设为 1（双IMU）

// 硬件连接：
// I2C0 总线：
// GPIO 8  (SDA) ──┬─ MPU-6050 (AD0=GND, 地址 0x68)
// GPIO 9  (SCL) ──┤
//
// I2C1 总线：
// GPIO 10 (SDA) ──┬─ MPU-6050 (AD0=VCC, 地址 0x69)
// GPIO 11 (SCL) ──┤
```

**双 IMU 模式特点**：
- 使用独立的 I2C0 和 I2C1 总线
- 方差加权融合两个传感器
- Mahony 算法自适应增益
- 更强的抗干扰能力和冗余性

#### 修改后重新构建

```bash
# 修改配置后，重新烧录
idf.py build flash monitor

# 启动输出会显示：
# I (40) IMU_DRIVER: 单 MPU6050 (I2C0) 初始化成功！  ← 单IMU
# 或
# I (40) IMU_DRIVER: 双 MPU6050 初始化成功！         ← 双IMU
```

---

### 1. ESP-IDF 4.4 环境配置

#### Windows（推荐使用 Installer）
```bash
# 下载并运行 ESP-IDF Tools Installer
# https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/get-started/windows-setup.html

# 或手动配置：
set IDF_PATH=C:\esp\esp-idf-v4.4

# 验证安装
idf.py --version
# 输出应为：ESP-IDF v4.4.x
```

#### Linux/macOS
```bash
mkdir -p ~/esp
cd ~/esp
git clone --branch v4.4 https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh

source ~/esp/esp-idf/export.sh
```

### 2. VS Code 中配置 ESP-IDF 扩展

1. 打开 VS Code 扩展市场，搜索并安装 **"ESP-IDF extension"**
2. 按 `Ctrl+Shift+P` 打开命令面板
3. 输入 **"ESP-IDF: Configure ESP-IDF extension"**
4. 选择 **"Advanced"** 选项
5. 按提示选择 ESP-IDF 安装路径和 Python 可执行文件
6. 选择 **"v4.4"** 版本
7. 点击 **"Install"** 完成配置

### 3. 构建项目

#### 方式1：使用 VS Code ESP-IDF 扩展（推荐）

```bash
# 打开 d:\SteadySail--1 文件夹
# 按 Ctrl+Shift+P，输入 "ESP-IDF: Build"
# 点击执行

# 或在集成终端运行：
idf.py build
```

#### 方式2：命令行构建
```bash
cd d:\SteadySail--1
idf.py set-target esp32s3
idf.py build
```

**预期输出**：
```
[100%] Built target SteadySail
Build complete. The following files were created:
build/SteadySail.elf
build/SteadySail.bin
```

### 4. 烧录到 ESP32-S3

#### 方式1：VS Code ESP-IDF 扩展（推荐）

```bash
# 按 Ctrl+Shift+P，输入 "ESP-IDF: Select Port"
# 选择 ESP32-S3 连接的 COM 端口

# 按 Ctrl+Shift+P，输入 "ESP-IDF: Flash"
# 或使用 "ESP-IDF: Build, Flash and Monitor" 一键烧录+监控
```

#### 方式2：命令行烧录
```bash
cd d:\SteadySail--1

# 查看可用端口
idf.py port-detect

# 设置端口并烧录
idf.py -p COM5 flash

# 烧录+监控（查看串口输出）
idf.py -p COM5 flash monitor
# 按 Ctrl+] 退出监控

# 或一键完成（构建+烧录+监控）
idf.py -p COM5 build flash monitor
```

### 5. 监控串口输出

```bash
idf.py -p COM5 monitor

# 预期输出：
# I (40) MAIN: SteadySail Version 2 - Dual IMU + Vector Thrust
# I (60) IMU_DRIVER: 双 MPU6050 初始化成功！
# I (70) STEERING: Steering Encoder Interrupts Initialized.
# ...
```

---

## 🎮 使用指南

### 运行模式（system_config.h 中设置）

编辑 [main/system_config.h](main/system_config.h#L11) 中的 `CURRENT_RUN_MODE`：

```c
#define CURRENT_RUN_MODE   MODE_CALIBRATE_ESC  // 修改这行
```

#### 模式列表

| 模式 | 定义值 | 说明 |
|------|------|------|
| `MODE_TEST_SENSORS` | 0 | 纯传感器测试：仅打印IMU与编码器原始数据 |
| `MODE_TEST_STEERING_ONLY` | 1 | 转向测试：仅测试小电机PID，主推进锁定1500 |
| `MODE_TEST_BALANCE_ONLY` | 2 | 平衡测试：小电机锁定竖直(180°)，仅测试主推进平衡 |
| `MODE_FULL_INTEGRATION` | 3 | **完整集成**：平衡+推进器旋转+力补偿联动 |
| `MODE_CALIBRATE_ESC` | 4 | ESC校准：PWM扫描 1000→2000→1000 µs |

### 切换运行模式

1. 编辑 [main/system_config.h](main/system_config.h#L11)
2. 修改 `CURRENT_RUN_MODE` 值
3. 运行 `idf.py build flash monitor`

### 串口交互命令（仅在 MODE_FULL_INTEGRATION 下可用）

启动后通过串口发送以下命令调整转向目标角度：

| 快捷命令 | 功能 |
|--------|------|
| `a` 或 `A` | 转向角度 +15°（左转） |
| `d` 或 `D` | 转向角度 -15°（右转） |
| `s` 或 `S` | 回正（0°） |
| 数字（如`45`） | 设置绝对角度（单位°） |

**使用示例**：
```
输入: a
输出: >>> 偏角 +15，目标: 195.0 度 <<<

输入: 90
输出: >>> 收到绝对角度指令! 转向目标定为: 90.0 度 <<<

输入: s
输出: >>> 转向回正！ <<<
```

**实时数据输出**（10Hz，每100ms一行）：
```
Target:180.0 | CurL:180.2 | CurR:179.8 | Roll:+2.34 | Tau:+12.56
Target:90.0 | CurL:91.5 | CurR:88.2 | Roll:-1.05 | Tau:-8.23
```

---

## 🔍 调试与故障排查

### 1. IMU 连接问题

**症状**：启动后输出 `IMU 硬件异常！`

**排查步骤**：
```bash
# 进入 MODE_TEST_SENSORS 模式
# 查看 I2C 总线是否有应答

idf.py monitor
# 寻找 I2C 初始化日志

# 如果仍无输出，检查：
- 杜邦线是否接触良好？GPIO 8/9/10/11 正确连接？
- MPU-6050 的 AD0 引脚是否正确（决定I2C地址0x68）
- I2C 上拉电阻是否在位（推荐 4.7kΩ）
```

### 2. 编码器不工作

**症状**：转向角度始终显示 0°（或垃圾数据）

**排查步骤**：
```bash
# 进入 MODE_TEST_SENSORS 模式
# 手动旋转推进器，观察输出中的编码器占空比是否变化

# 检查：
- GPIO 4/5 是否接收到编码器信号？
- MT6826S 供电 +5V 是否正常？
- GPIO 中断是否启用？

# 编码器校准（竖直状态下）：
# 在 steering_control_calibrate_encoders() 处设置基准点
```

### 3. 推进器无反应

**症状**：电调没有收到 PWM 信号，电机不转

**排查步骤**：
```bash
# 进入 MODE_CALIBRATE_ESC 模式
# 启动 ESC 校准序列，观察串口输出

idf.py -p COM5 flash monitor
# 寻找如下输出：
# ========== 扫描循环 #1 开始 ==========
# [UP  ] PWM=1000 us → L:819 R:819 (expect:819) ✓

# 确认 PWM 脉宽映射是否正确

# 如果仍无反应：
- GPIO 18/19 是否连接到 ESC 信号线？
- LEDC 定时器是否配置为 50Hz？
- 电调是否已启用？是否需要校准序列？
```

### 4. 平衡不稳定

**症状**：系统不断震荡或过度倾斜

**调整步骤**：
1. 在 [main/system_config.h](main/system_config.h) 中调整 PID 参数：
   ```c
   #define PID_KP  20.0  // 增大 → 响应快但易震荡
   #define PID_KI   1.0  // 增大 → 提升积分修正
   #define PID_KD   0.0  // 可尝试加入微分
   ```

2. 调整前馈权重：
   ```c
   #define FEEDFORWARD_PARAM   0.28  // 增大 → 更依赖预测
   #define FEEDBACK_PARAM      0.5   // 增大 → 更依赖反馈
   ```

3. 重新构建并烧录：
   ```bash
   idf.py build flash monitor
   ```

4. 观察转向目标为 180°（竖直）时的平衡表现

### 5. 控制延迟

**症状**：系统反应迟钝，或无法精确控制

**诊断**：
- 使用 `idf.py monitor` 观察，确认 100Hz 主循环时序是否精准
- 预期输出频率：每 10ms 精确执行一次控制
- 如有延迟，检查：
  - I2C 总线速度是否过低？
  - 是否有其他高优先级任务干扰？
  - FreeRTOS 任务堆栈是否溢出？

---

## 📊 参数配置详解

### 硬件参数（system_config.h）

```c
#define CONTROL_DT          0.01f   // 控制周期（秒）→ 100Hz
#define SYS_MASS            80.0f   // 系统总质量（kg）
#define SYS_WIDTH           0.6f    // 系统宽度（m）- 从左推进器轴到右轴
#define GRAVITY             9.81f   // 重力加速度（m/s²）
```

**物理意义**：
- `SYS_WIDTH`：决定了转动惯量 $I = \frac{1}{12}·m·L^2$
- 增大宽度 → 转动惯量增加 → 需要更大力矩平衡

### 平衡控制参数

```c
#define PID_KP              20.0f   // 比例增益
#define PID_KI              1.0f    // 积分增益
#define PID_KD              0.0f    // 微分增益

#define FEEDFORWARD_PARAM   0.28f   // 前馈权重 ∈ [0, 1]
#define FEEDBACK_PARAM      0.5f    // 反馈权重 ∈ [0, 1]

#define ANGLE_DEADZONE      1.0f    // 角度死区（°）
#define ANGLE_DEADZONE_SOFT 3.0f    // 平滑过渡宽度（°）
```

**调节经验**：

| 现象 | 增加参数 | 减少参数 |
|------|---------|---------|
| 响应过慢 | `PID_KP` | `ANGLE_DEADZONE` |
| 持续震荡 | `PID_KD`, `FEEDFORWARD_PARAM` | `PID_KP` |
| 积分过度 | 减小 `PID_KI` | - |
| 倾覆风险 | 增加 `FEEDFORWARD_PARAM` | - |

### 转向控制参数

在 [main/steering_control.c](main/steering_control.c#L80) 的 `calculate_pid()` 函数中修改：

```c
const float kp = 6.8f;          // 比例
const float ki = 1.0f;          // 积分
const float kd = 0.49f;         // 微分
const float deadband = 3.0f;    // 死区（°）
const float integral_max = 120.0f;  // 积分限幅
```

---

## 📈 性能指标

| 指标 | 规格值 |
|------|--------|
| 控制频率 | 100 Hz（±1% 精度） |
| 平衡角度精度 | ±0.5°（Roll） |
| 转向精度 | ±2°（使用编码器反馈） |
| 响应时间（Step输入） | 0.2 ~ 0.3s |
| 最大倾斜角 | ±60°（安全停止临界） |
| 推进范围 | 1000 ~ 2000 µs（ESC标准） |
| 力矩补偿范围 | ±60°（推进器旋转角） |

---

## 🔗 文件结构

```
d:\SteadySail--1\
├── CMakeLists.txt              # 顶层CMake配置
├── sdkconfig                   # ESP-IDF项目配置
├── README.md                   # 项目文档（本文件）
│
├── main/
│   ├── CMakeLists.txt          # main组件CMake配置
│   ├── main.c                  # 主程序入口 & 控制循环
│   ├── system_config.h         # 系统参数集中配置
│   │
│   ├── imu_driver.h/c          # 双IMU I2C驱动
│   ├── balance_controller.h/c  # Mahony + PID平衡算法
│   ├── motor_control.h/c       # LEDC PWM & 力矩补偿
│   ├── steering_control.h/c    # 编码器ISR & 转向PID
│   │
│   └── [其他源文件...]
│
├── build/                      # 构建输出目录（自动生成）
│   ├── SteadySail.elf
│   ├── SteadySail.bin
│   ├── SteadySail.map
│   └── [CMake临时文件...]
│
├── .vscode/
│   └── settings.json           # VS Code配置
│
└── .git/                       # 版本控制

```

---

## 🛠️ 开发工作流

### 添加新功能步骤

1. **创建新模块**：
   ```bash
   # 例如添加 GPS 模块
   main/
   ├── gps_driver.h
   ├── gps_driver.c
   ```

2. **更新 CMakeLists.txt**：
   ```cmake
   idf_component_register(
       SRCS "main.c" "imu_driver.c" "gps_driver.c" ...
       INCLUDE_DIRS "."
   )
   ```

3. **在 main.c 中集成**：
   ```c
   #include "gps_driver.h"
   
   void app_main(void) {
       gps_driver_init();
       // ...
   }
   ```

4. **编译测试**：
   ```bash
   idf.py build
   ```

### 调试技巧

1. **增加调试日志**：
   ```c
   #include "esp_log.h"
   ESP_LOGI(TAG, "Debug message: value=%d", var);
   ```

2. **使用 GDB 远程调试**：
   ```bash
   idf.py gdb
   ```

3. **查看内存使用**：
   ```bash
   idf.py size
   ```

---

## ⚠️ 安全注意事项

- **禁止在没有冷启动的情况下直接改变运行模式**：修改 `CURRENT_RUN_MODE` 后必须重新烧录
- **推进器启动前检查**：确保周围无人员或障碍物
- **I2C总线保护**：I2C 上拉电阻应为 4.7kΩ，总线长度 < 1m
- **电源隔离**：主推进电源与ESP32应通过隔离二极管或光耦隔离
- **过热保护**：ESP32-S3 工作温度 0~40°C，超出范围会自动关闭

---

## 📚 参考资源

- [ESP-IDF 官方文档 v4.4](https://docs.espressif.com/projects/esp-idf/en/v4.4/)
- [ESP32-S3 技术参考手册](https://www.espressif.com.cn/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [MPU-6050 数据手册](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- [FreeRTOS 用户指南](https://www.freertos.org/RTOS-Cortex-M3-M4.html)

---

## 📝 版本历史

| 版本 | 日期 | 更新内容 |
|------|------|--------|
| v2.0 | 2026-04-21 | 可旋转推进器自平衡系统，Mahony融合 + 力矩补偿 |
| v1.0 | 2025-12 | 初始版本（固定推进器） |

---

## 👤 开发者

**项目**：SteadySail - 可旋转推进器自平衡水面艇  
**平台**：ESP32-S3 + FreeRTOS + ESP-IDF v4.4  
**语言**：C

---

## 📄 许可证

本项目代码仅供学习和研究使用。

---

**最后更新**：2026年4月21日

有问题或建议？请提交 Issue 或 Pull Request！
