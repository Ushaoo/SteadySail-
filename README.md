# 🚤 SteadySail v2.1 - 可旋转推进器自平衡水面艇控制系统

**ESP32-S3 | 单IMU姿态融合 | Mahony四元数估计 | 实时100Hz控制 | 矢量推力补偿**

---

## 📋 项目概述

**SteadySail** 是一个基于 **ESP32-S3** 微控制器的**可旋转推进器自平衡水面艇（Unmanned Surface Vehicle, USV）自动控制系统**。该系统通过在桨板两侧安装独立可控的可旋转推进器，实现三层递进式功能控制：

### 三层功能架构

| 推进器状态 | 工作模式 | 主要功能 |
|----------|--------|--------|
| **竖直（0°）** | 🏊 **平衡模式** | 单IMU监测船体横滚，自动调节左右推力差产生平衡力矩，抵抗倾覆 |
| **水平（90°）** | 🚀 **推进模式** | 推进器完全水平，两侧推进器均匀出力提供前进动力 |
| **倾斜（0°~90°）** | 🎯 **混合模式** | 同时提供平衡与推进，根据旋转角自动补偿推力方向矢量 |

### 核心算法

- **传感器融合**：Mahony四元数固定增益融合（单IMU优化版）
- **姿态估计**：激进加速度修正 + 陀螺仪零偏自动校准
- **平衡控制**：前馈扰动补偿 + 2DOF反馈PID
- **推力分配**：矢量推力补偿 + 角度死区平滑

**系统以 100Hz 的实时控制频率运行**，确保在复杂海况下的稳定航行。

---

## ⚙️ 硬件配置

### 核心微控制器

| 参数 | 规格 |
|------|------|
| **型号** | ESP32-S3 |
| **处理器** | 双核Xtensa 32位 |
| **工作频率** | 240 MHz |
| **Flash存储** | ≥4 MB |
| **RAM** | ≥512 KB SRAM |
| **GPIO** | 48个（其中10个用于本系统） |

### GPIO 引脚分配（10个核心GPIO）

```
ESP32-S3 GPIO 引脚映射 (单IMU配置)
═══════════════════════════════════════════════════════

┌─ I2C 总线（单IMU传感器）──────────────────────────┐
│  I2C0: GPIO 8  (SDA) + GPIO 9  (SCL)  → IMU1     │
│  📌 仅连接I2C0，I2C1的GPIO 10/11可用于其他用途   │
└───────────────────────────────────────────────────┘

┌─ 转向编码器输入（MT6826S PWM占空比）─────────────┐
│  GPIO 4  → 左转向编码器（中断捕获）               │
│  GPIO 5  → 右转向编码器（中断捕获）               │
└───────────────────────────────────────────────────┘

┌─ 转向小电机 PWM 输出（50Hz）─────────────────────┐
│  GPIO 1  → 左转向舵机  (LEDC CH0, TIMER_0)       │
│  GPIO 2  → 右转向舵机  (LEDC CH1, TIMER_0)       │
└───────────────────────────────────────────────────┘

┌─ 主推进器 ESC PWM 输出（50Hz, 14-bit分辨率）────┐
│  GPIO 18 → 左推进电调  (LEDC CH2, TIMER_0)       │
│  GPIO 19 → 右推进电调  (LEDC CH3, TIMER_0)       │
└───────────────────────────────────────────────────┘

⚠️ I2C 上拉电阻：每条 I2C 线各需 4.7kΩ 上拉至 3.3V
⚠️ 推荐 PCB 设计中 I2C 和 GPIO 信号线与电源分离
```

### 传感器系统

#### 单IMU（MPU-6050 × 1）

**当前采用单IMU配置** (`USE_DUAL_IMU = 0`)，简化硬件设计，专注于单传感器融合算法。

- **加速度量程**：±16 g
- **角速度量范**：±2000 °/s
- **I2C地址**：0x68（AD0接GND）
- **采样率**：100 Hz（主控制循环驱动）
- **分辨率**：加速度14bit，角速度16bit

**当前硬件连接** 📌：

```
ESP32-S3 ──── I2C0 ──── MPU-6050 (IMU1)
  GPIO 8 (SDA) ────────── SDA
  GPIO 9 (SCL) ────────── SCL
  +3.3V ────────────────── +3.3V (或+5V经降压)
  GND ──────────────────── GND
  
上拉电阻：GPIO 8/9 各需一个 4.7kΩ 上拉至 +3.3V

备用GPIO：GPIO 10, 11 未使用，可用于其他模块
```

**升级到双IMU的步骤**（如需要）：

```
1. 购买支持改地址的MPU-6050（或使用AD0跳帽）
2. 在 main/system_config.h 修改：
   #define USE_DUAL_IMU  1
3. 硬件连接第二个IMU到I2C1 (GPIO 10/11)
4. 重新编译烧录
```

**对Mahony融合算法的影响**：
- 单IMU模式：使用固定自适应增益 (Kp=30.0)，加速度计校正强度高
- 双IMU模式：使用方差加权融合+自适应增益 (Kp=5+35*error)
```

#### 编码器（MT6826S PWM × 2）

- **输出格式**：PWM占空比（5%~95%对应0°~360°）
- **精度**：≤1°
- **频率**：~1 kHz
- **工作电压**：+5V

#### 推进电机（无刷直流 × 2）

- **驱动方式**：ESC电调（油门控制）
- **PWM频率**：50 Hz
- **脉宽范围**：1000~2000 µs（中立1500 µs）
- **典型功率**：50W per motor（满推力）

#### 转向舵机（小电机 × 2）

- **驱动方式**：PWM控制
- **频率**：50 Hz  
- **脉宽范围**：1000~2000 µs（中立1500 µs）
- **旋转范围**：约0°~180°（可到270°取决于模型）

### 电源系统

- **供电电压**：9V ~ 18V（推荐12V，对应3S LiPo电池）
- **系统功耗**：
  - 空载：~20W（仅MCU+IMU）
  - 满推力：~80W（包含电机）
- **推荐电池**：3000mAh 3S LiPo（约12V）

---

## 🏗️ 软件架构

### 模块化设计

```
┌──────────────────────────────────────────────┐
│              main.c (主程序)                  │
│  • app_main()：初始化所有模块                 │
│  • control_core_task()：100Hz实时控制循环    │
│  • UART命令处理任务：串口交互                │
└──────────────────┬───────────────────────────┘
                   │
        ┌──────────┼──────────┬──────────┐
        │          │          │          │
        ▼          ▼          ▼          ▼
   ┌────────┐ ┌────────┐ ┌────────┐ ┌────────┐
   │IMU驱动 │ │转向控制│ │电机驱动│ │平衡算法│
   │        │ │        │ │        │ │        │
   │• I2C   │ │• ISR   │ │• LEDC  │ │• 融合  │
   │• 数据  │ │• PID   │ │• PWM   │ │• 前馈  │
   │解析   │ │• 编码器│ │• 补偿  │ │• 反馈  │
   └────────┘ └────────┘ └────────┘ └────────┘
        ▲          ▲          ▲          ▲
        └──────────┴──────────┴──────────┘
                   │
                   ▼
        ┌─────────────────────────┐
        │  system_config.h (配置) │
        │  所有参数集中管理       │
        └─────────────────────────┘
```

### 实时控制流程（100Hz，每周期10ms）

```
时间轴 ──────────────────────────────────────────────────→

[0ms]  ┌─ IMU数据读取 (I2C, ~2ms)
       │    └─ I2C0: IMU1 (单IMU配置)
       │
       ▼
      [2ms] ┌─ 平衡算法计算 (Mahony+PID, ~1ms)
            │    ├─ 传感器融合
            │    ├─ 四元数更新
            │    ├─ 前馈计算
            │    └─ PID反馈
            │
            ▼
           [3ms] ┌─ 转向控制更新 (~0.5ms)
                 │    ├─ 编码器读取
                 │    └─ 转向PID计算
                 │
                 ▼
                [3.5ms] ┌─ 电机控制输出 (~0.5ms)
                        │    ├─ 推力矢量补偿
                        │    ├─ PWM计算
                        │    └─ LEDC更新
                        │
                        ▼
                       [4ms] ┌─ 安全检查
                              │   ├─ 倾斜角保护
                              │   └─ 推力限幅
                              │
                              ▼
                             [5ms] ┌─ 精确延时至10ms
                                    └─ vTaskDelayUntil()
                                        ↓
                                    [10ms] → 循环重启
```

**精度**：±0.1ms（FreeRTOS绝对定时）

### 核心算法模块详解

#### 1️⃣ 单IMU传感器数据处理

**当前采用单IMU配置** (`USE_DUAL_IMU = 0`)，IMU驱动直接从I2C0读取一个MPU-6050传感器：

```
数据流程：
步骤1：I2C0读取原始数据（16bit加速度+16bit陀螺仪）
   araw_x/y/z = 16-bit AD转换值
   ωraw_x/y/z = 16-bit 陀螺仪值

步骤2：数据标准化
   a_g = araw / 2048.0   (±16g量程转换到±8g，实际用±1g)
   ω_dps = ωraw / 131.0  (角速度转换到°/s)

步骤3：归一化加速度（准备用于四元数修正）
   norm = sqrt(ax² + ay² + az²)
   if norm > 0.01:
       a_normalized = [ax/norm, ay/norm, az/norm]  // 单位向量

步骤4：数据输出给平衡控制器
   balance_controller_update(imu_data, &state)
```

**单IMU特性**：
- 无多传感器融合，算法简化
- Mahony融合使用较高的自适应增益 (Kp=30.0)
- 对加速度计噪声依赖度高
- 适合快速原型和初期调试

**升级到双IMU的优势**（可选）：
- 方差加权融合能自动选择更可靠的传感器
- 自适应Mahony增益 (Kp=5+35*error) 更温和
- 强干扰抑制能力

#### 2️⃣ Mahony四元数姿态融合

**算法核心**：
```
四元数微分方程：
   dq/dt = 0.5 · ω × q

其中ω包含加速度修正项（修正陀螺仪漂移）：
   ω = ω_gyro + Kp_adaptive · e_accel

自适应增益设置：
   当前单IMU模式：
      Kp = 30.0f  (激进修正，快速消除漂移)
      reason: 仅依赖一个传感器，需要强加速度校正
   
   可选双IMU模式：
      error_acc = |a_measured| - |g|
      Kp = 5.0 + 35.0 · |error_acc|  (自适应)
      reason: 方差融合已处理传感器可靠性，修正可更温和

欧拉角解算（从四元数提取）：
   Roll  = atan2(2(q0·q1 + q2·q3), 1 - 2(q1² + q2²))
   Pitch = asin(2(q0·q2 - q3·q1))
   Yaw   = atan2(2(q0·q3 + q1·q2), 1 - 2(q2² + q3²))
```

**优势**：
- 无矩阵运算，计算量小（适合嵌入式）
- 自动处理万向锁
- 收敛速度快
- 单IMU模式固定增益，参数简单

#### 3️⃣ 陀螺仪零偏自动校准

```
启动阶段（前2秒）：
   1. 收集200个陀螺仪样本（静止状态）
   2. 计算零偏均值：
      bias_x = (1/200) · Σ ω_x
      bias_y = (1/200) · Σ ω_y
      bias_z = (1/200) · Σ ω_z
   3. 运行中持续应用：
      ω_corrected = ω_raw - bias

校准完成后不再进入此分支，保持零偏值不变。
系统启动时保持静止2秒完成校准。
```

#### 4️⃣ 前馈+反馈混合平衡控制

**前馈部分**（预测性补偿）：
```
物理模型：
   τ_disturb = m·g·(L/2)·sin(θ)   // 重力倾覆力矩
   τ_inertia = I·α                // 惯量补偿
   τ_self = -K_self·θ             // 虚拟刚度（防外翻）
   
前馈输出：
   τ_ff = -(τ_disturb + τ_inertia + τ_self)
```

**反馈部分**（误差修正）：
```
2DOF PID控制律：
   e = 0 - θ                      // 目标0°

   P项：
      P = KP·e  (if |θ| > DEADZONE)
      P = 0     (otherwise)
   
   I项：
      I_rate = KI·e
      I += I_rate·dt
      I = clamp(I, -I_max, +I_max)
   
   D项：
      D = 0  (实验表明D项容易引入高频噪声)
   
   反馈输出：
      τ_pid = P + I

系数设置（main/system_config.h）：
   KP = 20.0f
   KI = 1.0f
   KD = 0.0f
```

**最终控制输出**：
```
混合权重求和：
   τ_total = FEEDFORWARD_PARAM·τ_ff - FEEDBACK_PARAM·τ_pid
   
   FEEDFORWARD_PARAM = 0.28  // 前馈权重
   FEEDBACK_PARAM = 0.5      // 反馈权重
   
   限幅：
   τ_total = clamp(τ_total, -500, +500)
```

#### 5️⃣ 推进器旋转补偿

**物理约束**：
```
当推进器旋转角度α时，竖直方向推力分量：
   F_vertical = F_total · cos(α)

为保持竖直平衡力矩τ不变，需补偿：
   τ_compensated = τ_total / cos(α)

限制条件（避免分母过小，α>60°时不补偿）：
   α_clamped = clamp(α, -60°, +60°)
   cos(60°) = 0.5，最大补偿2倍
```

**PWM映射**：
```
力矩 → PWM增量：
   Δ_pwm = τ_compensated × 0.55   (比例系数)
   
左右推进器输出：
   thrust_L = 1500 + Δ_pwm      (增加左推力)
   thrust_R = 1500 - Δ_pwm      (减少右推力)
   → 产生力偶矩恢复平衡

限幅：
   Δ_pwm = clamp(Δ_pwm, -500, +500)
   thrust_L = clamp(thrust_L, 1000, 2000)
   thrust_R = clamp(thrust_R, 1000, 2000)
```

---

## 🎮 运行模式

编辑 [main/system_config.h](main/system_config.h#L11)，修改 `CURRENT_RUN_MODE`：

### 六种模式说明

| 模式 | 定义值 | 核心功能 | 用途 |
|------|------|--------|------|
| `MODE_TEST_SENSORS` | 0 | 输出IMU和编码器原始数据 | 硬件连接测试 |
| `MODE_TEST_STEERING_ONLY` | 1 | 小电机PID（大电机锁定1500µs） | 转向系统调试 |
| `MODE_TEST_BALANCE_ONLY` | 2 | 平衡算法（小电机锁定180°） | 平衡控制调试 |
| `MODE_FULL_INTEGRATION` | 3 | 完整集成运行（推荐生产）| 全系统集成测试 |
| `MODE_CALIBRATE_ESC` | 4 | PWM扫描1000→2000→1000µs | ESC初始化校准 |
| `MODE_TEST_IMU_ONLY` | 5 | 实时IMU原始和融合数据 | 传感器融合验证 |

### 快速切换方法

```bash
# 1. 编辑配置文件
nano main/system_config.h
# 修改第11行的 CURRENT_RUN_MODE

# 2. 重新构建并烧录
idf.py build flash monitor

# 3. 观察串口输出验证模式
# 会看到对应模式的初始化日志
```

---

## 🚀 快速开始

### 前置环境

- **操作系统**：Windows 10+, Linux, macOS
- **ESP-IDF**：v4.4（使用ESP-IDF Tools Installer）
- **VS Code**：1.70+
- **Python**：3.8+
- **VS Code扩展**：
  - ESP-IDF (by Espressif)
  - C/C++ (by Microsoft)
  - CMake Tools (by Microsoft)

### 1️⃣ 安装ESP-IDF v4.4

#### Windows（推荐Installer方式）

```bash
# 下载安装程序
# https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/get-started/windows-setup.html

# 运行Installer，选择v4.4分支
# 安装完成后设置环境变量：
set IDF_PATH=C:\Espressif\frameworks\esp-idf-v4.4

# 验证
idf.py --version
# 输出: ESP-IDF v4.4.8
```

#### Linux/macOS（手动安装）

```bash
mkdir -p ~/esp
cd ~/esp
git clone --branch v4.4 https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh

# 配置环境变量
echo "source ~/esp/esp-idf/export.sh" >> ~/.bashrc
source ~/.bashrc

# 验证
idf.py --version
```

### 2️⃣ VS Code 配置ESP-IDF扩展

1. 打开VS Code
2. 安装扩展 **"ESP-IDF extension"** (by Espressif)
3. 按 `Ctrl+Shift+P` 打开命令面板
4. 输入 `ESP-IDF: Configure ESP-IDF extension`
5. 选择 **Advanced** 配置方式
6. 按提示选择：
   - ESP-IDF安装路径（如 C:\Espressif\frameworks\esp-idf-v4.4）
   - Python可执行文件路径
   - 版本：**v4.4**
7. 点击 **Install** 完成

### 3️⃣ 克隆并打开项目

```bash
# 克隆仓库
git clone <repository_url> d:\SteadySail--1
cd d:\SteadySail--1

# 用VS Code打开
code .
```

### 4️⃣ 构建项目

```bash
# 方式1：VS Code命令（推荐）
# 按 Ctrl+Shift+P → 输入 "ESP-IDF: Build" → 执行

# 方式2：命令行
idf.py build

# 预期输出
# [100%] Built target version_2
# Build complete. The following files were created:
# - build/version_2.elf
# - build/version_2.bin
```

### 5️⃣ 烧录到硬件

```bash
# 步骤1：选择串口（首次需要）
idf.py port-detect    # 检测可用端口
idf.py set-port COM5  # 选择端口（Windows例）
# 或 idf.py set-port /dev/ttyUSB0  (Linux)

# 步骤2：烧录并监控（一键完成）
idf.py build flash monitor

# 或分步执行：
idf.py flash                    # 仅烧录
idf.py monitor                  # 仅监控（按Ctrl+]退出）

# 预期输出
I (40) MAIN: SteadySail Version 2.0
I (60) IMU_DRIVER: 单 MPU6050 (I2C0) 初始化成功！
I (70) MOTOR: === Motor Control Initialized Successfully! ===
I (80) STEERING: Steering Encoder Interrupts Initialized.
```

---

## 📊 实时控制与交互

### 串口命令（MODE_FULL_INTEGRATION）

启动后通过串口发送命令控制系统：

| 命令 | 功能 | 示例 |
|------|------|------|
| `A` / `a` | 左转 +15° | `>>> 偏角 +15°，目标: 195.0° <<<` |
| `D` / `d` | 右转 -15° | `>>> 偏角 -15°，目标: 165.0° <<<` |
| `S` / `s` | 回正 (0°) | `>>> 转向回正！目标: 180.0° <<<` |
| 数字(0-360) | 设置绝对角度 | `90` → 目标转向90° |

### 实时数据输出

系统每秒输出（10Hz）关键指标：

```
[转向数据]      [姿态数据]      [力矩数据]
─────────────────────────────────────────────────
Target:180.0 | CurL:180.2 CurR:179.8 | Roll:+2.34° Tau:+12.56
Target:90.0  | CurL:91.5 CurR:88.2   | Roll:-1.05° Tau:-8.23
Target:45.0  | CurL:45.1 CurR:44.9   | Roll:+0.12° Tau:+1.20
```

**字段解释**：
- `Target`：转向目标角度（度）
- `CurL/CurR`：左右推进器实际角度（来自编码器）
- `Roll`：船体横滚角（度，右倾正）
- `Tau`：平衡系统输出的力矩增量

---

## 🔧 参数调整

### 物理参数（main/system_config.h）

```c
#define CONTROL_DT          0.01f   // 控制周期(s) → 100Hz
#define SYS_MASS            80.0f   // 系统质量(kg)
#define SYS_WIDTH           0.6f    // 系统宽度(m) - 左右推进器距离
#define GRAVITY             9.81f   // 重力加速度(m/s²)
```

### 平衡控制参数

```c
#define PID_KP              20.0f   // 比例系数（↑ 响应快/易震荡）
#define PID_KI              1.0f    // 积分系数（↑ 提升补偿能力）
#define PID_KD              0.0f    // 微分系数（通常不用，易引噪）

#define FEEDFORWARD_PARAM   0.28f   // 前馈权重 ∈ [0, 1]
#define FEEDBACK_PARAM      0.5f    // 反馈权重 ∈ [0, 1]

#define ANGLE_DEADZONE      1.0f    // 角度死区(°)  - 内侧硬限
#define ANGLE_DEADZONE_SOFT 3.0f    // 软死区宽度(°) - 平滑过渡
```

### 调参经验表

| 现象 | 解决方案 |
|------|--------|
| 响应过慢 | ↑ `PID_KP` 或 ↓ `ANGLE_DEADZONE` |
| 持续震荡 | ↑ `PID_KD` 或 ↑ `FEEDFORWARD_PARAM` 或 ↓ `PID_KP` |
| 积分过度 | ↓ `PID_KI` 或 ↓ `FEEDBACK_PARAM` |
| 倾覆风险 | ↑ `FEEDFORWARD_PARAM` 或 ↓ `ANGLE_DEADZONE` |
| 推进器抖动 | ↑ `ANGLE_DEADZONE_SOFT` （增大平滑区间） |

### 转向控制参数

编辑 [main/steering_control.c](main/steering_control.c#L80) 的 `calculate_pid()` 函数：

```c
const float kp = 6.8f;              // 比例系数
const float ki = 1.0f;              // 积分系数
const float kd = 0.49f;             // 微分系数
const float deadband = 3.0f;        // 死区(°)
const float integral_max = 120.0f;  // 积分限幅
```

---

## 🔍 故障排查

### ❌ IMU连接失败

**症状**：启动后输出 `IMU 初始化失败`

```bash
# 1. 进入传感器测试模式
# 编辑 main/system_config.h
#define CURRENT_RUN_MODE   MODE_TEST_SENSORS

# 2. 重新烧录
idf.py build flash monitor

# 3. 检查项（单IMU配置）
# □ GPIO 8(SDA) 和 GPIO 9(SCL) 是否正确连接到MPU-6050？
# □ MPU-6050 电源(+3.3V, GND)是否稳定？
# □ I2C 上拉电阻(4.7kΩ) 在 GPIO 8/9 线上是否在位？
# □ MPU-6050 的 AD0 引脚是否接 GND（地址0x68）？
# □ I2C 线是否接触良好？（尝试轻轻摇晃杜邦线查看是否出错）
# □ GPIO 10/11 无需连接（I2C1在单IMU模式下关闭）
```

### ❌ 编码器数据为0

**症状**：转向角度始终显示 180° 或垃圾数据

```bash
# 1. 检查硬件
# □ GPIO 4/5 是否接收编码器信号？
# □ MT6826S 是否有 +5V 电源？
# □ 编码器数据线是否接触良好？

# 2. 校准编码器
# 进入 MODE_TEST_SENSORS 模式
# 保持推进器竖直向下(0°状态)
# 系统启动时自动校准（steering_control_calibrate_encoders()）

# 3. 验证信号
# 手动旋转推进器，观察编码器占空比输出是否变化
```

### ❌ 推进器无反应

**症状**：电调不工作，电机不转

```bash
# 1. 进入ESC校准模式测试
#define CURRENT_RUN_MODE   MODE_CALIBRATE_ESC

# 2. 观察输出
# [UP  ] PWM=1000 us → duty=819 ✓
# [DOWN] PWM=2000 us → duty=1638 ✓

# 3. 检查项
# □ GPIO 18/19 是否连接到 ESC 信号线？
# □ LEDC 定时器是否配置为50Hz？
# □ 电调是否已通电且进入油门监听模式？
# □ PWM脉宽映射公式是否正确？
```

### ❌ 平衡不稳定

**症状**：系统不断震荡、翻滚或完全无法平衡

```bash
# 1. 增加调试输出
# 在 main.c 中启用详细日志，观察：
# - Roll 角度变化
# - Tau 力矩输出
# - IMU 方差值

# 2. 逐步调整参数
# a. 先增大死区，降低对小扰动的响应
#    #define ANGLE_DEADZONE 2.0f
#
# b. 减小 PID_KP，降低增益
#    #define PID_KP 15.0f
#
# c. 增大前馈权重（更依赖预测）
#    #define FEEDFORWARD_PARAM 0.35f
#
# d. 减小反馈权重（降低反馈强度）
#    #define FEEDBACK_PARAM 0.3f

# 3. 重新构建测试
idf.py build flash monitor
```

---

## 📈 性能指标

| 指标 | 规格值 | 说明 |
|------|--------|------|
| 控制频率 | **100 Hz** | ±1% 精度（FreeRTOS绝对定时） |
| 平衡精度 | ±0.5° | Roll角误差（静止海面） |
| 转向精度 | ±2° | 编码器反馈精度 |
| 响应时间 | 0.2~0.3s | Step阶跃输入到90%稳定 |
| 最大倾角 | ±60° | 安全停止临界值（防翻车） |
| 推进范围 | 1000~2000 µs | ESC标准格式 |
| 力矩补偿 | ±60° | 推进器旋转角范围 |
| IMU更新率 | 100 Hz | 主控制循环驱动采样 |

---

## 🗂️ 文件结构

```
d:\SteadySail--1\
├── CMakeLists.txt                  # 顶层CMake配置
├── sdkconfig                       # ESP-IDF项目配置
├── README.md                       # 本文档
│
├── main/
│   ├── CMakeLists.txt              # main组件CMake配置
│   ├── main.c                      # 主程序入口 & 控制循环（~300行）
│   ├── system_config.h             # 所有参数集中配置（~80行）
│   │
│   ├── imu_driver.h/c              # I2C IMU驱动（~200行）
│   │   └─ 当前单IMU配置、支持单/双切换、自动零偏校准
│   │
│   ├── balance_controller.h/c      # Mahony姿态融合+PID平衡（~300行）
│   │   └─ Mahony四元数、方差融合、前馈反馈混合
│   │
│   ├── motor_control.h/c           # LEDC PWM & 推力补偿（~250行）
│   │   └─ 50Hz PWM输出、矢量推力补偿、ESC校准
│   │
│   ├── steering_control.h/c        # 编码器ISR & PID转向（~200行）
│   │   └─ GPIO中断、占空比解析、环形误差PID
│   │
│   └── [其他未列出的源文件...]
│
├── build/                          # 构建输出目录（自动生成）
│   ├── version_2.elf               # 可执行固件
│   ├── version_2.bin               # 二进制文件
│   ├── compile_commands.json       # 编译命令（用于IDE）
│   └── [CMake配置文件...]
│
├── .vscode/
│   ├── settings.json               # VS Code工作空间配置
│   ├── launch.json                 # 调试器配置
│   └── tasks.json                  # 构建任务
│
└── .git/                           # 版本控制（可选）
```

### 源代码统计

| 模块 | 文件 | 代码行数 | 功能 |
|------|------|--------|------|
| main | main.c | ~300 | 主控制循环、模式切换 |
| config | system_config.h | ~80 | 集中参数管理 |
| IMU | imu_driver.c/h | ~200 | I2C驱动、数据读取 |
| 平衡 | balance_controller.c/h | ~300 | Mahony+PID融合 |
| 电机 | motor_control.c/h | ~250 | LEDC PWM、补偿 |
| 转向 | steering_control.c/h | ~200 | 编码器ISR、PID |
| **总计** | **6个主文件** | **~1330行** | 完整闭环系统 |

---

## 🛠️ 开发与扩展

### 添加新功能

1. **创建模块**：
   ```bash
   # 例如添加GPS模块
   touch main/gps_driver.h main/gps_driver.c
   ```

2. **更新CMakeLists.txt**：
   ```cmake
   idf_component_register(
       SRCS "main.c" "imu_driver.c" "gps_driver.c" ...
       INCLUDE_DIRS "."
   )
   ```

3. **在main.c中集成**：
   ```c
   #include "gps_driver.h"
   void app_main(void) {
       gps_driver_init();
       // ...
   }
   ```

4. **测试**：
   ```bash
   idf.py build flash monitor
   ```

### 调试技巧

```bash
# 1. 实时日志过滤
idf.py monitor -F IMU_DRIVER

# 2. 输出二进制日志分析
idf.py build flash
idf.py monitor --decode-coredump=gdbstub

# 3. 内存使用统计
idf.py size

# 4. 项目大小分析
idf.py size-files
```

---

## ⚠️ 安全注意事项

- ✅ **系统启动前检查**：确保周围无人员或障碍物
- ✅ **修改模式需重烧**：修改 `CURRENT_RUN_MODE` 后必须重新烧录
- ✅ **I2C总线保护**：每条线都需4.7kΩ上拉电阻，长度<1m
- ✅ **电源隔离**：主推进电源(12V)应与ESP32(3.3V)隔离
- ✅ **过热保护**：ESP32-S3工作温度0~40°C，超出自动关闭
- ✅ **倾覆保护**：系统在Roll>60°时自动紧急停止

---

## 📚 参考资源

- 📖 [ESP-IDF官方文档 v4.4](https://docs.espressif.com/projects/esp-idf/en/v4.4/)
- 📖 [ESP32-S3技术参考手册](https://www.espressif.com.cn/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- 📖 [MPU-6050数据手册](https://www.invensense.com/products/motion-tracking/6-axis/mpu-6050/)
- 📖 [FreeRTOS用户指南](https://www.freertos.org/RTOS-Cortex-M3-M4.html)
- 📖 [Mahony滤波器论文](http://dx.doi.org/10.2514/1.47282)

---

## 📝 版本历史

| 版本 | 日期 | 核心更新 |
|------|------|--------|
| **v2.1** | 2026-04-23 | 单IMU配置、简化硬件、固定增益Mahony融合 |
| v2.0 | 2026-04-23 | 可旋转推进器、Mahony四元数融合、矢量推力补偿、实时100Hz控制 |
| v1.5 | 2025-03 | 双IMU方差加权融合、自适应Mahony增益 |
| v1.0 | 2025-01 | 初始版本（固定推进器、基础平衡） |

---

## 🤝 贡献与反馈

- 发现bug？提交Issue
- 有改进建议？提交Pull Request
- 问题讨论？参加Discussions

---

## 📄 许可证

本项目代码仅供**学习和研究使用**。商业用途需获得授权。

---

**最后更新**：2026年4月23日  
**开发平台**：ESP32-S3 | FreeRTOS | ESP-IDF v4.4  
**编程语言**：C (ISO C11)

**有问题？** 👉 提交Issue或查阅[故障排查](#🔍-故障排查)部分
