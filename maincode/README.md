# 🚤 SteadySail ESP32-S3 船舶自稳定系统

**完整的 C++ / ESP-IDF 自稳定平台控制系统**

版本: 1.0 | 日期: 2026-04-14 | 目标硬件: ESP32-S3

---

## 📋 目录

- [快速开始](#-快速开始-3-分钟)
- [硬件要求](#-硬件要求)
- [软件环境](#-软件环境)
- [项目结构](#-项目结构)
- [配置说明](#-配置说明)
- [编译和烧录](#-编译和烧录)
- [调试方法](#-调试方法)
- [系统架构](#-系统架构)
- [常见问题](#-常见问题)
- [参数调整](#-参数调整)

---

## 🚀 快速开始 (3 分钟)

### 前置条件
- ✅ 已安装 **ESP-IDF VS Code 扩展**（见下文）
- ✅ 已连接 ESP32-S3 开发板

### 步骤 1: 打开项目
```bash
# 在 VS Code 中打开此文件夹
File → Open Folder → d:\SteadySail--1\maincode
```

### 步骤 2: 编译
```
按 Ctrl+Shift+P，输入 "ESP-IDF: Build your Project"
或点击 VS Code 底部 "Build" 按钮
```

### 步骤 3: 烧录
```
按 Ctrl+Shift+P，输入 "ESP-IDF: Flash your Project"
或点击 VS Code 底部 "Flash" 按钮
选择串口（通常 COM3 或 COM4）
```

### 步骤 4: 监控
```
按 Ctrl+Shift+P，输入 "ESP-IDF: Monitor your Device"
观察实时日志输出
```

**预期输出**:
```
=== SteadySail ESP32-S3 System Initialization ===
I2C initialized: port=0, SDA=21, SCL=22, freq=400000 Hz
Scanning I2C devices...
I2C scan found device at: 0x68  ← IMU1
I2C scan found device at: 0x69  ← IMU2
I2C scan found device at: 0x40  ← PCA9685
Initializing control loop...
MPU6050 @ 0x68 initialized
MPU6050 @ 0x69 initialized
PCA9685 @ 0x40 initialized
Control loop task created (priority=20, stack=8192 bytes)
=== System initialization complete ===
```

✅ **系统已启动并运行 100 Hz 控制循环**

---

## 🛠 硬件要求

### 主控制器
- **ESP32-S3** 开发板（必需）
- USB-C 数据线（用于烧录和监控）

### 传感器
| 器件 | 型号 | I2C 地址 | 连接 |
|------|------|---------|------|
| IMU1 | MPU6050 | 0x68 | GPIO21(SDA), GPIO22(SCL) |
| IMU2 | MPU6050 | 0x69 | GPIO21(SDA), GPIO22(SCL) |

**I2C 连接图**:
```
ESP32-S3           IMU1 & IMU2
─────────────────────────────
GPIO21 (SDA) ───→ SDA (都连到同一线)
GPIO22 (SCL) ───→ SCL (都连到同一线)
GND         ───→ GND
3V3         ───→ VCC

注: 两个 IMU 的产物地址通过 AD0 引脚区分
  - IMU1: AD0 接 GND → 0x68
  - IMU2: AD0 接 VCC → 0x69
```

### 执行器
| 器件 | 型号 | I2C 地址 | 用途 |
|------|------|---------|------|
| PWM 驱动 | PCA9685 | 0x40 | 16 通道 PWM |

**PWM 通道分配**:
```
通道 0: 左推进器 (LEFT_THRUSTER)
通道 1: 右推进器 (RIGHT_THRUSTER)
通道 2: 左旋转舵机 (LEFT_ROTATION_CH)
通道 3: 右旋转舵机 (RIGHT_ROTATION_CH)
通道 4-15: 预留
```

---

## 💻 软件环境

### 系统要求
- **Windows 10/11** 或 **Linux/macOS**
- **VS Code** 1.70+ 版本
- **Python** 3.8+

### 必需工具

#### 1. 安装 ESP-IDF VS Code 扩展（推荐方式）

**方法 A: 直接安装**（最简单）
```
1. 打开 VS Code
2. 按 Ctrl+Shift+X 打开扩展市场
3. 搜索 "ESP-IDF"
4. 找到 "Espressif IDF" 官方扩展
5. 点击 "Install"
6. 等待安装完成（包自动下载 ESP-IDF）
```

**方法 B: 手动配置**（如扩展无法自动配置）
```bash
# 1. 下载 ESP-IDF (官方推荐 v5.0 或 v5.1)
git clone https://github.com/espressif/esp-idf.git ~/esp-idf

# 2. 运行安装脚本
cd ~/esp-idf
./install.ps1  # 在 Windows PowerShell 中执行

# 3. 在 VS Code 中配置 IDF_PATH
按 Ctrl+Shift+P → "ESP-IDF: Configure ESP-IDF Extension"
选择已安装的 ESP-IDF 路径
```

#### 2. 验证环装
```bash
# 打开 PowerShell，运行以下命令验证
idf.py --version  # 应显示 "ESP-IDF v5.0" 或更新版本
esptool.py version  # 应显示版本号
```

### 推荐扩展

| 扩展名 | 发布者 | 用途 |
|-------|-------|------|
| **Espressif IDF** | Espressif | 编译、烧录、监控 |
| C/C++ | Microsoft | 代码补全、调试 |
| Cortex-Debug | Arm | 硬件调试 |

---

## 📁 项目结构

```
maincode/                           ← 单击打开此文件夹
│
├── CMakeLists.txt                  ← 根 CMake 配置
├── sdkconfig                       ← ESP-IDF 编译配置
├── build/                          ← 编译输出 (自动生成)
│
└── main/                           ← 核心代码
    ├── CMakeLists.txt              ← 组件注册 (8 个源文件)
    ├── config.h                    ← 🔑 全局配置（调参从这里开始）
    ├── main.c                      ← 程序入口
    │
    ├── i2c_driver.c/h              ← I2C 底层驱动
    │
    ├── sensors/                    ← 传感器模块
    │   ├── mpu6050_driver.c/h      ← IMU 寄存器驱动
    │   └── imu_fusion.c/h          ← 双 IMU 融合算法 ⭐
    │
    ├── actuators/                  ← 执行器模块
    │   ├── pca9685_driver.c/h      ← PWM 驱动
    │   └── motor_controller.c/h    ← 电机控制逻辑
    │
    ├── control/                    ← 控制模块
    │   ├── pid_controller.c/h      ← PID 控制器
    │   ├── deadzone.c/h            ← 死区处理
    │   └── control_loop.c/h        ← 主控制循环 (100 Hz) ⭐⭐
    │
    └── utils/                      ← 辅助函数 (预留)
```

### 文件说明

| 文件 | 功能 | 调参 |
|------|------|------|
| `config.h` | 全局配置（I2C、PID、死区等） | ⭐⭐⭐ 最常改 |
| `imu_fusion.c/h` | 双 IMU 融合算法 | ⭐ 一般不改 |
| `control_loop.c/h` | 主控制循环 | ⭐⭐ 偶尔改 |
| `pid_controller.c/h` | PID 控制 | ⭐⭐ 调参用 |
| `motor_controller.c/h` | 电机驱动 | ⭐ 一般不改 |

---

## ⚙️ 配置说明

### 在 `config.h` 中调参

所有系统参数都在 `main/config.h` 中集中定义。打开此文件按需修改：

#### 1. I2C 硬件配置
```c
#define I2C_PORT            I2C_NUM_0      // I2C 端口
#define I2C_SDA_PIN         21             // SDA 引脚
#define I2C_SCL_PIN         22             // SCL 引脚
#define I2C_FREQ_HZ         400000         // I2C 频率 (Hz)
```

#### 2. PID 控制参数（最重要）
```c
#define PID_KP              20.0f          // 比例增益（越大响应越快）
#define PID_KI              1.0f           // 积分增益（消除稳态误差）
#define PID_KD              0.0f           // 微分增益（通常为 0）
#define PID_B               0.8f           // 比例权重 (保持 0.8)
#define PID_C               0.0f           // 微分权重 (保持 0.0)
```

**调参指南**:
```
如果船体振荡过度  → 减小 Kp (如 15.0)
如果响应太慢      → 增大 Kp (如 25.0)
如果偏离不回正    → 增大 Ki (如 2.0)
如果颤抖增多      → 减小 Ki (如 0.5)
```

#### 3. 死区参数
```c
#define ANGLE_DEADZONE      1.0f           // 角度死区核心 (度)
#define ANGLE_DEADZONE_SOFT 3.0f           // 软死区边界 (度)
```

**含义**:
```
|Roll| < 1.0°      → 不响应 (完全死区)
1.0° ~ 3.0°       → 平滑过渡 (软死区)
|Roll| ≥ 3.0°     → 完整响应
```

#### 4. 电机参数
```c
#define BASE_PULSE          1500           // 中立脉宽 (μs)
#define MIN_PULSE           1000           // 最小脉宽
#define MAX_PULSE           2000           // 最大脉宽
#define THRUST_SCALE        0.55f          // 推力缩放系数
```

#### 5. IMU 轴向配置
```c
#define IMU1_INVERT_X       1              // 1=反转, 0=正向
#define IMU1_INVERT_Y       1
#define IMU1_INVERT_Z       0

#define IMU2_INVERT_X       0
#define IMU2_INVERT_Y       0
#define IMU2_INVERT_Z       0
```

**如何判断需要反转**:
1. 运行系统，观察日志中的 Roll 角
2. 船体倾向右时，如果 Roll 显示为负值，则需要反转

#### 6. 融合参数（一般不改）
```c
#define ALPHA_ACC           0.98f          // 加速度互补系数
#define ALPHA_EMA           0.15f          // EMA 低通滤波系数
```

---

## 🔧 编译和烧录

### 使用 VS Code 扩展（推荐）

#### 编译步骤
```
1. 在 VS Code 中打开项目文件夹
2. 按 Ctrl+Shift+P
3. 输入 "Build"
4. 选择 "ESP-IDF: Build your Project"
5. 等待编译完成（第一次约 2-3 分钟）
```

**成功标志**:
```
[100%] Built target steadysail_esp32s3
Build complete! ESP32-S3 project ...
```

**编译失败解决**:
```
如果看到 "Error: IDF_PATH not found"
→ 按 Ctrl+Shift+P → "ESP-IDF: Configure ESP-IDF Extension"
→ 选择 ESP-IDF 安装路径

如果看到 C 编译错误
→ 检查 config.h 中是否有语法错误
→ 或重新配置 ESP-IDF 扩展
```

#### 烧录步骤
```
1. 用 USB-C 数据线连接 ESP32-S3
2. 按 Ctrl+Shift+P
3. 输入 "Flash"
4. 选择 "ESP-IDF: Flash your Project"
5. 选择正确的串口（通常 COM3 或 COM4）
6. 等待烧录完成

进度显示类似:
  Writing at 0x00000000...
  [============================] 100%
  Wrote XXX bytes to address 0x00000000 in 2.3 seconds
```

#### 监控步骤
```
1. 烧录完成后，按 Ctrl+Shift+P
2. 输入 "Monitor"
3. 选择 "ESP-IDF: Monitor your Device"
4. 实时查看系统日志
```

#### 停止监控
```
按 Ctrl+C 或点击终端窗口右上角的关闭按钮
```

### 使用命令行（高级）

如果 VS Code 扩展出问题，可用命令行烧录：

```bash
# 进入项目目录
cd d:\SteadySail--1\maincode

# 编译
idf.py build

# 烧录 (替换 COM_PORT，如 COM3)
idf.py -p COM_PORT flash

# 监控
idf.py -p COM_PORT monitor

# 一次性执行上述三个操作
idf.py -p COM_PORT build flash monitor
```

---

## 🔍 调试方法

### 方法 1: 串口监控日志

这是最简单的调试方法。按照上文的"监控步骤"运行监控，观察以下日志：

**关键日志信息**:
```
I2C initialized                          ← I2C 初始化成功
I2C scan found device at: 0x68           ← IMU1 检测到
I2C scan found device at: 0x69           ← IMU2 检测到
MPU6050 @ 0x68 initialized               ← IMU1 初始化成功
MPU6050 @ 0x68 calibration complete      ← IMU1 校准完成
Gyro bias: x=0.05, y=0.02, z=-0.01      ← 陀螺仪零偏

PCA9685 @ 0x40 initialized               ← PWM 驱动初始化成功
Motor controller initialized             ← 电机驱动初始化成功

Control loop task created                ← 100Hz 循环已启动
Loops=500 | Attitude: roll=-0.5° | ...   ← 运行状态（每 500 次循环打一次）
```

### 方法 2: 添加自定义日志

在 `control_loop.c` 的 `control_loop_task()` 函数中添加：

```c
// 在 Step 6 之后添加调试信息
if ((loop->loop_count % 100) == 0) {
    ESP_LOGI(TAG, "[DEBUG] Roll=%.2f° | GyroX=%.2f°/s | PID_out=%.2f | L=%u, R=%u",
             euler.roll, gyro1.x, pid_output, pulse_left, pulse_right);
}
```

重新编译烧录后，每 1 秒会打印一次详细信息。

### 方法 3: 监控特定传感器数据

在 `main()` 中添加临时代码读取原始数据：

```c
// 临时测试代码（调试完成后删除）
static void debug_imu_data(void) {
    mpu6050_t imu;
    mpu6050_init(&imu, IMU1_ADDRESS);
    
    float_data_t accel, gyro;
    for (int i = 0; i < 10; i++) {
        mpu6050_read_accel_g(&imu, &accel);
        mpu6050_read_gyro_dps(&imu, &gyro);
        ESP_LOGI("DEBUG", "Accel: (%.2f, %.2f, %.2f) | Gyro: (%.2f, %.2f, %.2f)",
                 accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

---

## 🏗 系统架构

### 6 层模块化架构

```
┌─────────────────────────────────┐
│ 应用层 (main.c)                 │  - 初始化和启动
├─────────────────────────────────┤
│ 控制循环 (control_loop.c)       │  - 100 Hz 闭环
│ - IMU 读取 + 融合               │  - PID 计算 + 电机控制
├─────────────────────────────────┤
│ 控制算法                        │  - PID、死区、融合
│ - pid_controller.c              │  - imu_fusion.c
├─────────────────────────────────┤
│ 执行器和传感器驱动             │  - 寄存器级操作
│ - pca9685_driver.c              │  - mpu6050_driver.c
│ - motor_controller.c            │
├─────────────────────────────────┤
│ I2C 底层驱动 (i2c_driver.c)    │  - ESP-IDF I2C API
├─────────────────────────────────┤
│ 硬件 (config.h)                │  - GPIO、I2C、地址
└─────────────────────────────────┘
```

### 控制流程

```
[100 Hz 循环开始]
       ↓
[读取 IMU1 和 IMU2]
       ├─ ACCEL_X/Y/Z (加速度)
       ├─ GYRO_X/Y/Z (陀螺仪)
       └─ TEMP (温度)
       ↓
[轴反演校正]
       └─ 根据 IMUx_INVERT_* 反转
       ↓
[双 IMU 融合]
       ├─ 加速度互补 (ALPHA_ACC=0.98)
       ├─ 陀螺仪加权 (基于噪声方差)
       ├─ 四元数 Mahony 积分
       └─ 输出: Roll, Pitch, Yaw
       ↓
[死区处理]
       └─ |Roll| < 1.0° → 0; 1.0-3.0° → 平滑; ≥3.0° → 直接
       ↓
[PID 控制计算]
       ├─ 目标角 = 0°
       ├─ 比例项 = 20.0 * (0 - Roll)
       ├─ 积分项 = 1.0 * ∫error
       └─ 输出: 力矩 (脉宽差)
       ↓
[电机差分驱动]
       ├─ pulse_left  = 1500 + output
       ├─ pulse_right = 1500 - output
       └─ 通过 PCA9685 发送 PWM
       ↓
[等待下一个周期 (10 ms)]
```

---

## ❓ 常见问题

### Q1: 烧录时提示 "Device not found"

**原因**: 串口未识别或驱动程序缺失

**解决**:
```
1. 用 USB 线连接 ESP32-S3，并确保数据线接触良好
2. 检查设备管理器中是否显示 "USB Serial Device" 或 "COM3/COM4"
   - 如果显示"未知设备"，下载 CP2102 驱动程序
3. 在 VS Code 中选择正确的串口 (通常 COM3)
4. 重试烧录
```

### Q2: 编译失败，提示 "IDF_PATH not found"

**原因**: ESP-IDF 环境变量未配置

**解决**:
```
1. 按 Ctrl+Shift+P
2. 输入 "Configure ESP-IDF Extension"
3. 选择 "Install ESP-IDF"（会自动下载）
   或选择已有的 ESP-IDF 路径
4. 等待配置完成后重试编译
```

### Q3: 监控日志中看不到系统输出

**原因**: 波特率错误或串口选择错误

**解决**:
```
1. 确保选择了正确的 COM 端口
2. 波特率应为 115200 (通常自动识别)
3. 重启 VS Code 或断开/重新连接 USB
4. 按 ESP32 上的 reset 按钮
```

### Q4: I2C 扫描找不到外围设备

**原因**: 硬件连接问题或 I2C 地址冲突

**解决**:
```
硬件检查:
  □ 检查 I2C 接线是否正确 (SDA/SCL/GND/VCC)
  □ 检查上拉电阻 (通常 4.7kΩ)
  □ 用万用表测量 I2C 信号 (应有 3.3V 上拉)

地址检查:
  □ IMU1: AD0 接 GND (地址 0x68)
  □ IMU2: AD0 接 VCC (地址 0x69)
  □ PCA9685: 默认地址 0x40

软件检查:
  □ 确认 config.h 中地址定义正确
  □ 重新编译烧录
```

### Q5: 船体不稳定，持续振荡

**原因**: PID 参数不当或传感器校准偏差

**解决**:
```
步骤 1: 查看日志中的陀螺仪零偏
  Gyro bias: x=0.05, y=0.02, z=-0.01
  
  如果零偏过大 (>0.5)，则校准可能有问题
  → 重新运行校准或增加样本数

步骤 2: 调整 PID 参数
  如果振荡: Kp 20.0 → 15.0
  如果不回正: Ki 1.0 → 2.0

步骤 3: 检查死区
  将 ANGLE_DEADZONE 从 1.0° 改为 0.5°
```

### Q6: 电机没有响应

**原因**: PWM 驱动未初始化或脉宽超出范围

**解决**:
```
1. 检查日志中 "PCA9685 @ 0x40 initialized" 是否出现
2. 检查 config.h 中的脉宽范围:
   MIN_PULSE: 1000, MAX_PULSE: 2000, BASE_PULSE: 1500
3. 检查电机连接到了正确的 PCA9685 通道:
   - 通道 0: 左推进器
   - 通道 1: 右推进器
4. 用示波器测量 PCA9685 输出是否有 PWM 信号
```

---

## 🎚 参数调整指南

### 情景 1: 船体响应太慢

**问题**: 受小扰动时，船体需要很长时间才能恢复

**调整**:
```c
// config.h
#define PID_KP              25.0f          // ↑ 增大比例增益
#define PID_KI              1.5f           // ↑ 增大积分增益
```

### 情景 2: 船体振荡

**问题**: 持续"摇晃"，无法稳定

**调整**:
```c
// config.h
#define PID_KP              15.0f          // ↓ 减小比例增益
#define ANGLE_DEADZONE      1.5f           // ↑ 增大死区
#define ANGLE_DEADZONE_SOFT 4.0f           // ↑ 增大软死区
```

### 情景 3: 偏离不能回正

**问题**: 船体一直偏向一侧

**调整**:
```c
// config.h
#define PID_KI              2.5f           // ↑ 增大积分增益
```

### 情景 4: 对高频噪声敏感

**问题**: 看到很多小的抖动

**调整**:
```c
// config.h
#define ALPHA_EMA           0.1f           // ↓ 增强低通滤波
#define ALPHA_ACC           0.95f          // ↓ 降低加速度权重
```

---

## 📊 参数参考表

### PID 参数速查

| 场景 | Kp | Ki | Kd | 备注 |
|------|----|----|----|----|
| 默认（推荐） | 20.0 | 1.0 | 0.0 | 通用配置 |
| 快速响应 | 25.0 | 1.5 | 0.0 | 高灵敏度 |
| 平缓响应 | 15.0 | 0.5 | 0.0 | 低灵敏度 |
| 激进 | 30.0 | 2.0 | 0.0 | 快速恢复 |
| 保守 | 10.0 | 0.2 | 0.0 | 防过度振荡 |

### 死区参数速查

| 场景 | 核心死区 | 软死区 | 备注 |
|------|---------|--------|------|
| 默认 | 1.0° | 3.0° | 推荐 |
| 敏感 | 0.5° | 2.0° | 快速响应 |
| 钝感 | 2.0° | 5.0° | 平缓响应 |
| 无死区 | 0.0° | 0.0° | 测试用 |

---

## 📞 获取帮助

### 查看源代码注释

每个文件都有详细的中文注释：
```c
/**
 * @file imu_fusion.c
 * @brief 双 IMU 融合算法
 * 
 * 步骤:
 * 1. 加速度互补融合
 * 2. 陀螺仪方差加权
 * 3. Mahony 四元数更新
 */
```

### 查看函数文档

所有重要函数都有 Doxygen 格式的文档：
```c
/**
 * @brief PID 控制器更新
 * 
 * @param pid PID 实例指针
 * @param setpoint 目标值
 * @param measured 测量值
 * @param omega_filtered 角速度
 * @return PID 输出结果
 */
float pid_update(pid_controller_t *pid, float setpoint, float measured, float omega_filtered);
```

### 问题排查清单

遇到问题时按顺序检查：

- [ ] 硬件是否正确连接?
- [ ] 驱动程序是否安装?
- [ ] ESP-IDF 是否配置正确?
- [ ] 配置参数是否合理?
- [ ] 日志中是否有错误信息?
- [ ] 是否需要重新校准?

---

## 📈 系统性能指标

| 指标 | 数值 |
|------|------|
| 控制频率 | 100 Hz (10 ms 周期) |
| IMU 采样频率 | 125 Hz (内部) |
| 总系统延迟 | 5-8 ms |
| 定时精度 | ±1-2 ms |
| 融合算法 | Mahony 四元数 |
| I2C 速度 | 400 kHz |
| PWM 频率 | 50 Hz |

---

## 🔐 安全提示

⚠️ **使用此系统时请注意**:

1. **紧急停止**: 系统在轨道内运行时，任何时刻按下 ESP32 的 RST 按钮可执行紧急停止
2. **脉宽限制**: PWM 脉宽被硬件限制在 1000-2000 μs 范围
3. **电源管理**: 确保电源供电充足，否则 I2C 通信可能失败
4. **温度监控**: MPU6050 内置温度传感器，温度过高时会影响精度

---

## 📝 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| 1.0 | 2026-04-14 | 初始版本，完整的 C++/ESP-IDF 实现 |

---

## 📄 许可证

本项目遵循 MIT 许可证。

---

**需要更多帮助?** 
- 查看代码中的注释 (都是中文)
- 参考 `config.h` 中的参数说明
- 检查编译输出和日志信息

**最后更新**: 2026-04-14
