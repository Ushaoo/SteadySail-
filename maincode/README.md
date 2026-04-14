# 🚤 SteadySail ESP32-S3 自适应平衡系统

**完整的嵌入式自动稳定与推进控制项目**  
*从 MicroPython 迁移到 ESP-IDF C++ 版本*

---

## 📖 目录

- [系统概述](#系统概述)
- [项目结构](#项目结构)
- [硬件要求](#硬件要求)
- [构建与部署](#构建与部署)
- [配置指南](#配置指南)
- [使用说明](#使用说明)
- [技术文档](#技术文档)
- [常见问题](#常见问题)
- [文件对应关系](#文件对应关系)

---

## 🎯 系统概述

### 核心功能

SteadySail 是一个小型船体的**自动平衡与推进系统**，通过双 IMU 传感器融合、实时 PID 控制和差分推进，实现：

1. **自动平衡** - 保持船体横滚角在 ±3° 以内
2. **双 IMU 融合** - 通过四元数算法融合两个 MPU6050 的数据，提高精度
3. **差分推进控制** - 独立控制左右推进器，实现转向和加速
4. **推进器旋转** - 可选的旋转舵机，改变推进方向
5. **实时 100 Hz 控制** - 通过 FreeRTOS 任务实现稳定的闭环控制

### 系统架构

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        ┌─────────────────────┐
        │   ESP32-S3 微控制器   │
        │   (主控制器)        │
        └────────┬────────────┘
                 │
        ┌────────┼────────┐
        │        │        │
    ┌───▼──┐ ┌──▼───┐ ┌──▼───┐
    │ I2C  │ │UART  │ │GPIO  │
    │总线  │ │通信  │ │中断  │
    └───┬──┘ └──────┘ └──────┘
        │
    ┌───┴─────────────────────┐
    │                         │
┌──▼──┐  ┌──────┐  ┌────────┐
│IMU1 │  │IMU2  │  │PCA9685 │
│     │  │      │  │  PWM   │
│0x68 │  │0x69  │  │ 0x40   │
└─────┘  └──────┘  └───┬────┘
                       │
                  ┌────┴────────────┐
                  │                 │
              ┌───▼──┐         ┌───▼──┐
              │ 左推进 │         │ 右推进 │
              │  器   │         │  器   │
              └───┬──┘         └───┬──┘
                  │                │
              ┌───▼──┐         ┌───▼──┐
              │旋转   │         │旋转   │
              │舵机   │         │舵机   │
              └───────┘         └───────┘
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

### 核心算法

#### 1. 双 IMU 融合 (DualIMUFusion)

- **加速度互补滤波**：占比 98% IMU1, 2% IMU2
- **陀螺仪动态加权**：根据噪声方差自动选择更可信的 IMU
- **四元数积分**：Mahony 算法，支持高速旋转
- **输出**：Roll, Pitch, Yaw 欧拉角

#### 2. 2 自由度 PID 控制 (PID2DOF)

```
控制输出 = Kp*(b*setpoint - measured) 
         + Ki*integral(error) 
         + Kd*(c*setpoint' - measured')

默认参数：
- Kp = 20.0   (比例增益，主要补偿)
- Ki = 1.0    (积分增益，消除稳态误差)
- Kd = 0.0    (微分增益，阻尼效果)
- b = 0.8     (比例权重，减少超调)
- c = 0.0     (微分权重，禁用 setpoint kick)
```

#### 3. 死区处理 (Deadzone)

平滑的非线性死区，防止小扰动导致频繁调整：

```
|error| < 1°       → 输出 = 0 (核心死区)
1° < |error| < 3°  → 平滑过渡
|error| >= 3°      → 输出 = error (软死区)
```

#### 4. 力分配 (Force Allocation)

当推进器旋转时，自动调整平衡控制的输出，补偿推进力方向的变化。

---

## 📁 项目结构

```
maincode/
│
├── CMakeLists.txt                    # 根项目配置 (ESP-IDF)
│
└── main/
    ├── CMakeLists.txt                # 主项目编译配置
    ├── config.h                      # 全局参数配置 ⭐ 主要调参文件
    ├── main.c                        # 入口：系统初始化、任务启动
    ├── i2c_driver.h/c                # I2C 底层驱动 (读写、扫描)
    │
    ├── sensors/ (传感器驱动)
    │   ├── mpu6050_driver.h/c         # IMU 驱动：寄存器操作、数据转换
    │   └── imu_fusion.h/c             # 双 IMU 融合：四元数、加速度互补
    │
    ├── actuators/ (执行器驱动)
    │   ├── pca9685_driver.h/c         # PWM 驱动：频率设置、脉宽控制
    │   └── motor_controller.h/c       # 电机控制：推进、旋转、安全停止
    │
    ├── control/ (控制算法)
    │   ├── pid_controller.h/c         # PID 控制器：2DOF 算法
    │   ├── deadzone.h/c               # 死区处理：平滑过渡
    │   ├── force_allocation.h/c       # 力分配：旋转时的补偿
    │   └── control_loop.h/c           # 主控制循环：100 Hz 闭环任务
    │
    ├── io/ (通信接口)
    │   └── uart_command.h/c           # UART 命令接口：参数调节、实时监控
    │
    └── utils/ (工具函数)
        └── (数学函数、滤波器等)
```

---

## 🛠️ 硬件要求

### 必需

| 组件 | 型号 | I2C 地址 | 说明 |
|------|------|---------|------|
| 主控 | ESP32-S3 | - | 微控制器 |
| IMU (主) | MPU6050 | 0x68 | 9-DoF 传感器 |
| IMU (从) | MPU6050 | 0x69 | 冗余融合 |
| PWM 驱动 | PCA9685 | 0x40 | 16 通道 PWM |

### 连接

```
ESP32-S3            MPU6050(1)    MPU6050(2)    PCA9685
─────────           ──────────    ──────────    ───────
GPIO21 (SDA) ──────→ SDA           SDA       →   SDA
GPIO22 (SCL) ──────→ SCL           SCL       →   SCL
GND ──────────────→ GND           GND       →   GND
3V3 ──────────────→ VCC           VCC       →   VCC

I2C 频率: 400 kHz
线路: 100 mA 最大拉动电流
```

### 电机连接

```
PCA9685 PWM 通道配置
┌─────────────┬──────────────┐
│ 通道        │ 功能         │
├─────────────┼──────────────┤
│ 0           │ 左推进器脉宽 │
│ 1           │ 右推进器脉宽 │
│ 2           │ 左旋转舵机   │
│ 3           │ 右旋转舵机   │
│ 4-15        │ 预留         │
└─────────────┴──────────────┘

PWM 信号:
- 频率: 50 Hz
- 脉宽范围: 1000-2000 μs
- 中立: 1500 μs
```

---

## 🔧 构建与部署

### 前置条件

1. **ESP-IDF** v5.0+ 已安装
2. **CMake** 3.22+
3. **Python 3.8+**

### 构建步骤

```bash
# 1. 进入项目目录
cd maincode

# 2. 配置目标设备
idf.py set-target esp32s3

# 3. 构建项目
idf.py build

# 4. 刷写固件到设备
idf.py -p COM3 flash monitor              # Windows
idf.py -p /dev/ttyUSB0 flash monitor       # Linux

# 5. 打开日志监视器（已在部署过程中自动启动）
idf.py -p COM3 monitor
```

### 清理和重建

```bash
# 完全清理
idf.py fullclean

# 只清理 menuconfig 缓存
rm -rf sdkconfig

# 重新配置后构建
idf.py menuconfig
idf.py build
```

---

## ⚙️ 配置指南

### 主配置文件：`config.h`

所有可配置参数集中在此文件。**修改参数后需要重新编译**。

#### I2C 硬件配置

```c
#define I2C_PORT            I2C_NUM_0       // I2C 端口
#define I2C_SDA_PIN         21              // SDA 引脚
#define I2C_SCL_PIN         22              // SCL 引脚
#define I2C_FREQ_HZ         400000          // I2C 频率
```

#### 控制周期

```c
#define CONTROL_FREQ_HZ     100             // 控制频率 (100 Hz)
#define CONTROL_DT_MS       10              // 计算周期 (10 ms)
#define CONTROL_DT_S        0.01f           // 秒单位
```

#### IMU 轴向配置

如果 IMU 安装方向与默认不同，调整反转参数：

```c
#define IMU1_INVERT_X       1               // 1=反向 X 轴, 0=保持
#define IMU1_INVERT_Y       1               // 反向 Y 轴
#define IMU1_INVERT_Z       0               // 保持 Z 轴

#define IMU2_INVERT_X       0               // IMU2 轴向配置
#define IMU2_INVERT_Y       0
#define IMU2_INVERT_Z       0
```

#### PID 参数调节 ⭐ 最重要

```c
#define PID_KP              20.0f     // 比例增益 (响应速度)
#define PID_KI              1.0f      // 积分增益 (消除偏差)
#define PID_KD              0.0f      // 微分增益 (通常为 0)
#define PID_B               0.8f      // 比例权重 (减少超调)
#define PID_C               0.0f      // 微分权重 (通常为 0)
```

**调参指南：**
- **增加 Kp** → 响应更快，但容易过度补偿
- **增加 Ki** → 减少稳态误差，但积分慢
- **Kd 通常为 0** → 除非系统严重震荡

#### 死区参数

```c
#define ANGLE_DEADZONE      1.0f      // 角度死区核心 (度)
#define ANGLE_DEADZONE_SOFT 3.0f      // 软边界 (平滑过渡范围)
#define OMEGA_DEADZONE_SOFT 6.0f      // 角速度死区 (deg/s)
```

#### 融合算法参数

```c
#define ALPHA_ACC           0.98f     // 加速度互补滤波系数
                                      // 越大越信 IMU1，越小越信 IMU2
#define WEIGHT_DYNAMIC      0.8f      // 动态时的权重偏向
#define ALPHA_EMA           0.15f     // 低通滤波系数 (0.1-0.3)
```

#### 推进器配置

```c
#define BASE_PULSE          1500      // 中立脉宽 (μs)
#define MIN_PULSE           1000      // 最小脉宽
#define MAX_PULSE           2000      // 最大脉宽
#define THRUST_SCALE        0.55f     // 推力缩放系数
```

#### 可选功能开关

```c
#define ENABLE_SERVO_ROTATION    1     // 1=启用旋转舵机
#define ENABLE_FORCE_ALLOCATION  1     // 1=启用力分配补偿
```

---

## 💻 使用说明

### 启动系统

1. **编译并烧写固件**（见上节）
2. **连接 USB 监视器**，查看日志：

```
I (100) MAIN: ============================================
I (101) MAIN: SteadySail - Boat Stabilization System
I (102) MAIN: ESP32-S3 Firmware v1.0
I (103) MAIN: ============================================
I (110) MAIN: === SteadySail ESP32-S3 System Initialization ===
I (120) MAIN: Initializing I2C (SDA=21, SCL=22, Freq=400000 Hz)
I (140) I2C_DRIVER: I2C initialized: port=0, SDA=21, SCL=22, freq=400000 Hz
I (150) MAIN: Scanning I2C devices...
I (180) I2C_DRIVER: Found device at address 0x68
I (190) I2C_DRIVER: Found device at address 0x69
I (200) I2C_DRIVER: Found device at address 0x40
I (210) I2C_DRIVER: Total devices found: 3
```

3. **验证硬件连接**
   - 看到 0x68, 0x69, 0x40 说明 I2C 连接正常
   - 如果缺少设备，检查接线和 I2C 地址

### 角度零点校准 ⭐ **新增功能**

系统支持**自动角度零点校准**，用于消除磁铁位置导致的角度偏差。

#### 工作原理

```
示例：
启动时推进器实际垂直（90°），但 IMU 读数显示 95°
系统会自动记录偏差: offset = 95° - 90° = 5°
后续所有读数都减去这个偏差：
  - IMU 读: 100° → 校准后: 100° - 5° = 95° ✓
```

#### 校准流程

1. **打开日志监视器**

   ```bash
   idf.py monitor
   ```

2. **启动系统**，会看到：

   ```
   I (xxx) CONTROL_LOOP: Angle calibration ENABLED - waiting 3 seconds for user to set vertical position...
   ```

3. **在这 3 秒内**：
   - 让推进器保持**完全垂直**（90°）
   - 不要移动或晃动系统
   - 等待倒计时完成

4. **系统自动校准**：

   ```
   I (xxx) CONTROL_LOOP: Starting angle calibration - sampling 100 readings...
   I (xxx) CONTROL_LOOP: Calibrating... 10/100 samples
   I (xxx) CONTROL_LOOP: Calibrating... 20/100 samples
   ...
   I (xxx) CONTROL_LOOP: Angle calibration completed! Offset = 5.23°
   ```

5. **校准完成** ✓

   系统开始正常运行，所有角度读数都自动补偿

#### 配置校准参数

在 `config.h` 中修改：

```c
#define ENABLE_ANGLE_CALIBRATION    1       // 1=启用校准, 0=禁用
#define CALIBRATION_WAIT_TIME       3000    // 等待时间 (ms)
#define CALIBRATION_SAMPLES_COUNT   100     // 采样数（更多=更精准）
```

**调参建议：**
- **CALIBRATION_WAIT_TIME** - 增加值给用户更多时间调整
- **CALIBRATION_SAMPLES_COUNT** - 增加值提高精度（但启动变慢）

#### 手动校准（无需重编译）

如果需要重新校准，可以通过 UART 命令：

```bash
# 设置角度偏差为 5.0°
set_angle_offset 5.0

# 查询当前偏差
get_angle_offset

# 查询校准状态
get_calib_status
```

---

### IMU 零偏校准

系统在启动时会自动校准 IMU 的加速度和陀螺仪零偏（需要 5-10 秒，**设备必须保持完全静止**）。这与上面的**角度零点校准**不同：

- **IMU 零偏校准**（自动）：消除传感器本身的偏差
- **角度零点校准**（自动）：消除安装位置导致的角度偏差

### 实时监控

通过 UART 命令接口（可选）监控和调节参数：

```bash
# 使用串口工具连接到 ESP32-S3 (波特率 115200)
# 发送命令格式: command param1 param2 ...

get_roll           # 获取当前横滚角
get_pid            # 获取当前 PID 参数
set_kp 25.0        # 设置 Kp = 25.0
set_motor 1500 1500  # 设置两个电机脉宽
...
```

---

## 📚 技术文档

### 关键模块详解

#### 1. I2C 驱动 (`i2c_driver.c`)

**功能**：底层 I2C 读写，所有传感器/执行器都通过它通信

**关键函数**：
- `i2c_driver_init()` - 初始化 I2C
- `i2c_read_bytes()` / `i2c_write_bytes()` - 读写数据
- `i2c_scan_devices()` - 扫描总线

**特点**：
- 400 kHz 标准 I2C 速度
- 1 ms 超时（可调）
- 自动错误检测

#### 2. MPU6050 驱动 (`sensors/mpu6050_driver.c`)

**功能**：读取和处理 IMU 数据

**关键函数**：
- `mpu6050_init()` - 初始化传感器
- `mpu6050_read_all_raw()` - 读取原始数据
- `mpu6050_read_accel_g()` / `mpu6050_read_gyro_dps()` - 读取缩放数据

**输出范围**：
- 加速度：±2g（16384 LSB/g）
- 陀螺仪：±250°/s（131 LSB/deg/s）

#### 3. 双 IMU 融合 (`sensors/imu_fusion.c`)

**算法**：
1. 加速度互补滤波（ALPHA_ACC = 0.98）
2. 陀螺仪方差估计
3. 动态加权融合
4. 四元数积分（Mahony）
5. 欧拉角输出

**输入**：两个 IMU 的加速度和陀螺仪数据  
**输出**：Roll, Pitch, Yaw 欧拉角

#### 4. PID 控制器 (`control/pid_controller.c`)

**算法**：2 DOF PID

```
output = Kp * (b*sp - meas)
       + Ki * ∫error dt
       + Kd * (c*sp' - meas')
```

**特点**：
- Setpoint weighting (b=0.8) 减少超调
- 积分累计可重置

#### 5. 电机控制 (`actuators/motor_controller.c`)

**功能**：
- 推进器脉宽控制（1000-2000 μs）
- 旋转舵机角度映射（-45 ~ +45°）
- 紧急停止

**脉宽映射**：
```
推进力 = (pulse - 1500) / 500 * 100%
例：1500 μs = 0％, 2000 μs = +100%, 1000 μs = -100%
```

#### 6. 主控制循环 (`control/control_loop.c`)

**流程** (每 10 ms 执行一次)：

```
1. 读取 IMU1 和 IMU2 数据 (6ms)
   ↓
2. 双 IMU 融合 (1ms)
   ↓
3. 低通滤波 (0.1ms)
   ↓
4. PID 计算 (0.5ms)
   ↓
5. 死区处理 (0.1ms)
   ↓
6. 力分配补偿 (0.5ms)
   ↓
7. 设置电机脉宽 (1.8ms)
   ↓
8. 等待到 10 ms 周期 (循环延迟)
```

**性能指标**：
- 周期：10 ms (100 Hz)
- 实际执行时间：~8-9 ms
- CPU 占用：80-90% (单核)

---

## ❓ 常见问题

### Q: I2C 设备扫描不到？

**A:** 检查以下几点：
1. USB 串口是否正确连接
2. I2C 接线（SDA/SCL）是否正确
3. 上拉电阻是否已连接（4.7kΩ）
4. 设备地址是否正确（MPU6050: 0x68/0x69, PCA9685: 0x40）
5. 尝试 `idf.py set-target esp32s3 && idf.py menuconfig` 检查 I2C 引脚配置

### Q: 平衡不稳定，船体频繁振荡？

**A:** 调节 PID 参数：
1. **降低 Kp**（20→15）减少激进响应
2. **增加 Ki**（1.0→2.0）消除稳态偏差
3. **增加死区**（3.0→5.0）减少高频振荡

### Q: IMU 数据不正确？

**A:** IMU 校准/轴向问题：
1. 确保校准时设备**完全静止**
2. 检查 `config.h` 中的轴向反转设置（IMU1/IMU2_INVERT_X/Y/Z）
3. 查看日志中的加速度值，应接近 1.0g (9.81 m/s²)

### Q: 编译错误？

**A:** 常见原因：
1. ESP-IDF 版本过旧（需要 v5.0+）
2. CMake 路径错误，检查 `IDF_PATH` 环境变量
3. 代码有 TODO，某些函数未完整实现，可以先注释掉相关代码

### Q: 烧写失败？

**A:** 尝试：
```bash
# 1. 擦除 flash
idf.py erase-flash

# 2. 使用更慢的波特率烧写
idf.py -p COM3 -b 115200 flash monitor

# 3. 检查驱动程序（Windows）
```

### Q: 角度校准后仍有偏差？

**A:** 检查以下几点：
1. **校准时未保持垂直** - 重启系统重新校准，确保推进器完全垂直
2. **校准采样数太少** - 增加 `CALIBRATION_SAMPLES_COUNT`（100→200）
3. **系统晃动** - 确保在等待 3 秒间内系统完全静止
4. **磁铁位置变了** - 如果移动了磁铁，需要重新校准

### Q: 如何禁用自动角度校准？

**A:** 修改 `config.h`：
```c
#define ENABLE_ANGLE_CALIBRATION    0       // 禁用自动校准
```
然后重编译。

### Q: 校准过程中系统没有反应？

**A:** 这是正常的！系统在等待 3 秒让用户调整位置，然后采集 100 个样本（需要 1 秒），所以总共需要 4 秒。只需耐心等待日志中出现"Angle calibration completed"。

---

## 📊 文件对应关系

从原始版本迁移到新版本的对应关系：

| 功能 | 树莓派版本 | MicroPython版本 | C++ 版本 |
|------|----------|-----------------|---------|
| **核心控制** | `feedforward_dual_imu.py` | `esp32_new/main.py` | `main/control_loop.c` |
| **IMU 融合** | `dual_imu.py` | `esp32_new/main.py` | `sensors/imu_fusion.c` |
| **PID 控制** | `feedforward_dual_imu.py` (PID2DOF) | `esp32_new/main.py` (PID2DOF) | `control/pid_controller.c` |
| **死区处理** | `feedforward_dual_imu.py` (apply_deadzone_smooth) | `esp32_new/main.py` (apply_deadzone_smooth) | `control/deadzone.c` |
| **PWM 驱动** | `motor_test.py` (PCA9685) | `esp32_new/lib/pca9685.py` | `actuators/pca9685_driver.c` |
| **IMU 驱动** | `mpu6050.py` (库) | `esp32_new/lib/mpu6050.py` | `sensors/mpu6050_driver.c` |
| **电机控制** | `feedforward_dual_imu.py` (SafeMotorController) | `esp32_new/main.py` | `actuators/motor_controller.c` |

---

## 🚀 后续开发

### 已完成

- ✅ 完整的项目结构和模块化设计
- ✅ I2C 底层驱动
- ✅ MPU6050 驱动框架
- ✅ PCA9685 PWM 驱动框架
- ✅ 双 IMU 融合算法框架
- ✅ 2DOF PID 控制器
- ✅ 死区处理函数
- ✅ 电机控制逻辑

### 待完成（标有 TODO）

- [ ] 完整的 MPU6050 寄存器读写实现
- [ ] 完整的 PCA9685 频率和脉宽设置实现
- [ ] 双 IMU 融合的完整四元数算法
- [ ] 力分配算法完整实现
- [ ] UART 命令接口
- [ ] 数据日志和远程监控

### 可选改进

- 低功耗模式（减少采样率）
- Wi-Fi 远程监控（需要 WiFi 模块）
- SD 卡数据记录
- 神经网络优化参数
- 多水道和风浪仿真

---

## 📞 支持

如有问题，检查以下资源：

1. **日志输出** - 通过 `idf.py monitor` 查看
2. **代码注释** - 每个文件都有详细的函数文档
3. **参考版本** - 查看 `raspberry_pi_backup/` 和 `esp32_new/` 中的原始实现
4. **ESP-IDF 官方文档** - https://docs.espressif.com/projects/esp-idf/

---

## 📝 版本记录

| 版本 | 日期 | 描述 |
|------|------|------|
| v1.0 | 2026-04-14 | 项目框架完成，设计和文档就绪 |

---

**SteadySail © 2026 - 开放式船体稳定系统**
