# 调试开关配置指南

## 概述

SteadySail 系统提供了两个编译时调试开关，用于分开调试舵机旋转和力分配功能。

---

## 参数配置

在 `main/config.h` 中配置：

```c
/* ==================== 调试开关 ==================== */
#define ENABLE_SERVO_ROTATION       1   // 1 = 启用, 0 = 禁用
#define ENABLE_FORCE_ALLOCATION     1   // 1 = 启用, 0 = 禁用
```

---

## 参数说明

### 1. `ENABLE_SERVO_ROTATION` (舵机旋转功能)

**作用**：控制是否启用通过 UART 终端命令改变推进器角度的功能

| 值 | 状态 | 行为 |
|----|------|-----|
| `1` | 启用 | ✅ UART 接收线程启动，可通过终端输入角度值 (0-180°) 控制推进器旋转 |
| `0` | 禁用 | ⛔ UART 接收线程不启动，推进器保持初始角度 (90°，竖直向下) |

**调试用途**：
- 独立测试舵机机械系统
- 验证 UART 通信和命令解析
- 不受 PID 和力分配影响，纯粹测试舵机硬件

---

### 2. `ENABLE_FORCE_ALLOCATION` (力分配功能)

**作用**：控制是否启用根据推进器角度进行动态力分配的功能

| 值 | 状态 | 行为 |
|----|------|-----|
| `1` | 启用 | ✅ PID 输出通过力分配算法映射到左右推进器（考虑推进器角度） |
| `0` | 禁用 | ⛔ PID 输出直接作为差分推进力，推进器保持竖直向下 (90°) |

**调试用途**：
- 验证 PID 控制器（不涉及复杂的力分配计算）
- 测试基础平衡功能
- 简化调试链路

---

## 调试场景

### 场景 1：纯 PID 平衡调试
```c
#define ENABLE_SERVO_ROTATION       0   // 禁用舵机
#define ENABLE_FORCE_ALLOCATION     0   // 禁用力分配
```

**特点**：
- 推进器固定竖直向下 (90°)
- PID 输出直接控制左右推进器差分
- 最简单的控制模式，用于快速验证 PID 参数

**观察**：
```
Loops=500 | Attitude: roll=-0.5° | PID: Δ=12.3 | Thruster: 90.0° | Servo: 0.0°
```

---

### 场景 2：舵机机械验证
```c
#define ENABLE_SERVO_ROTATION       1   // 启用舵机
#define ENABLE_FORCE_ALLOCATION     0   // 禁用力分配
```

**特点**：
- 可通过终端输入改变推进器角度
- PID 输出不影响推进器方向
- 用于测试舵机旋转机械

**操作**：
```bash
# 终端输入
90      # 推进器竖直向下
0       # 推进器水平向前
180     # 推进器水平向后
45      # 推进器 45° 角
```

**观察**：
```
Loops=500 | Attitude: roll=-2.1° | PID: Δ=25.6 | Thruster: 45.0° | Servo: -22.5°
```

---

### 场景 3：完整功能验证（推荐生产模式）
```c
#define ENABLE_SERVO_ROTATION       1   // 启用舵机
#define ENABLE_FORCE_ALLOCATION     1   // 启用力分配
```

**特点**：
- 完整的自适应平衡系统
- 推进器角度可通过 UART 改变
- PID 输出通过力分配适应推进器角度变化
- 系统自动维持船体平衡并推进

**操作**：
```bash
# 终端输入改变推进器角度
90      # 推进器竖直向下 + 直进平衡
0       # 推进器向前 + 自动平衡
180     # 推进器向后 + 自动平衡
```

**观察**：
```
Loops=500 | Attitude: roll=0.2° | PID: Δ=5.3 | Thruster: 0.0° | Servo: -45.0°
# 推进器旋转至向前方向，PID 自动调整以维持平衡
```

---

## 初始化日志示例

### 启用两个功能
```
I (720) CONTROL_LOOP: Initializing control loop...
I (725) CONTROL_LOOP: DEBUG SWITCHES:
I (730) CONTROL_LOOP:   - ENABLE_SERVO_ROTATION: 1
I (735) CONTROL_LOOP:   - ENABLE_FORCE_ALLOCATION: 1
...
I (850) CONTROL_LOOP: Servo rotation ENABLED - Initializing UART command receiver...
...
```

### 禁用舵机旋转
```
I (720) CONTROL_LOOP: Initializing control loop...
I (725) CONTROL_LOOP: DEBUG SWITCHES:
I (730) CONTROL_LOOP:   - ENABLE_SERVO_ROTATION: 0
I (735) CONTROL_LOOP:   - ENABLE_FORCE_ALLOCATION: 1
...
I (850) CONTROL_LOOP: Servo rotation DISABLED - UART command receiver not started
...
```

---

## 调试流程建议

### 第一步：验证硬件
1. 设置 `ENABLE_SERVO_ROTATION=1, ENABLE_FORCE_ALLOCATION=0`
2. 通过终端输入控制舵机旋转
3. 观察舵机与电机响应

### 第二步：调试 PID
1. 设置 `ENABLE_SERVO_ROTATION=0, ENABLE_FORCE_ALLOCATION=0`
2. 调整 `PID_KP`, `PID_KI`, `PID_KD` 参数
3. 观察 Roll 角是否稳定在 ±3° 内

### 第三步：集成测试
1. 设置 `ENABLE_SERVO_ROTATION=1, ENABLE_FORCE_ALLOCATION=1`
2. 通过终端改变推进器角度
3. 观察系统在不同角度下的平衡效果

---

## 编译命令

修改参数后重新编译：

```bash
cd d:\SteadySail--1\maincode
idf.py clean
idf.py build
idf.py flash monitor
```

---

## 性能影响

| 开关组合 | CPU 负载 | 内存占用 | 说明 |
|---------|---------|---------|-----|
| 0, 0 | 低 | ~5KB | 仅 IMU + PID，最轻 |
| 1, 0 | 中 | ~8KB | +UART 接收任务 |
| 0, 1 | 中 | ~8KB | +力分配计算 |
| 1, 1 | 中 | ~12KB | 完整功能 |

---

## 故障排查

### 推进器不旋转
- 检查 `ENABLE_SERVO_ROTATION` 是否为 1
- 检查终端输入格式（应为纯数字 + 回车）
- 查看日志中是否有 UART 错误

### Roll 角不稳定
- 检查 `ENABLE_FORCE_ALLOCATION` 是否为 1
- 尝试调整 PID 参数
- 检查 IMU 校准是否正确

### 电机输出异常
- 检查 `ENABLE_FORCE_ALLOCATION=0` 时基础平衡是否工作
- 检查 PWM 驱动初始化是否成功
- 观察裸露的 PID 输出值是否在合理范围

