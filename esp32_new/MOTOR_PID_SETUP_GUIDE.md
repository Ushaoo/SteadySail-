

# 🔌 电机 PID 控制系统 - 完整设置指南

## 📋 目录
1. [烧录步骤](#烧录步骤)
2. [连线方案](#连线方案)
3. [单电机配置](#单电机配置说明)
4. [使用方法](#使用方法)
5. [测试和调试](#测试和调试)
6. [常见问题](#常见问题)

---

## 1️⃣ 烧录步骤

### 第一步：准备工具
```bash
# 安装必要的工具
pip install esptool pyserial mpremote

# 或使用 requirements.txt
pip install -r esp32_new/requirements.txt
```

### 第二步：烧录 MicroPython 固件

**获取 ESP32 COM 端口：**
- 连接 ESP32 到电脑
- Windows 设备管理器 → 端口 (COM 和 LPT) → 记录 COM 号 (如 COM3)

**擦除并烧录固件 (PowerShell 或 CMD)：**

⚠️ **重要提示：** 如果你的 ESP32 是 **ESP32-S3** 版本，用 `esp32-s3` 替换下面的 `esp32`

```powershell
# 1. 擦除 Flash 存储
python -m esptool --chip esp32-s3 --port COM3 erase-flash

# 2. 烧录最新 MicroPython 固件
# 下载地址: https://micropython.org/download/esp32-s3/
# 这里假设固件在 D:\Downloads\ESP32_S3-20240105-v1.22.1.bin
python -m esptool --chip esp32-s3 --port COM3 --baud 460800 write-flash -z 0x0 `
  "D:\SteadySail--1\ESP32_GENERIC_S3-20260406-v1.28.0.bin"

# 3. 验证烧录成功
python -m esptool --chip esp32-s3 --port COM3 flash-id
```

**如果是普通 ESP32** (非 S3)，改用：
```powershell
python -m esptool --chip esp32 --port COM3 erase-flash
python -m esptool --chip esp32 --port COM3 --baud 460800 write-flash -z 0x1000 `
  "D:\Downloads\esp32-20240105-v1.22.1.bin"
```

### 第三步：上传 Python 代码

**方式 A：使用 Thonny IDE (推荐，最简单)**

1. 下载安装 Thonny: https://thonny.org/
2. 打开 Thonny
3. 菜单 Tools → Options → Interpreter → MicroPython (ESP32)
4. 选择串口 COM3，波特率 115200
5. 在 Thonny 中打开文件并保存到设备：
   ```
   esp32_new/config.py          → 保存到 /config.py
   esp32_new/motor_pid_control.py → 保存到 /motor_pid_control.py
   esp32_new/lib/pca9685.py     → 保存到 /lib/pca9685.py (需要先创建 /lib 文件夹)
   ```

**方式 B：使用命令行 (mpremote)**

```powershell
# 连接设备
mpremote connect COM3

# 创建 lib 文件夹
mpremote mkdir :/lib

# 上传代码文件
mpremote cp esp32_new/config.py :/config.py
mpremote cp esp32_new/motor_pid_control.py :/motor_pid_control.py
mpremote cp esp32_new/lib/pca9685.py :/lib/pca9685.py

# 断开连接
mpremote disconnect
```

---

## 2️⃣ 连线方案

### 硬件连接图

```
ESP32 开发板 (30 pin)
┌──────────────────────────────────┐
│  USB  ②  ③  ④⑤⑥  ⑦  ⑧  ⑨  ⑩  │
│  ⑪    ⑫  ⑬  ⑭⑮⑯  ⑰  ⑱  ⑲  ⑳  │
│                                  │
└──────────────────────────────────┘

GPIO2  → PWM 电机输出 (50Hz)
GPIO4  → PWM 输入 (舵机反馈)
GPIO21 → I2C SDA
GPIO22 → I2C SCL
```

### 详细连线表

| 编号 | ESP32 PIN | 功能 | 目标设备 | 备注 |
|-----|----------|------|--------|------|
| 1 | GPIO2 | PWM 输出 | 伺服电机信号线 | 50Hz PWM，1000-2000μs |
| 2 | GPIO4 | PWM 输入 | 伺服电机反馈（可选） | 读取电机位置反馈 |
| 3 | GPIO21 | I2C SDA | PCA9685 SDA | 排线扩展板控制 |
| 4 | GPIO22 | I2C SCL | PCA9685 SCL | 排线扩展板控制 |
| 5 | 5V | 电源 | PCA9685 6 脚 | 需要稳定电源 |
| 6 | GND | 地线 | PCA9685 5 脚 | 共地 |
| 7 | 3.3V | 逻辑电压 | PCA9685 4 脚 | 可选 |

### 伺服电机接线 (标准 3 线)

```
伺服舵机线缆:
  ┌─────┐
  │ GND │ 棕色  → GND
  │ VCC │ 红色  → 5V (通常接 PCA9685)
  │ SIG │ 黄色  → GPIO2 (PWM 控制) 或 PCA9685
  └─────┘

反馈信号 (可选，用于闭环控制):
  黑色线 → GND
  红色线 → 5V
  紫色线 → GPIO4 (PWM 输入)
```

### 接线步骤

**1. 准备物品：**
- 1× ESP32 开发板
- 1× 伺服电机/推进器 (带反馈信号最佳)
- 1× PCA9685 PWM 扩展板 (可选，但推荐用于多个电机)
- 1× USB 数据线 (连接 ESP32 到电脑进行编程)
- 导线、杜邦线、焊接工具

**2. 接线顺序：**
```
A. 电源接线 (先做这个!)
   ├─ ESP32 GND (任意 GND 脚) → 电源 GND
   ├─ ESP32 5V (5V 脚) → 电源 5V
   └─ PCA9685 GND (5 脚) → 电源 GND
   
B. I2C 接线 (数据通信)
   ├─ ESP32 GPIO21 → PCA9685 SDA (2 脚)
   ├─ ESP32 GPIO22 → PCA9685 SCL (3 脚)
   └─ PCA9685 6 脚 → 5V
   
C. 电机信号接线 (控制输出)
   ├─ PCA9685 V+ (1 脚) → 5V (电机电源)
   └─ PCA9685 通道 2 → 伺服电机信号线 (黄色)
   
D. 反馈接线 (闭环检测，可选)
   └─ ESP32 GPIO4 → 伺服反馈信号线 (如有)
```

---

## 3️⃣ 单电机配置说明

由于你现在只连接一个电机，需要进行以下配置：

### 修改电机控制参数

编辑 `motor_pid_control.py`，修改以下部分：

#### 选项 A：只控制一个电机（推荐）

```python
# ==================== 修改部分 ====================
# 行号约 32-48

# 电机 PWM 配置 - 改为只使用一个电机
MOTOR_PWM_GPIO = 2              # PWM 输出引脚 (保持)
MOTOR_PWM_FREQ_HZ = 50          # 50Hz 伺服频率 (保持)
MOTOR_PWM_NEUTRAL = 1400        # 中立脉宽 (单电机中立值)
MOTOR_PWM_MAX = 1480            # 最大脉宽 (调整以适应你的电机)
MOTOR_PWM_MIN = 1320            # 最小脉宽 (调整以适应你的电机)

# 如果电机的中立点不是 1400μs，修改这个值
# 一般伺服电机中立点範圍: 1400-1500μs
# 推进器中立点範圍: 1500-1600μs
```

#### 选项 B：校准你的电机中立点

```python
# 运行这个测试代码找到电机的准确中立点
from motor_pid_control import MotorControl
motor = MotorControl(2)

# 逐步调整，找出电机不动的脉宽值
motor.set_duty(1400)  # 测试 1400μs
time.sleep(1)
motor.set_duty(1420)  # 测试 1420μs
time.sleep(1)
motor.set_duty(1380)  # 测试 1380μs
```

### 单电机 PID 参数调整

如果电机响应过快或过慢，调整这些参数：

```python
# ==================== PID 参数 ====================
# 行号约 55-58

PID_KP = 9.8   # 比例系数 - 增大使响应更快，减小使响应更平缓
PID_KI = 1.0   # 积分系数 - 用于消除稳态误差
PID_KD = 0.49  # 微分系数 - 用于阻尼，防止超调

# 建议调整顺序：
# 1. 先调 Kp (从 5.0 开始，逐步增加)
# 2. 再调 Kd (从 0.2 开始，逐步增加)
# 3. 最后调 Ki (通常保持在 0.5-2.0)
```

### 禁用双电机连接代码

程序默认兼容单电机。如果你想完全移除双电机代码，注释掉这部分：

```python
# 在 MotorPIDControlSystem.__init__ 中，
# 如果你只想使用一个电机，完整代码已经支持了
# 不需要修改任何东西！
```

---

## 4️⃣ 使用方法

### 运行程序

**方式 1：使用 Thonny IDE**
1. 打开 Thonny
2. 文件 → 打开 → `esp32_new/motor_pid_control.py`
3. 按 F5 运行或菜单 Run → Run

**方式 2：使用命令行重应加载**
```powershell
# 连接设备并进入 REPL
mpremote connect COM3 repl

# 在 REPL 中
>>> exec(open('motor_pid_control.py').read())
```

**方式 3：自动启动 (重启自动运行)**

编辑 `boot.py`：
```python
import motor_pid_control
motor_pid_control.main()
```

### 与程序交互

程序运行后，你会看到：
```
============================================================
电机 PID 角度控制系统初始化
============================================================
✓ PWM 捕获初始化完成 (引脚 4)
✓ 电机控制初始化完成 (引脚 2)
✓ PID 控制器初始化: Kp=9.8, Ki=1.0, Kd=0.49
✓ 串口初始化完成 (波特率 115200)
✓ 系统初始化完成

============================================================
启动 PID 闭环角度控制
============================================================
通过串口输入目标角度 (0-360°)
示例: 90 或 270.5
按 Ctrl+C 停止
```

### 发送命令

在 Thonny 的串口终端或任何串口工具中，输入目标角度：

```
90          ← 回车，电机旋转到 90°
180.5       ← 回车，电机旋转到 180.5°
0           ← 回车，电机回到起始位置 (0°)
```

**实时数据输出示例：**
```
初始目标角度设置为当前角度: 45.00°
新目标角度: 90.00°
target= 90.00° angle= 88.50° err_raw= 1.50° err=  1.50° u_raw= 14.70 u_filt= 14.20 duty=1385 period=20000 high=1440
target= 90.00° angle= 89.80° err_raw= 0.20° err=  0.20° u_raw=  1.96 u_filt=  8.10 duty=1391 period=20000 high=1455
target= 90.00° angle= 90.00° err_raw= 0.00° err=  0.00° u_raw=  0.00 u_filt=  3.62 duty=1396 period=20000 high=1457
```

---

## 5️⃣ 测试和调试

### 测试 1：PWM 输出测试

验证 ESP32 GPIO2 是否输出正确的 PWM 信号：

```python
from motor_pid_control import MotorControl
import time

motor = MotorControl(2)

# 测试中立点
motor.set_duty(1400)
print("电机应该停止")
time.sleep(2)

# 测试正转
motor.set_duty(1450)
print("电机应该正转")
time.sleep(2)

# 测试反转
motor.set_duty(1350)
print("电机应该反转")
time.sleep(2)

# 回到中立
motor.set_duty(1400)
print("电机应该停止")
```

### 测试 2：反馈信号测试

验证 ESP32 GPIO4 是否正确读取电机反馈：

```python
from motor_pid_control import PWMCapture
import time

pwm = PWMCapture(4)

for i in range(20):
    valid, angle_deg, period_us, high_us = pwm.get_latest_angle()
    if valid:
        print(f"有效 - 角度: {angle_deg:6.2f}° | 周期: {period_us}μs | 高电平: {high_us}μs")
    else:
        print("无效或无反馈信号")
    time.sleep(0.1)
```

### 测试 3：完整 PID 控制测试

```python
from motor_pid_control import MotorPIDControlSystem

system = MotorPIDControlSystem()
system.run_pid_control()

# 通过串口发送:
# 90 (回车)
# 180 (回车)
# 270 (回车)
```

### 常见问题排查

| 问题 | 原因 | 解决方案 |
|-----|------|--------|
| 电机不动 | 中立点不对 | 调整 `MOTOR_PWM_NEUTRAL` 值 |
| 电机抖动 | PID 参数太激进 | 降低 `PID_KP` 值 |
| 反馈信号无效 | GPIO4 未连接 | 检查反馈线路或禁用反馈 (见下) |
| 无法烧录 | USB 驱动问题 | 重新安装 CH340 或 CP2102 驱动 |
| 串口连接失败 | COM 端口错误 | 检查设备管理器中的实际 COM 号 |

### 禁用反馈信号 (仅开环测试)

如果你的电机没有反馈信号或想简化测试，可以禁用反馈：

```python
# 在 motor_pid_control.py 中，修改 run_pid_control() 方法

# 注释掉这一行:
# valid, angle_deg, period_us, high_us = self.pwm_capture.get_latest_angle()

# 替换为:
valid = False  # 禁用反馈验证
angle_deg = self.pid.target_deg  # 假设电机完全执行命令
```

---

## 6️⃣ 常见问题

### Q1: 为什么电机反应缓慢？
**A:** PWM 脉宽范围可能设置不对。尝试调整：
- 增加 `MOTOR_PWM_MAX - MOTOR_PWM_MIN` 的差值
- 或增加 `PID_KP` 值

### Q2: 如何找到我的电机的准确中立点？
**A:** 运行测试脚本逐步搜索:
```python
# 测试脉宽范围 1300-1500μs，找到电机不动的位置
for pulse in range(1300, 1500, 10):
    motor.set_duty(pulse)
    time.sleep(0.5)
    # 观察电机，记录停止转动的脉宽值
```

### Q3: 能同时控制两个电机吗？
**A:** 可以，但需要小改动 (后续可更新程序)

### Q4: 电机怎么控制旋转角度？
**A:** PWM 脉宽 1000-2000μs 对应 0-360° 旋转角 (取决于你的旋转舵机)。程序会自动计算。

### Q5: 连接 PCA9685 时还需要 GPIO2 吗？
**A:** GPIO2 用于直接 PWM 输出。如果用 PCA9685，改成使用 PCA9685 的 I2C 通道（需要更新代码）。

---

## 📞 快速参考

**快速启动命令：**
```bash
# 1. 连接 USB，识别 COM 端口
esptool.py --port COM3 flash_id

# 2. 烧录固件
esptool.py --chip esp32 --port COM3 erase_flash
esptool.py --chip esp32 --port COM3 write_flash -z 0x1000 esp32-20240105-v1.22.1.bin

# 3. 上传代码
mpremote connect COM3
mpremote cp motor_pid_control.py :/motor_pid_control.py

# 4. 运行
mpremote connect COM3 repl
>>> exec(open('motor_pid_control.py').read())
```

**关键 GPIO 速查表：**
- GPIO2: 电机 PWM 输出
- GPIO4: 电机反馈信号 (PWM 输入)
- GPIO21: I2C SDA
- GPIO22: I2C SCL

---

祝你调试顺利！🚀
