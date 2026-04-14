# 🔌 电机 PWM 反馈 PID 控制系统 - 设置指南

## 概述

本指南基于 `my_project` 中的 `pwm_capture.c` 实现，使用 **GPIO4 PWM 反馈信号**（而非 I2C 磁编码器）进行闭环角度控制，完全兼容 ESP32-S3。

**文件列表：**
- `motor_pwm_feedback_pid.py` - 主控制程序（推荐使用）
- `motor_pid_single.py` - 开环控制（仅测试用）

---

## 1️⃣ 固件烧录

### 步骤 1：获取 COM 端口

```powershell
# 连接 ESP32-S3，在设备管理器中查看 COM 端口
# 例如: COM3
```

### 步骤 2：擦除 Flash

```powershell
# 使用 ESP32-S3 芯片类型
python -m esptool --chip esp32-s3 --port COM3 erase-flash
```

### 步骤 3：烧录 MicroPython 固件

```powershell
# 1. 下载最新固件：https://micropython.org/download/esp32-s3/
# 例如下载到：D:\Downloads\ESP32_S3-20240105-v1.22.1.bin

# 2. 烧录
python -m esptool --chip esp32-s3 --port COM3 --baud 460800 write-flash -z 0x0 `
  "D:\Downloads\ESP32_S3-20240105-v1.22.1.bin"

# 3. 验证
python -m esptool --chip esp32-s3 --port COM3 flash-id
```

---

## 2️⃣ 硬件连线

### 引脚配置

| 功能 | ESP32-S3 引脚 | 电机侧 | 说明 |
|-----|------------|-------|------|
| PWM 控制输出 | GPIO2 | 控制信号 (黄) | 驱动电机旋转方向和速度 |
| PWM 反馈输入 | GPIO4 | 反馈信号 (绿) | 读取电机当前角度 |
| 电源 VCC | 3.3V 或 5V | VCC (红) | 根据电机规格选择 |
| 地线 | GND | GND (黑) | 共地，必须连接 |

### 接线图

```
ESP32-S3               伺服电机/推进器
┌─────────┐           ┌────────────┐
│ GPIO2 ──────────────→ 控制信号(黄)│
│ GPIO4 ←──────────────  反馈信号(绿)│
│ GND ───────────────→ GND (黑)   │
│ 5V ────────────────→ VCC (红)   │
└─────────┘           └────────────┘
```

---

## 3️⃣ 上传代码

### 方式 A：Thonny IDE (推荐)

1. 下载安装 Thonny: https://thonny.org/
2. 菜单 → Tools → Options → Interpreter → MicroPython (ESP32)
3. 选择串口 COM3，波特率 115200
4. 打开 `motor_pwm_feedback_pid.py`
5. Ctrl+S 保存到设备 → `motor_pwm_feedback_pid.py`

### 方式 B：mpremote 命令行

```powershell
# 上传主程序
mpremote connect COM3
mpremote cp motor_pwm_feedback_pid.py :/motor_pwm_feedback_pid.py
mpremote disconnect
```

---

## 4️⃣ 运行程序

### 使用 Thonny

1. 打开 `motor_pwm_feedback_pid.py`
2. 按 F5 运行
3. 查看输出信息

### 使用 mpremote

```powershell
mpremote connect COM3 repl
>>> exec(open('motor_pwm_feedback_pid.py').read())
```

### 程序输出示例

```
============================================================
电机 + PWM 反馈 PID 控制系统
============================================================

✓ PWM 反馈捕获初始化 (GPIO 4)
✓ 电机控制初始化 (GPIO 2)
✓ PID 初始化: Kp=9.8, Ki=1.0, Kd=0.49
✓ 串口初始化 (波特率 115200)
✓ 系统就绪

============================================================
启动 PID 反馈控制
============================================================
模式: 闭环 PID（有 PWM 反馈）
通过串口输入目标角度 (0-360°)
示例: 90 或 270.5
Ctrl+C 停止

初始目标: 0.0°

✓ 目标: 90.0° | 当前: 88.5° | 误差: 1.5° | 输出: 14.7 | PWM: 1385μs | 周期: 20000μs | 脉宽: 1440μs
✓ 目标: 90.0° | 当前: 89.8° | 误差: 0.2° | 输出: 2.0 | PWM: 1398μs | 周期: 20000μs | 脉宽: 1455μs
✓ 目标: 90.0° | 当前: 90.0° | 误差: 0.0° | 误差: 0.0° | 输出: 0.0 | PWM: 1400μs | 周期: 20000μs | 脉宽: 1457μs
```

---

## 5️⃣ 使用指南

### 通过串口发送命令

在 Thonny 的串口终端中输入目标角度：

```
90       ← 回车，电机旋转到 90°
180.5    ← 回车，电机旋转到 180.5°
270      ← 回车，电机旋转到 270°
0        ← 回车，电机回到起始位置
```

### 实时反馈信息

每 200ms 输出一次状态：

- **✓** 表示反馈信号有效
- **✗** 表示反馈信号超时或无效
- **目标**: 目标角度
- **当前**: 反馈读取的当前角度
- **误差**: 角度误差
- **输出**: PID 输出值
- **PWM**: 实际输出到电机的脉宽 (μs)
- **周期/脉宽**: 反馈 PWM 信号的周期和高电平宽度

---

## 6️⃣ 调试和校准

### 电机不动

1. 检查供电是否正常
2. 检查 GPIO2 线路是否连接正确
3. 测试电机中立点：
   ```python
   motor.set_duty(1400)  # 应该停止
   ```

### 反馈信号无效 (显示 ✗)

1. 检查 GPIO4 是否收到反馈信号
2. 使用万用表测量 GPIO4 电压变化 (应该在 0-3.3V 范围)
3. 检查反馈线路是否松动

### 电机响应缓慢/抖动

1. 调整 PID 参数：
   - 增加 `PID_KP` 使响应更快
   - 增加 `PID_KD` 减少超调和抖动
   
2. 在 `motor_pwm_feedback_pid.py` 中修改：
   ```python
   PID_KP = 9.8   # 增大→更快; 减小→更平缓
   PID_KD = 0.49  # 增大→更稳定
   ```

### 找到电机中立点

电机中立点应在 1300-1500μs 范围，找到正确值：

```python
from machine import Pin, PWM
pin = Pin(2, Pin.OUT)
pwm = PWM(pin, freq=50)

# 逐步测试，观察电机是否停止
for pulse in [1350, 1380, 1400, 1420, 1450]:
    duty_u16 = int((pulse / 20000.0) * 65535)
    pwm.duty_u16(duty_u16)
    print(f"Test {pulse}μs")
    time.sleep(2)

# 找到不动的位置后，更新 MOTOR_PWM_NEUTRAL = XXX
```

---

## 7️⃣ 常见问题

### Q：无法连接到 ESP32-S3

**A：** 检查以下几点：
1. USB 线是否是数据线（非充电线）
2. 驱动程序是否已安装（CH340 或 CP2102）
3. COM 端口是否正确
4. 设备管理器中是否显示串口设备

### Q：烧录失败 "Wrong chip argument"

**A：** 确保使用正确的芯片类型：
```powershell
# 如果是 ESP32-S3
python -m esptool --chip esp32-s3 ...

# 如果是普通 ESP32
python -m esptool --chip esp32 ...
```

### Q：反馈信号一直无效

**A：** 可能的原因：
1. 电机没有反馈输出（某些电机没有反馈功能）
2. GPIO4 接线不正确
3. 反馈频率过高/过低，超出范围

此时程序会自动切换到开环模式，电机仍然可以工作。

### Q：能否控制多个电机？

**A：** 可以，但需要修改代码：
- 额外的电机可用 GPIO 其他引脚的 PWM 输出
- 反馈信号可用其他 GPIO 的输入捕获（需要中断处理）

### Q：程序运行速度如何？

**A：** 当前设置：
- PID 循环频率：100Hz (10ms)
- 反馈更新频率：50Hz (20ms)
- 串口波特率：115200 bps

---

## 8️⃣ 快速参考

### 安装依赖

```bash
pip install esptool mpremote pyserial
```

### 快速烧录命令

```powershell
# 全流程
python -m esptool --chip esp32-s3 --port COM3 erase-flash
python -m esptool --chip esp32-s3 --port COM3 --baud 460800 write-flash -z 0x0 "path\to\firmware.bin"
python -m esptool --chip esp32-s3 --port COM3 flash-id

# 上传程序
mpremote cp motor_pwm_feedback_pid.py :/motor_pwm_feedback_pid.py
```

### 关键参数

| 参数 | 值 | 说明 |
|-----|-----|-----|
| GPIO2 (PWM out) | 50 Hz | 伺服舵机标准频率 |
| PWM 范围 | 1320-1480 μs | 对应电机行程范围 |
| PWM 中立 | 1400 μs | 电机停止位置 |
| GPIO4 (PWM in) | 变化 | 读取电机反馈 |
| PID 循环 | 10 ms | 100 Hz 控制频率 |

---

祝调试顺利！🚀
