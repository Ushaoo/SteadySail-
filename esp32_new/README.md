# ESP32 MicroPython 前馈双IMU控制器

完整的 ESP32 MicroPython 项目，用于无人船的平衡控制。

## 📁 项目结构

```
esp32_new/
├── main.py                      # 主程序入口 ⭐ 这是运行的主文件
├── boot.py                      # ESP32 启动脚本（自动运行）
├── config.py                    # 配置参数（修改这里调参）
├── calibration_imu1.json        # IMU1 校准参数
├── calibration_imu2.json        # IMU2 校准参数
└── lib/                         # 驱动库文件夹
    ├── mpu6050.py              # MPU6050 IMU 驱动
    └── pca9685.py              # PCA9685 PWM 驱动
```

## ⚡ 快速开始

### 1️⃣ 烧录 MicroPython 固件

```bash
# 安装工具
pip install esptool

# 擦除 Flash
esptool.py --chip esp32 --port COM3 erase_flash

# 烧录固件（下载最新版本）
esptool.py --chip esp32 --port COM3 --baud 460800 write_flash -z 0x1000 esp32-20240105-v1.22.1.bin
```

### 2️⃣ 上传项目文件到 ESP32

使用 **Thonny IDE** 或 **PyMakr** 上传所有文件：
- `main.py`
- `boot.py`
- `config.py`
- `calibration_*.json`
- `lib/` 文件夹及其内容

### 3️⃣ 配置硬件连接

**ESP32 引脚配置：**
```
I2C SCL -------- GPIO 22
I2C SDA -------- GPIO 21
```

**I2C 设备地址：**
- IMU1 (MPU6050): 0x68
- IMU2 (MPU6050): 0x69
- PCA9685: 0x40

### 4️⃣ 配置参数

编辑 `config.py` 设置：
- WiFi SSID 和密码（可选）
- PID 参数 (KP, KI, KD)
- 前馈和反馈系数
- 死区参数

### 5️⃣ 启动程序

**方式 A：自动启动（推荐）**
```
ESP32 上电后自动运行 boot.py，然后运行 main.py
```

**方式 B：手动启动（调试）**
```python
# 在 Thonny IDE 或 REPL 中执行
import main
main.main()
```

## 🎮 控制方式

### 目前的控制流程
1. **IMU 数据采集** → 读取 IMU1 和 IMU2
2. **双 IMU 融合** → 计算姿态角（roll, pitch, yaw）
3. **PID 控制** → 计算反馈力矩
4. **前馈补偿** → 计算前馈力矩
5. **PWM 输出** → 驱动推进器

### 控制参数

在 `main.py` 或 `config.py` 中修改：

```python
# PID 参数 (影响稳定性)
PID_KP = 20.0       # 比例 - 增加使反应更快（容易振荡）
PID_KI = 1.0        # 积分 - 增加消除静差
PID_KD = 0.0        # 微分 - 增加阻尼（抑制振荡）

# 前馈反馈 (影响稳定性和功耗)
FEEDFORWARD_PARAM = 0.28  # 增加更主动的补偿
FEEDBACK_PARAM = 0.5       # 增加更强的反馈控制

# 死区 (避免抖动)
ANGLE_DEADZONE = 1.0       # 内死区（< 1° 不动作）
ANGLE_DEADZONE_SOFT = 3.0  # 软边界（1-3° 渐进响应）
```

## 📊 数据记录

程序会自动记录 CSV 文件：
- 文件名: `feedforward_esp32_<timestamp>.csv`
- 包含: 时间戳、姿态角、角速度、力矩、PWM 等
- 缓冲写入，自动防止 SPIFFS 填满

## 🔧 调试和监控

### 使用 Thonny IDE 查看输出

```
✓ IMU 初始化成功
✓ PCA9685 初始化成功
✓ 校准参数已加载
开始校准 (200 样本)...
✓ 校准完成
✓ 控制器已启动
Roll:  -0.45° | PWM: 1487 1513
Roll:  -0.32° | PWM: 1490 1510
...
```

### 查看数据日志

```bash
# 从 ESP32 下载 CSV 文件
# 然后用 Excel 或 Python 分析
```

## ⚠️ 常见问题

### Q: I2C 通信失败
**A:** 检查：
- GPIO 21/22 是否有上拉电阻 (4.7kΩ 推荐)
- I2C 地址是否正确 (用 i2c.scan() 验证)
- 连接线是否牢固

### Q: IMU 数据不稳定
**A:** 
- 增加 `ALPHA_EMA` 值 (0.1-0.3)
- 远离 WiFi 天线和电源线
- 检查 IMU 是否安装稳定

### Q: 功率不足
**A:**
- 增加 `FEEDFORWARD_PARAM` 系数
- 降低 `ANGLE_DEADZONE` 让它更灵敏
- 检查电源质量 (需要稳定 5V)

### Q: ESP32 重启循环
**A:**
- 减少 `CONTROL_FREQUENCY` (降低 CPU 使用率)
- 禁用 WiFi (`ENABLE_WIFI = False`)
- 检查内存使用 (在 REPL 中运行 `import gc; print(gc.mem_free())`)

## 📞 文件说明

| 文件 | 说明 |
|------|------|
| `main.py` | 主控制程序，包含所有算法 |
| `boot.py` | 启动脚本，自动连接 WiFi 并运行 main.py |
| `config.py` | 配置文件，修改这里来调参 |
| `lib/mpu6050.py` | IMU 驱动，处理 I2C 通信和数据解析 |
| `lib/pca9685.py` | PWM 驱动，控制推进器脉宽 |
| `calibration_*.json` | 校准参数，首次运行自动生成 |

## 🚀 性能指标

- **控制循环频率**: 100 Hz
- **IMU 采样率**: ~125 Hz
- **数据记录速率**: ~100 Hz
- **内存使用**: ~200 KB
- **存储空间**: ~1 MB (SPIFFS)

## 📝 相比树莓派的改动

### ✅ 完全保留
- PID2DOF 控制器算法
- DualIMUFusion 融合算法
- apply_deadzone_smooth 死区处理
- 所有控制逻辑

### 🔄 轻微改动
- 去除 NumPy，改用原生 Python math
- 去除 threading，改用 MicroPython timer
- CSV 缓冲写入（防止 SPIFFS 满溢）
- I2C 驱动改为 MicroPython 版本

### ❌ 移除
- Flask Web 调参服务器 (可用 config.py 替代)
- UDP 数据发送 (可选，需自己实现)
- 复杂的多线程逻辑

## 🔗 资源链接

- **MicroPython 文档**: https://docs.micropython.org/
- **Thonny IDE**: https://thonny.org/
- **ESP32 引脚参考**: https://en.wikipedia.org/wiki/ESP32

---

**最后更新**: 2026-03-24
**兼容版本**: MicroPython 1.22+
**硬件**: ESP32 DevKit
