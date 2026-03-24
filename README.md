# 🚤 SteadySail 项目

主动平衡水上运动设备稳定器系统

## 📋 项目概述

SteadySail 是一个为水上运动设备（如 SUP、皮划艇、小型渔船）提供主动平衡和稳定性的智能系统。

- ✅ **双 IMU 融合** - 高精度姿态估计
- ✅ **PID 反馈控制** - 前馈补偿
- ✅ **实时 PWM 控制** - 双电机同步
- ✅ **WiFi 数据传输** - 远程监测

---

## 📁 项目结构

```
SteadySail-/
├── esp32_new/                           ⭐ ESP32 MicroPython 项目 (生产环境)
│   ├── main.py                          主程序
│   ├── boot.py                          启动脚本
│   ├── config.py                        配置参数
│   └── lib/                             驱动库
│       ├── mpu6050.py                   IMU 驱动
│       ├── pca9685.py                   PWM 驱动
│       ├── pid_controller.py            PID 控制器
│       ├── imu_fusion.py                IMU 融合算法
│       └── wifi_helper.py               WiFi 辅助
│
├── 📄 配置文件 (JSON)
│   ├── calibration_imu1.json            IMU1 校准参数
│   ├── calibration_imu2.json            IMU2 校准参数
│   └── pid_presets.json                 PID 预设值
│
├── 📚 docs/                             文档和指南
│   ├── ESP32_DEPLOYMENT_GUIDE.md        ESP32 部署指南
│   ├── MICROPYTHON_GUIDE.md             MicroPython 教程
│   ├── ESP32_COMPATIBILITY_ANALYSIS.md  兼容性分析
│   └── ...
│
├── 🔧 raspberry_pi_backup/              树莓派原始代码 (历史参考)
│   ├── feedforward_dual_imu.py          原始主程序
│   ├── motor_test.py                    电机驱动
│   ├── dual_imu.py                      IMU 融合
│   └── ...
│
├── 📊 data_archive/                     数据存档 (测试/分析)
│   ├── analysis_results/                分析结果
│   ├── data_analysis/                   分析脚本
│   └── ...
│
└── README.md                            本文件
```

---

## 🚀 快速开始

### 1️⃣ 烧录 MicroPython 固件到 ESP32

```bash
# 安装工具
pip install esptool

# 擦除 Flash
esptool.py --chip esp32 --port COM3 erase_flash

# 烧录固件 (下载最新版本)
esptool.py --chip esp32 --port COM3 write_flash -z 0x1000 esp32-xxx.bin
```

### 2️⃣ 上传代码到 ESP32

**使用 Thonny IDE (推荐):**
1. 下载并安装 Thonny
2. 工具 → 配置解释器 → 选择 MicroPython (ESP32)
3. 选择串口和波特率
4. 将 `esp32_new/` 文件夹中的所有文件上传到 ESP32

### 3️⃣ 配置参数

编辑 `esp32_new/config.py` 中的参数：

```python
# WiFi 配置
WIFI_SSID = "Your_WiFi_SSID"
WIFI_PASSWORD = "Your_WiFi_Password"

# I2C 地址
IMU1_ADDRESS = 0x68
IMU2_ADDRESS = 0x69
PCA9685_ADDRESS = 0x40

# PID 参数
PID_KP = 20.0
PID_KI = 1.0
PID_KD = 0.0

# 等等...
```

### 4️⃣ 运行程序

在 REPL 中执行：
```python
>>> import main
```

---

## 📖 文档导航

| 文档 | 用途 |
|------|------|
| **docs/ESP32_DEPLOYMENT_GUIDE.md** | 详细的烧录和部署步骤 |
| **docs/MICROPYTHON_GUIDE.md** | MicroPython 基础和 API 说明 |
| **docs/ESP32_COMPATIBILITY_ANALYSIS.md** | 树莫派到 ESP32 的迁移分析 |
| **PROJECT_STRUCTURE.md** | 项目文件结构说明 |

---

## 🔧 主要功能

### 📊 双 IMU 融合
- MPU6050 × 2 (I2C)
- 互补滤波算法
- 在线零偏估计
- 动态权重调整

### 🎛️ PID 控制
- 2 自由度 PID 控制器
- 前馈补偿项
- 反馈误差修正
- 自适应死区

### 🔌 电机驱动
- PCA9685 PWM 扩展板
- 两个推进器同步控制
- PWM 脉宽范围: 1000-2000 μs
- 频率: 50 Hz

### 📡 数据通信
- WiFi UDP 数据发送
- 实时姿态和控制数据
- 可选: CSV 数据记录

---

## 🛠️ 硬件要求

| 器件 | 型号 | 说明 |
|------|------|------|
| 控制器 | **ESP32** | 主控制器 |
| 惯性传感器 | **MPU6050** × 2 | 双 IMU |
| PWM 驱动 | **PCA9685** | 16 路 PWM 扩展 |
| 推进器 | **ESC + 电机** × 2 | 水下推进 |
| 电源 | **5V/3A** | 供电 |

---

## 🎯 针对不同用户的指南

### 👨‍💻 软件开发者
1. 查看 `docs/ESP32_DEPLOYMENT_GUIDE.md` 了解如何部署
2. 修改 `esp32_new/config.py` 调整参数
3. 编辑 `esp32_new/main.py` 自定义控制逻辑
4. 参考 `esp32_new/lib/` 中的驱动代码

### 🔬 研究人员
1. 查看 `docs/MICROPYTHON_GUIDE.md` 理解代码结构
2. 参考 `raspberry_pi_backup/feedforward_dual_imu.py` 原始算法
3. 查看 `data_archive/` 中的分析结果
4. 在 `esp32_new/main.py` 中修改算法进行研究

### 🔧 硬件集成师
1. 检查 `esp32_new/lib/mpu6050.py` 中的 I2C 地址配置
2. 修改 `esp32_new/lib/pca9685.py` 中的 PWM 频率
3. 查看 `docs/MICROPYTHON_GUIDE.md` 中的硬件配置部分
4. 使用 REPL 测试各个硬件模块

---

## 📊 参数文件说明

### `calibration_imu1.json` / `calibration_imu2.json`
```json
{
  "accel_offset": [-50, 100, 0],
  "gyro_offset": [5.2, -3.1, 2.8],
  "temperature_offset": 25.0
}
```

### `pid_presets.json`
```json
{
  "default": {
    "kp": 20.0,
    "ki": 1.0,
    "kd": 0.0
  },
  "aggressive": {
    "kp": 35.0,
    "ki": 2.0,
    "kd": 5.0
  }
}
```

---

## 🐛 常见问题

| 问题 | 解决方案 |
|------|--------|
| **IMU 无法读取** | 检查 I2C 引脚 (21/22), 查看 `esp32_new/lib/mpu6050.py` |
| **PWM 输出异常** | 检查 PCA9685 I2C 地址，修改 `config.py` 中的 `PCA9685_ADDR` |
| **WiFi 无法连接** | 检查 `config.py` 中的 WiFi 凭证和信号强度 |
| **程序崩溃** | 查看 REPL 输出的错误信息，可能是内存溢出 |

---

## 🔄 版本历史

- **v2.0** (2024 年) - ESP32 MicroPython 版本 ⭐ 当前版本
- **v1.0** (2023 年) - 树莴派 Python 版本 (详见 `raspberry_pi_backup/`)

---

## 📞 项目团队

- **Tony (Ushao)** - 算法和控制系统
- **MAX** - 硬件设计
- **Adam Wei** - 机械结构

---

## 📄 许可证

本项目为学位项目，详见团队文档。

---

## 🎓 学习资源

- [MicroPython 官方文档](https://docs.micropython.org/en/latest/esp32/)
- [ESP32 快速参考](https://docs.micropython.org/en/latest/esp32/quickref.html)
- [Thonny IDE](https://thonny.org/) - MicroPython IDE

---

**最后更新**: 2024 年 3 月  
**状态**: ✅ 生产就绪
