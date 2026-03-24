# 📁 SteadySail 项目文件夹结构说明

## 🎯 当前项目状态

本项目包含**两个版本**的控制代码：
- **原始版本**: 树莓派 Python 代码
- **新版本**: ESP32 MicroPython 代码

---

## 📂 文件夹说明

### 🆕 新建的文件夹

#### `esp32_new/` ⭐ **这是 ESP32 项目，直接使用这个！**
```
esp32_new/
├── main.py                      ⭐ 主程序 (运行这个启动控制器)
├── boot.py                      启动脚本 (ESP32 自动运行)
├── config.py                    配置文件 (修改这里调参)
├── README.md                    完整使用指南
├── DEPLOY_GUIDE.md              部署快速参考
├── calibration_imu1.json        校准参数
├── calibration_imu2.json        校准参数
└── lib/                         驱动库
    ├── mpu6050.py              IMU 驱动
    └── pca9685.py              PWM 驱动
```

**用途**: 直接在 ESP32 上烧录和运行  
**状态**: ✅ 完全准备就绪  
**文件数**: 9 个  

---

#### `raspberry_pi_backup/` 📦 **原始树莮派代码 (备份)**
用途: 参考和备份原始树莮派代码  
**注**: 这里的代码在 ESP32 上无法直接运行

---

### 📋 主文件夹中的关键文件

#### 📄 `ESP32_PROJECT_COMPLETE.md` ⭐ **先看这个！**
- 项目完成总结
- 文件清单
- 功能说明
- 使用步骤

#### 📄 `ESP32_COMPATIBILITY_ANALYSIS.md`
- 兼容性详细分析
- 哪些功能可用，哪些不可用

#### 📄 `MICROPYTHON_GUIDE.md`
- MicroPython 详细指南
- ESP32 入门知识

#### 📄 其他文档文件
- `ESP32_QUICK_START.md`
- `ESP32_DEPLOYMENT_GUIDE.md`
- `ESP32_FILES_SUMMARY.md`

---

## 🚀 快速开始 (3 步)

### 1️⃣ 进入 ESP32 项目文件夹
```
cd esp32_new/
```

### 2️⃣ 阅读 README.md
```
内含完整的部署和使用指南
```

### 3️⃣ 上传文件到 ESP32
```
使用 Thonny IDE 上传所有文件
```

---

## 📊 项目结构概览

```
SteadySail-/
│
├── 🆕 esp32_new/              ← ESP32 项目 (推荐使用)
│   ├── main.py                ⭐ 主程序
│   ├── boot.py                启动脚本
│   ├── config.py              配置文件
│   ├── README.md              使用指南
│   ├── DEPLOY_GUIDE.md        部署参考
│   └── lib/                   驱动库
│
├── 📦 raspberry_pi_backup/    ← 原始树莓派代码 (备份)
│
├── 📄 ESP32_PROJECT_COMPLETE.md  ⭐ 先读这个！
├── 📄 ESP32_COMPATIBILITY_ANALYSIS.md
├── 📄 MICROPYTHON_GUIDE.md
│
├── 分析结果和数据文件 (旧版本)
│   ├── analysis_results/
│   ├── comparison_results/
│   ├── data_analysis/
│   └── ...
│
└── 其他 (不需要关注)
    ├── torpedo/
    ├── IMU_data_collection/
    └── ...
```

---

## ✅ 使用建议

### 如果你想在 **ESP32** 上运行
```
✓ 进入 esp32_new/ 文件夹
✓ 阅读 README.md
✓ 上传文件到 ESP32
✓ 修改 config.py 调参
✓ 运行！
```

### 如果你想回到 **树莓派**
```
✓ 从 raspberry_pi_backup/ 获取原始代码
✓ 或者 git 回滚到之前的版本
```

### 如果你想了解 **兼容性问题**
```
✓ 阅读 ESP32_COMPATIBILITY_ANALYSIS.md
✓ 了解哪些功能可用，哪些需要改
```

---

## 🎯 核心改动

### 保留了什么 ✅
- PID 控制算法
- IMU 融合算法
- 前馈补偿逻辑
- 所有物理参数

### 改写了什么 🔄
- I2C 驱动 (smbus → machine.I2C)
- IMU 驱动 (重写为 MicroPython)
- PWM 驱动 (重写为 MicroPython)
- 数据存储 (缓冲 CSV)

### 移除了什么 ❌
- Flask Web 服务器
- threading 多线程
- NumPy 库
- 复杂的远程调参

---

## 📞 文件清单

### 需要上传到 ESP32 的文件
```
esp32_new/
├── main.py ⭐
├── boot.py
├── config.py
├── calibration_imu1.json
├── calibration_imu2.json
└── lib/
    ├── mpu6050.py
    └── pca9685.py
```

### 不需要上传的文件
```
- README.md (可选，只是文档)
- DEPLOY_GUIDE.md (可选，只是文档)
- 其他 .md 文件 (都是说明文档)
```

---

## 🔧 常见问题

### Q: 我应该编辑哪个文件？
**A:** 
- **主程序逻辑**: `esp32_new/main.py`
- **配置参数**: `esp32_new/config.py` (推荐)
- **IMU 驱动**: `esp32_new/lib/mpu6050.py`
- **PWM 驱动**: `esp32_new/lib/pca9685.py`

### Q: 树莮派版本在哪？
**A:** 原始树莮派代码在 `raspberry_pi_backup/` 文件夹或旧的 git 提交

### Q: 我想同时支持 ESP32 和树莮派怎么办？
**A:** 
- 保持 `esp32_new/` 作为 ESP32 版本
- 保持 `raspberry_pi_backup/` 作为树莮派版本
- 共享核心算法代码

### Q: 哪些文档最重要？
**A:** 按优先级：
1. `ESP32_PROJECT_COMPLETE.md` (总结)
2. `esp32_new/README.md` (使用指南)
3. `esp32_new/DEPLOY_GUIDE.md` (快速参考)
4. `ESP32_COMPATIBILITY_ANALYSIS.md` (技术细节)

---

## 🎓 学习路径

如果你对 ESP32 开发感兴趣：

1. **基础知识**
   ```
   读: MICROPYTHON_GUIDE.md
   学: MicroPython 官方文档
   ```

2. **实践应用**
   ```
   读: esp32_new/README.md
   做: 烧录和运行
   改: 修改 config.py 调参
   ```

3. **深入理解**
   ```
   读: main.py 源代码
   改: 修改算法
   学: 硬件驱动编写
   ```

4. **扩展功能**
   ```
   添加: WiFi 通信
   添加: 蓝牙控制
   添加: Web 界面
   ```

---

## 📊 版本信息

| 方面 | 树莓派版本 | ESP32 版本 |
|------|----------|----------|
| 文件位置 | `raspberry_pi_backup/` | `esp32_new/` |
| 语言/框架 | Python 3 | MicroPython |
| 主文件 | `feedforward_dual_imu.py` | `main.py` |
| 代码量 | ~1000 行 | ~600 行 |
| 控制频率 | 200-500 Hz | 100-200 Hz |
| 存储需求 | 无限 | 1-2 MB |
| 状态 | 原始版本 | 新版本 ✅ |

---

## ✨ 总结

```
原始树莮派代码
        ↓
改写适配 ESP32
        ↓
完整的 ESP32 MicroPython 项目
(保存在 esp32_new/)
        ↓
可直接烧录到 ESP32 使用
```

**现在你可以：**
- ✅ 在 ESP32 上运行所有控制逻辑
- ✅ 轻松修改配置文件调参
- ✅ 自动记录数据到 SPIFFS
- ✅ 扩展功能（WiFi、蓝牙等）

**立即开始**: 进入 `esp32_new/` 文件夹，阅读 `README.md`！

---

**最后更新**: 2026-03-24  
**状态**: ✅ 项目完成，可投入使用
