# SteadySail (ESP32-S3) 当前工程说明

本 README 对应当前仓库中的 ESP32-S3 主控代码实现，重点覆盖：

- `main/`
- `components/Blinker/`（IoT 远程控制）
- `CMakeLists.txt` / `main/CMakeLists.txt`
- `sdkconfig` / `config/`

说明：`raspberry_pi_backup/` 是历史 Python 备份，不属于当前控制链路。

---

## 1. 项目概览

SteadySail 是一个基于 ESP32-S3 的双推进器 + 双矢量舵机姿态稳定控制系统。当前实现采用：

- 主循环 100 Hz（10 ms 固定周期）
- MPU6050 IMU（默认单 IMU，可切换双 IMU）
- MT6826S PWM 编码器采样矢量舵机角度
- LEDC 50 Hz 14-bit PWM 驱动转向舵机与推进 ESC
- Mahony 四元数姿态融合 + 启动 IMU 零位归零
- V3 矢量分解控制律（竖直分量反相、水平分量同相、电机不反转）
- 推进–舵机响应耦合：舵机就绪门控（Scheme A）
- NVS 持久化舵机零点校准（一次校准、永久保存）
- Blinker IoT 双向遥控 + 串口命令双通道

核心目标：在给定前进推力的同时维持横滚稳定，并允许通过 App / 串口实时干预。

---

## 2. 代码结构

### 顶层

- `CMakeLists.txt`：ESP-IDF 工程入口，项目名 `version_2`
- `main/CMakeLists.txt`：注册以下源文件，依赖 `Blinker json nvs_flash`
  - `main.c`、`imu_driver.c`、`balance_controller.c`
  - `motor_control.c`、`steering_control.c`
  - `control_params.c`、`blinker_bridge.c`

### 主功能模块（`main/`）

- [main.c](main/main.c)
  - 系统初始化、按模式分发任务
  - 100 Hz `control_core_task`：V3 矢量控制 + 舵机就绪门控 + 安全保护
  - 串口命令解析（`w/s/space/数字/r/f/cal`）
- [system_config.h](main/system_config.h)
  - 运行模式、转向控制模式
  - GPIO 映射
  - PWM 安全限幅、方向反转开关
  - 控制参数与安全保护开关
- [imu_driver.c](main/imu_driver.c) / [.h](main/imu_driver.h)
  - I2C 初始化、MPU6050 唤醒与读取
  - 单/双 IMU 数据输出统一接口
- [balance_controller.c](main/balance_controller.c) / [.h](main/balance_controller.h)
  - Mahony 四元数姿态估计
  - 陀螺仪零偏自动校准（200 样本）
  - 启动 IMU 姿态归零（再积 100 样本后捕获 attitude offset）
  - 前馈 + 2DOF PID 力矩计算
- [steering_control.c](main/steering_control.c) / [.h](main/steering_control.h)
  - GPIO 中断捕获 PWM、占空比转角度
  - 角度 clamp 到 MT6826S 真实有效范围（DC 1%–99%）
  - PID 闭环输出（DEADZONE = 3°，最小输出 30/60 µs 阈值）
  - **NVS 持久化校准**：自动加载 / 串口 `cal` 写入；未校准时锁 PWM = 1500
- [motor_control.c](main/motor_control.c) / [.h](main/motor_control.h)
  - LEDC 初始化、PWM 输出
  - 双向推力 `motor_control_set_pwm_bidirectional()`
  - ESC 校准任务、紧急停推
- [control_params.c](main/control_params.c) / [.h](main/control_params.h)
  - 全局可调参数（推力、PID、演示 Roll 等）
  - NVS Flash 初始化
- [blinker_bridge.c](main/blinker_bridge.c) / [.h](main/blinker_bridge.h)
  - Blinker IoT 任务封装
  - 解析 App 文本命令（PID 调参 / `r <角度>` / `f <推力%>`）

### 第三方组件

- [components/Blinker](components/Blinker)：基于乐鑫 esp-idf-blinker 的 Blinker 客户端
  - `blinker_timesync` 任务栈已上调到 4 KB，解决 SNTP 栈溢出

---

## 3. 运行模式

在 [system_config.h](main/system_config.h) 中通过 `CURRENT_RUN_MODE` 选择：

| 宏 | 值 | 说明 |
|---|---|---|
| `MODE_TEST_SENSORS` | 0 | 仅打印 IMU 与编码器 |
| `MODE_TEST_STEERING_ONLY` | 1 | 仅舵机闭环测试 |
| `MODE_TEST_BALANCE_ONLY` | 2 | 仅平衡链路测试 |
| `MODE_FULL_INTEGRATION` | 3 | 全链路（默认） |
| `MODE_CALIBRATE_ESC` | 4 | 大电机 ESC 校准扫描 |
| `MODE_TEST_IMU_ONLY` | 5 | IMU 原始 + 融合数据测试 |

**当前默认：`MODE_FULL_INTEGRATION`**。

### 演示开关

`DEMO_MANUAL_ROLL = 1`（仅 FULL_INTEGRATION 生效）：忽略 IMU，使用 `g_demo_roll_deg` 模拟横滚，方便桌面演示舵机响应。

---

## 4. 转向控制模式

`STEERING_CONTROL_MODE`：

- `STEERING_MODE_PID`（默认）：PID 闭环，精度高
- `STEERING_MODE_DIRECT`：直接误差→PWM 映射

---

## 5. 硬件接口与引脚

| 功能 | 接口 |
|---|---|
| IMU0 I2C SDA / SCL | GPIO 8 / 9 |
| IMU1 I2C SDA / SCL（预留） | GPIO 10 / 11 |
| 编码器输入 左 / 右 | GPIO 5 / 4 |
| 转向舵机 PWM 左 / 右 | GPIO 2 / 1 |
| 推进 ESC PWM 左 / 右 | GPIO 18 / 19 |

### PWM 配置

- 频率 50 Hz，分辨率 14-bit
- 中立脉宽 1500 µs
- 调试限幅（[system_config.h](main/system_config.h)）：
  - 推进：`THRUST_PWM_MIN_US = 1200`，`THRUST_PWM_MAX_US = 1800`
  - 舵机：`STEER_PWM_MIN_US = 1000`，`STEER_PWM_MAX_US = 2000`

### 方向反转开关

| 宏 | 当前 | 含义 |
|---|---|---|
| `THRUST_LEFT_INVERT` / `THRUST_RIGHT_INVERT` | 0 / 0 | 推进器方向 |
| `STEER_LEFT_INVERT` / `STEER_RIGHT_INVERT` | 1 / 1 | 舵机 PWM 镜像 |
| `ENC_LEFT_REVERSE` / `ENC_RIGHT_REVERSE` | 0 / 0 | 编码器读数翻转 |

---

## 6. 主控制流程（FULL_INTEGRATION）

`control_core_task()` 100 Hz：

1. 读取舵机当前角度（编码器） + IMU 数据
2. `balance_controller_update()` → 姿态、`tau_total`
3. 安全保护：`|roll| > 60°` → 紧急停推 + 舵机回 180°
4. **V3 矢量分解**（详见 §8）→ `target_angle_L/R` + `T_thrust_L/R`
5. 目标角做环形低通：`filter_target += 0.1 * Δ`，对 360° 取模
6. **L/R 物理输出交换**：`send_tgt_L = 360 − filter_target_R`
7. 显示用反向：`disp_tgt_L = 360 − filter_target_L`
8. **Scheme A 舵机就绪门控**：`r = max(0, 1 − |err|/15)`，仅允许推力上升
9. `motor_control_set_pwm_bidirectional()` 下发 + 编码器健康检查
10. `vTaskDelayUntil()` 保持 10 ms 周期

调试输出 ~10 Hz。

---

## 7. 姿态融合（balance_controller.c）

### 7.1 启动两阶段校准

1. 累计 200 样本求陀螺仪零偏均值
2. 再积 100 样本等 Mahony 收敛 → 捕获 `attitude_offset_roll/pitch/yaw`
3. 之后所有输出欧拉角减去 offset 并归一化到 ±180°

效果：上电时记录"当前姿态 = 零位"，无需手动摆正下方。

### 7.2 Mahony 估计

- 单 IMU：`Kp_mahony = 30.0`
- 双 IMU：自适应增益

### 7.3 控制律

- 前馈：重力扰动 + 惯量项 + 虚拟刚度
- 反馈：2DOF PID（积分限幅）
- `tau_total = FEEDFORWARD_PARAM * tau_ff − FEEDBACK_PARAM * tau_pid`

关键参数：`PID_KP=20`、`PID_KI=1`、`PID_KD=0`、`FEEDFORWARD_PARAM=0.28`、`FEEDBACK_PARAM=0.5`。

---

## 8. V3 矢量分解控制律

[main.c](main/main.c) `control_core_task()`：

```text
V_R = +ΔV   V_L = −ΔV         // 竖直分量相反
H = H_thrust                  // 水平分量同相
phi_R = atan2(H, V_R)         // 右舵角
phi_L = atan2(H, V_L)         // 左舵角
T = sqrt(V² + H²)             // 矢量幅值即推力
```

电机 **不反转**；左右靠舵机 0/180° 切换实现"推/拉"对称。`ZERO_EPS = 1e-3` 防止 `atan2(0, −0)` 的 π 跳变。

---

## 9. 转向闭环与编码器（steering_control.c）

- GPIO 双边沿中断捕获 PWM 高低电平时长
- `compute_angle()`：占空比 `clamp` 到 `[1%, 99%]`，映射到 `[0°, 360°]`（不再 wrap，避免边界抖动）
- PID：`DEADZONE = 3°`，无内层 smoothstep；最小有效输出 30/60 µs 阈值
- `set_target()` 内部 `360 − target` 翻转，对应硬件方向

### 9.1 NVS 持久化校准（新增）

- NVS 命名空间 `"steering"`，键 `off_l` / `off_r`（float 以 u32 存储）
- `steering_control_init()` 上电尝试加载：
  - **成功** → 进入"已校准"，正常工作
  - **失败** → "未校准"，`steering_control_update()` 强制下发 PWM = 1500（360° 连续舵机此时静止，最安全）
- `steering_control_calibrate_and_save()`：把当前编码器读数标定为 180°，写入 NVS
- 触发方式：把舵机摆到正下方，串口或 App 输入 `cal`

### 9.2 编码器健康

`valid` 状态判断 + 主循环周期打印故障/恢复日志。

---

## 10. 推进输出与安全（motor_control.c）

- `motor_control_set_pwm_bidirectional(push_L, push_R, invert_L, invert_R)`
- 输入 → 限幅 → 低通滤波 → PWM
- 紧急停止：双路回 1500 µs

ESC 校准模式 (`MODE_CALIBRATE_ESC`) 循环扫描 1000 → 2000 → 1000 µs。

---

## 11. 串口命令（FULL_INTEGRATION）

| 命令 | 作用 |
|---|---|
| `w` / `W` | 推力 +10% |
| `s` / `S` | 推力 −10% |
| `space` 或 ` ` | 推力归零 |
| 数字（如 `30` / `-50`） | 直接设置推力百分比（±100 限幅） |
| `r <角度>` | 演示模式下设置模拟 Roll（仅 `DEMO_MANUAL_ROLL=1`） |
| `f <推力%>` | App / 串口共用的推力指令（±100 限幅） |
| **`cal`** | **舵机零点校准并写入 NVS**（首次烧录后必须执行一次） |

### MODE_TEST_STEERING_ONLY 额外

- 输入数字（90~270）= 目标舵角
- `cal` = 校准并保存

---

## 12. Blinker App 远程控制

- `blinker_bridge_start()` 在 IoT 模式下创建客户端任务
- App 文本框命令支持：
  - `kp 20` / `ki 1` / `kd 0` 等 PID 在线调参
  - `r <角度>` 演示模式 Roll
  - `f <推力%>` 推力百分比
- 时间同步任务栈 4 KB（避免 SNTP 栈溢出）

---

## 13. 构建与烧录

标准 ESP-IDF v4.4.x 工程。

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p <COM端口> flash monitor
```

切换运行模式或参数：

1. 修改 [system_config.h](main/system_config.h)
2. `idf.py build flash`

### 首次上电流程（重要）

1. 烧录后串口监视器会提示 **"⚠ 未发现校准数据，舵机已锁定 PWM=1500"**
2. 用手把两个矢量舵机的舵叶摆到 **正下方（垂直 180°）**
3. 串口或 App 输入 `cal`
4. 看到 **"✓ 校准已保存到 NVS"**，舵机进入正常控制
5. 之后每次上电自动加载，**无需重复校准**

---

## 14. 当前默认配置一览

| 项 | 值 |
|---|---|
| 运行模式 | `MODE_FULL_INTEGRATION` |
| 转向模式 | `STEERING_MODE_PID` |
| IMU 配置 | 单 IMU (`USE_DUAL_IMU = 0`) |
| 控制频率 | 100 Hz (`CONTROL_DT = 0.01`) |
| 倾覆保护 | 开启 (`ENABLE_EMERGENCY_STOP = 1`，阈值 60°) |
| I2C 自救 | 开启 (`ENABLE_I2C_RECOVERY = 1`) |
| 演示模式 | 关闭 (`DEMO_MANUAL_ROLL = 0`) |
| 舵机就绪容差 | `SERVO_TOL_DEG = 15°` |

---

## 15. 调参与排障

### 15.1 上电检查清单

- [ ] IMU 初始化日志正常（无 I2C 超时）
- [ ] 编码器 `valid` 持续为真
- [ ] 看到 "✓ 已从 NVS 加载校准" 或完成首次 `cal`
- [ ] 串口 `f 10` 后推进 PWM 在 1500–1800 µs 区间变化

### 15.2 常见现象

| 现象 | 排查方向 |
|---|---|
| 船体抖动 | 降 `PID_KP` 或 `FEEDBACK_PARAM` |
| 响应偏慢 | 升 `PID_KP` 或 `FEEDFORWARD_PARAM` |
| 舵机滞后 / 跳变 | 检查编码器 PWM 信号、`compute_angle` clamp 是否触发 |
| 推进无响应 | 进入 `MODE_CALIBRATE_ESC` 验证 ESC |
| 推力先动舵机后动 | 已被 Scheme A 解决；如仍出现可调小 `SERVO_TOL_DEG` |
| 上电后舵机不动 | 检查是否输入 `cal` 完成首次校准 |
| 重启后又要校准 | NVS 写入失败 → 看串口 `nvs_save_offsets` 错误码 |

### 15.3 双 IMU 切换

- `USE_DUAL_IMU = 1`，接入第二路 I2C 的 IMU，重新编译烧录

---

## 16. 重要更新日志（相对老 README）

- ✅ V3 矢量分解控制律（替代旧的"反转电机"方案）
- ✅ 启动 IMU 姿态自动归零
- ✅ NVS 持久化舵机零点校准 + 未校准锁 PWM = 1500
- ✅ 舵机就绪门控（Scheme A）解决推力/舵机响应不对称
- ✅ 编码器 clamp 替代 wrap，消除 0/360 边界抖动
- ✅ 串口/App 双通道命令：`r` `f` `cal` + PID 在线调参
- ✅ Blinker timesync 栈 2 KB → 4 KB
- ✅ 取消每次上电自动 `calibrate_encoders()`

---

## 17. 版本说明

本 README 与当前仓库代码一一对应；`raspberry_pi_backup/` 仅作历史参考。
