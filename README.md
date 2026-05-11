# SteadySail (ESP32-S3) 当前工程说明

本 README 对应当前仓库中的 ESP32-S3 主控代码实现，重点覆盖：

- `main/`
- `components/Blinker/`（IoT 远程控制）
- `CMakeLists.txt` / `main/CMakeLists.txt`
- `sdkconfig` / `config/`

说明：`raspberry_pi_backup/` 是历史 Python 备份，不属于当前控制链路。

---

## 1. 项目概览

SteadySail 是一个基于 ESP32-S3 的双推进器 + 双矢量舵机姿态稳定控制系统，目标是在帆船/小艇上实现自动横滚稳定与航向保持。当前实现采用：

- 主循环 100 Hz（10 ms 固定周期）
- **BNO055** IMU（片上 NDOF 融合，绝对横滚 + 绝对偏航，无积分漂移）
- MT6826S 磁编码器 **SPI 模式**（两路独立 CS，抗 EMI，2 MHz）
- LEDC 50 Hz 14-bit PWM 驱动转向舵机（小电机）与推进 ESC（大电机）
- V3 矢量分解控制律：竖直分量反相、水平分量同相、**电机始终同向不反转**
- H=0 静止平衡路径：无前进推力时舵机锁 180°，双电机正反转产生上下力差
- 差速转向：左右水平分量差（`TURN_DELTA_H`）驱动偏航
- **航向保持**：BNO055 绝对偏航角 PI 控制，RC 定速时自动锁定航向
- RC 遥控输入（标准 1000–2000 µs PWM）+ **定速巡航**（拨杆保持 2 s 后松手定速）
- NVS 持久化舵机零点校准（一次校准、永久保存）
- Blinker IoT 双向遥控 + 串口命令双通道
- 双级急停：软急停（`g_estop_active`）+ 硬急停（`g_hard_estop`，直接压 1500 µs）

核心目标：在给定前进推力的同时维持横滚稳定，并允许通过 App / 串口 / RC 遥控实时干预。

---

## 2. 代码结构

### 顶层

- `CMakeLists.txt`：ESP-IDF 工程入口，项目名 `version_2`
- `main/CMakeLists.txt`：注册以下源文件，依赖 `Blinker json nvs_flash`
  - `main.c`、`imu_driver.c`、`balance_controller.c`
  - `motor_control.c`、`steering_control.c`
  - `control_params.c`、`blinker_bridge.c`
  - `bno055_driver.c`、`rc_input.c`

### 主功能模块（`main/`）

- [main.c](main/main.c)
  - 系统初始化、按模式分发任务
  - 100 Hz `control_core_task`：V3 矢量控制 + H=0 静止路径 + 差速转向 + 航向保持 + 安全保护
  - RC 输入读取（`rc_input_get_throttle()`），定速巡航激活时同步锁定航向
  - 串口命令解析（`w/s/space/数字/r/f/cal`）
- [system_config.h](main/system_config.h)
  - 运行模式、转向控制模式
  - GPIO 映射（I2C/SPI/PWM 输入输出/RC 输入）
  - PWM 安全限幅、方向反转开关、目标角镜像开关
  - 差速转向参数（`TURN_DELTA_H`、`TURN_MIN_FWD_PCT`）
  - 航向保持 PID 参数（`HEADING_KP`、`HEADING_KI`）
  - 安全保护与控制基础参数
- [bno055_driver.c](main/bno055_driver.c) / [.h](main/bno055_driver.h)
  - BNO055 I2C 驱动（NDOF 融合模式，I2C0 GPIO 8/9）
  - 读取 Roll（横滚）、偏航（Heading 0~360°，磁北参考）、X 轴角速度
- [imu_driver.c](main/imu_driver.c) / [.h](main/imu_driver.h)
  - MPU6050 备用驱动（当 `USE_BNO055_FOR_ROLL=0` 时启用）
  - 单/双 IMU 数据统一接口（`dual_imu_data_t`）
- [balance_controller.c](main/balance_controller.c) / [.h](main/balance_controller.h)
  - BNO055 路径：直接调用 `bno055_get_roll()` / `bno055_get_gyro_x()`，无软件融合负担
  - MPU6050 路径（备用）：Mahony 四元数姿态估计 + 陀螺仪零偏自动校准
  - 前馈 + 2DOF PID 力矩计算，输出 `tau_total`
  - 角度死区平滑（硬死区 1°，软死区 3°，Hermite 过渡）
- [steering_control.c](main/steering_control.c) / [.h](main/steering_control.h)
  - **SPI 模式**：通过 MT6826S 连续读命令（0xA0 0x03）采集 14-bit 绝对角度
  - 角度 clamp、ENC_*_REVERSE 开关、NVS 零点偏移补偿
  - PID 闭环输出（DEADZONE = 3°，最小输出阈值）
  - **NVS 持久化校准**：自动加载 / 串口 `cal` 写入；未校准时锁 PWM = 1500
- [motor_control.c](main/motor_control.c) / [.h](main/motor_control.h)
  - LEDC 初始化、PWM 输出
  - 双向推力 `motor_control_set_pwm_bidirectional()`（H=0 静止路径专用）
  - 矢量推力 `motor_control_set_pwm_vector()`（V3 路径）
  - ESC 校准任务、紧急停推
- [rc_input.c](main/rc_input.c) / [.h](main/rc_input.h)
  - GPIO 双边沿中断测量标准 RC PWM 脉宽（1000–2000 µs）
  - 死区 ±30 µs，信号超时 200 ms 自动归零
  - **定速巡航状态机**：拨杆稳定 2 s 后松手触发，`rc_input_is_cruising()` 查询
  - 定速巡航激活/取消时联动 `g_heading_hold_active`
- [control_params.c](main/control_params.c) / [.h](main/control_params.h)
  - 全局可调参数（`g_balance_kp/ki/kd`、`g_steer_kp/ki/kd`）
  - NVS Flash 初始化
- [blinker_bridge.c](main/blinker_bridge.c) / [.h](main/blinker_bridge.h)
  - Blinker IoT 任务封装（Wi-Fi 连接后启动）
  - 解析 App 文本命令：PID 调参 / `r <角度>` 演示 Roll / `f <推力%>`
  - 三按钮差速转向（`turn_l` / `turn_f` 航向锁定 / `turn_r`）
  - 1 Hz 上报实时 Roll 角与推力百分比

### 第三方组件

- [components/Blinker](components/Blinker)：基于乐鑫 esp-idf-blinker 的 Blinker 客户端
  - `blinker_timesync` 任务栈已上调到 4 KB，解决 SNTP 栈溢出

---

## 3. 运行模式

在 [system_config.h](main/system_config.h) 中通过 `CURRENT_RUN_MODE` 选择：

| 宏 | 值 | 说明 |
|---|---|---|
| `MODE_TEST_SENSORS` | 0 | 仅打印 IMU 与编码器 |
| `MODE_TEST_STEERING_ONLY` | 1 | 仅舵机闭环测试（含响应时间统计） |
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
| BNO055 / IMU0 I2C SDA / SCL | GPIO 8 / 9 |
| IMU1 I2C SDA / SCL（预留） | GPIO 10 / 11 |
| 编码器 SPI MISO / SCLK | GPIO 5 / 4 |
| 编码器 SPI MOSI（共用） | GPIO 15 |
| 编码器 SPI CS 左 / 右 | GPIO 6 / 3 |
| 转向舵机 PWM 左 / 右 | GPIO 2 / 1 |
| 推进 ESC PWM 左 / 右 | GPIO 18 / 19 |
| RC 油门 PWM 输入 | GPIO 7 |
| 磁控开关（硬急停） | GPIO 12（内部上拉，NO 型干簧管） |

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
| `ENC_LEFT_REVERSE` / `ENC_RIGHT_REVERSE` | 0 / 1 | 编码器读数翻转 |
| `STEER_SEND_LEFT_REVERSE` / `STEER_SEND_RIGHT_REVERSE` | 1 / 1 | main.c 下发目标角镜像 |
| `STEER_TARGET_LEFT_REVERSE` / `STEER_TARGET_RIGHT_REVERSE` | 0 / 1 | steering_control 内部目标角镜像 |

---

## 6. 主控制流程（FULL_INTEGRATION）

`control_core_task()` 100 Hz：

1. **RC 输入**：`rc_input_get_throttle()` 更新 `g_forward_thrust`；检测定速巡航状态，激活时同步锁定 BNO055 当前偏航角（`g_heading_hold_active = true`）
2. 读取舵机当前角度（MT6826S SPI 编码器）
3. **BNO055 路径**：`balance_controller_update()` 直接从 BNO055 读取 Roll + GyroX → PID → `tau_total`
4. 安全保护：`|roll| > 60°` → 紧急停推 + 舵机回 180°；**磁控急停**（`g_hard_estop = true`）→ 全通道锁 1500 µs，等待磁铁重新吸合 500 ms 后自动重启
5. **H=0 静止路径**：`|H_thrust| < 5`（无前进推力）→ 舵机锁 180°，双电机正反转产生上下力差
6. **V3 矢量分解**（详见 §8）→ `target_angle_L/R` + `T_thrust_L/R`
7. **差速转向 / 航向保持**：`g_turn_state` 或 BNO055 偏航 PI 修正 `H_L / H_R`
8. 目标角做环形低通（`filter_target += 0.1 * Δ`，对 360° 取模）
9. **L/R 物理输出交换**：`send_tgt_L = 360 − filter_target_R`
10. `motor_control_set_pwm_bidirectional()` 下发
11. `vTaskDelayUntil()` 保持 10 ms 周期

调试输出 ~10 Hz。

---

## 7. 姿态估计与控制律（balance_controller.c）

### 7.1 BNO055 路径（当前默认，`USE_BNO055_FOR_ROLL = 1`）

- 调用 `bno055_get_roll()` / `bno055_get_gyro_x()` 直接获取片上 NDOF 融合结果
- 不运行 Mahony，无软件融合负担，**无积分漂移**
- 偏航由磁力计持续修正，可作为航向保持参考

### 7.2 MPU6050 路径（备用，`USE_BNO055_FOR_ROLL = 0`）

1. 累计 200 样本求陀螺仪零偏均值
2. Mahony 融合（单 IMU `Kp = 30.0`）收敛后捕获姿态 offset
3. 所有输出欧拉角减去 offset 并归一化到 ±180°

### 7.3 控制律

- 前馈：重力扰动 + 惯量项 + 虚拟刚度（`K_SELF = 100`）
- 反馈：2DOF PID（积分限幅）
- `tau_total = FEEDFORWARD_PARAM × tau_ff − FEEDBACK_PARAM × tau_pid`
- 角度死区：硬死区 1°（输出 = 0），软死区 1°–3°（Hermite 平滑过渡）

关键参数：`PID_KP=20`、`PID_KI=1`、`PID_KD=0`、`FEEDFORWARD_PARAM=0.28`、`FEEDBACK_PARAM=0.5`。

---

## 8. V3 矢量分解控制律

[main.c](main/main.c) `control_core_task()`：

```text
dV = tau_total × THRUST_SCALE     // 竖直分量（带符号）
V_R = +dV   V_L = −dV             // 两侧反相
H_L / H_R = H_thrust ± DELTA_H/2  // 差速转向时不等
phi_R = atan2(H_R, V_R)           // 右舵目标角
phi_L = atan2(H_L, V_L)           // 左舵目标角
T_L = sqrt(V_L² + H_L²)           // 左推力幅值
T_R = sqrt(V_R² + H_R²)           // 右推力幅值
```

**电机始终同向不反转**；靠舵机跨 180° 实现推/拉方向切换。`THRUST_SCALE = 0.55`。

### H=0 静止平衡路径

`|H_thrust| < 5` 时跳过 V3，改为：
- 舵机强制锁 180°（中立位）
- `motor_control_set_pwm_bidirectional(|dV|, |dV|, inv_L, inv_R)` 直接用电机正反转产生上下力差
- 该路径支持船体**静止时**的横滚稳定，无需前进推力

---

## 9. 转向闭环与编码器（steering_control.c）

- **MT6826S SPI 模式**（`ENC_USE_SPI = 1`）：两路编码器共用 MISO/MOSI/SCLK，各独立 CS
  - 连续读命令 `0xA0 0x03`，14-bit 绝对角度，2 MHz
  - 抗 EMI 能力优于 PWM 捕获模式
- 角度 clamp + `ENC_*_REVERSE` 开关 + NVS 零点偏移补偿
- PID 闭环：`DEADZONE = 3°`，最小有效输出阈值
- `set_target()` 内 `STEER_TARGET_*_REVERSE` 翻转，对应硬件方向

### 9.1 NVS 持久化校准

- NVS 命名空间 `"steering"`，键 `off_l` / `off_r`（float 以 u32 存储）
- 上电自动加载：**成功** → 正常工作；**失败** → PWM 锁 1500（连续舵机静止，最安全）
- `steering_control_calibrate_and_save()`：把当前 SPI 编码器读数标定为 180°，写入 NVS
- 触发方式：把舵机摆到正下方，串口或 App 输入 `cal`

### 9.2 编码器健康

`valid` 状态判断 + 主循环周期打印故障/恢复日志。

---

## 10. 推进输出与安全（motor_control.c）

- `motor_control_set_pwm_bidirectional(push_L, push_R, invert_L, invert_R)`
- 输入 → 限幅 → 低通滤波 → PWM
- 紧急停止：双路回 1500 µs

ESC 校准模式 (`MODE_CALIBRATE_ESC`) 循环扫描 1000 → 2000 → 1000 µs。

### 10.1 磁控开关硬急停

#### 硬件设计

- 器件：**NO 型干簧管**（常开，磁铁靠近时闭合）
- 引脚：**GPIO 12**，内部上拉（`GPIO_PULLUP_ENABLE`）
- 信号逻辑：
  - 磁铁在位（正常）→ 干簧管闭合 → GPIO **LOW**
  - 磁铁移走（触发）→ 干簧管断开 → GPIO **HIGH**
- 配置常量（[system_config.h](main/system_config.h)）：
  - `PIN_MAG_ESTOP = 12`
  - `MAG_ESTOP_TRIGGER_LEVEL = 1`（HIGH 触发，对应 NO 型）

#### 中断与标志

- `mag_estop_isr()`（`IRAM_ATTR`）：任意边沿触发，检测到 `gpio_get_level() == MAG_ESTOP_TRIGGER_LEVEL` 时置 `g_hard_estop = true`
- `mag_estop_init()`：上电时在 `app_main()` 中初始化，注册中断

#### 控制任务处理逻辑

`control_core_task()` 100 Hz 循环最高优先级分支：

```c
if (g_hard_estop) {
    // 1. 首次触发打印一次（s_hard_estop_logged 防重复）
    // 2. 推进 ESC 压到 1500 µs（motor_control_emergency_stop）
    // 3. 舵机压到 1500 µs（motor_control_set_steering_pwm(1500, 1500)）
    // 4. 轮询 GPIO：连续 50 帧（500 ms）检测磁铁重新吸合
    // 5. 确认恢复 → 打印日志 → 延迟 1 s → esp_restart()
}
```

- **急停期间所有 4 路输出均锁定 1500 µs**（推进器停转，连续舵机静止）
- 磁铁重新吸合后不直接恢复控制，而是通过 **软件重启** 保证系统进入干净初始态
- `s_recovery_count` 计数器确保消抖：需连续 500 ms 均检测到磁铁在位才触发重启

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
  - `kp / ki / kd`：平衡 PID 在线调参
  - `r <角度>`：演示模式 Roll
  - `f <推力%>`：推力百分比
- 三按钮差速控制：
  - **turn_l**：左转（`g_turn_state = -1`），解除航向保持
  - **turn_f**：锁定当前 BNO055 偏航角（`g_heading_hold_active = true`）
  - **turn_r**：右转（`g_turn_state = +1`），解除航向保持
- 1 Hz 上报实时 Roll 角与当前推力百分比
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
| IMU 配置 | BNO055 NDOF (`USE_BNO055_FOR_ROLL = 1`，`USE_DUAL_IMU = 0`) |
| 编码器模式 | SPI (`ENC_USE_SPI = 1`，2 MHz) |
| 控制频率 | 100 Hz (`CONTROL_DT = 0.01`) |
| 倾覆保护 | 开启 (`ENABLE_EMERGENCY_STOP = 1`，阈值 60°) |
| I2C 自救 | 开启 (`ENABLE_I2C_RECOVERY = 1`) |
| 演示模式 | 关闭 (`DEMO_MANUAL_ROLL = 0`) |
| 差速转向 dH | `TURN_DELTA_H = 100`（量程 0–500） |
| 航向保持 PID | `HEADING_KP = 8.0`，`HEADING_KI = 0.05` |
| RC 定速巡航 | 拨杆保持 2 s 后松手激活 |
| 磁控急停引脚 | GPIO 12（NO 型干簧管，内部上拉，HIGH 触发） |
| 急停恢复方式 | 磁铁重新吸合 500 ms → `esp_restart()` 软件重启 |

---

## 15. 调参与排障

### 15.1 上电检查清单

- [ ] IMU 初始化日志正常（无 I2C 超时）
- [ ] 编码器 `valid` 持续为真
- [ ] 看到 "✓ 已从 NVS 加载校准" 或完成首次 `cal`
- [ ] 串口 `f 10` 后推进 PWM 在 1500–1800 µs 区间变化
- [ ] 磁控开关磁铁在位时串口无急停日志；移走磁铁后出现 `⛔ 磁控开关断开` 并输出锁定 1500 µs

### 15.2 常见现象

| 现象 | 排查方向 |
|---|---|
| 船体抖动 | 降 `PID_KP` 或 `FEEDBACK_PARAM` |
| 响应偏慢 | 升 `PID_KP` 或 `FEEDFORWARD_PARAM` |
| 舵机滞后 / 跳变 | 检查 SPI 线路与 CS 接线，确认 `ENC_*_REVERSE` 设置正确 |
| 推进无响应 | 进入 `MODE_CALIBRATE_ESC` 验证 ESC |
| 上电后舵机不动 | 检查是否输入 `cal` 完成首次校准 |
| 重启后又要校准 | NVS 写入失败 → 看串口 `nvs_save_offsets` 错误码 |
| 航向保持漂移 | BNO055 磁力计校准不足，在空旷处进行"8字"校准手势 |
| RC 推力归零不灵 | 确认 RC 中位死区 ±30 µs；若信号超时则检查接收机供电 |
| 定速巡航意外激活 | RC 拨杆机械卡在固定位置 2 s 触发，属正常；可调大 `RC_CRUISE_SETTLE_US` |
| BNO055 初始化失败 | 检查 I2C0 (GPIO 8/9) 接线，确认 BNO055 地址 `0x29`（ADR 悬空） |
| 上电即急停（磁铁在位却触发） | 确认使用 NO 型干簧管；检查 `MAG_ESTOP_TRIGGER_LEVEL = 1`；用万用表确认磁铁在位时 GPIO 12 为 LOW |
| 移走磁铁无反应 | 确认 GPIO 12 上拉已使能；检查干簧管是否为 NC 型（应换 NO 型或将 `MAG_ESTOP_TRIGGER_LEVEL` 改为 0） |
| 急停后不自动重启 | 磁铁重新吸合后需保持 500 ms 稳定（干簧管抖动会重置计数），属正常消抖行为 |

### 15.3 回退到 MPU6050

将 `USE_BNO055_FOR_ROLL = 0` 并接入 MPU6050，重新编译即可切换到软件 Mahony 融合路径。

---

## 16. 重要更新日志（相对老 README）

- ✅ **BNO055 替代 MPU6050**：片上 NDOF 融合，绝对横滚无漂移，偏航磁力计持续修正
- ✅ **MT6826S SPI 模式**：替代 PWM 捕获，抗 EMI，14-bit 高精度
- ✅ **RC 遥控输入**（GPIO 7）：标准 1000–2000 µs，映射为推力百分比
- ✅ **定速巡航**：RC 拨杆保持 2 s 后松手触发，自动保持推力
- ✅ **航向保持**：BNO055 绝对偏航 PI 控制；定速巡航时自动锁定
- ✅ **H=0 静止平衡路径**：无前进推力时舵机锁 180°，电机正反转维持横滚稳定
- ✅ **差速转向**：Blinker 三按钮驱动左右 H 不等量产生偏航
- ✅ **双级急停**：软急停（`g_estop_active`，倾覆保护）+ 硬急停（`g_hard_estop`，全通道压 1500 µs）
- ✅ **磁控开关硬急停**：GPIO 12 NO 型干簧管，移走磁铁立即锁定所有输出 1500 µs；磁铁重新吸合 500 ms 后自动软件重启
- ✅ V3 矢量分解控制律（替代旧的"反转电机"方案）
- ✅ NVS 持久化舵机零点校准 + 未校准锁 PWM = 1500
- ✅ 编码器 clamp 替代 wrap，消除 0/360 边界抖动
- ✅ 串口/App 双通道命令：`r` `f` `cal` + PID 在线调参
- ✅ Blinker timesync 栈 2 KB → 4 KB

---

## 17. 版本说明

本 README 与当前仓库代码一一对应；`raspberry_pi_backup/` 仅作历史参考。
