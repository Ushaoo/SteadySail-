#ifndef BNO055_DRIVER_H
#define BNO055_DRIVER_H

#include "esp_err.h"

// BNO055 I2C 地址（ADR 引脚悬空 = 0x29，接 GND = 0x28）
#define BNO055_I2C_ADDR   0x29

/**
 * @brief 初始化 BNO055（使用 I2C0，GPIO 8/9，复用原 MPU6050 接口）
 *        上电后进入 NDOF 融合模式（加速度计 + 陀螺仪 + 磁力计）。
 *        NDOF 模式下偏转角由磁力计持续修正，不会积分漂移。
 *        注意：I2C0 总线由本驱动负责初始化，不依赖 imu_driver。
 * @return ESP_OK 成功，其他値失败（芯片 ID 校验错 / I2C 不通）
 */
esp_err_t bno055_init(void);

/**
 * @brief 读取当前绝对偏转角（heading）
 * @param[out] heading_deg  0.0 ~ 360.0°，磁北方向为 0°，顺时针增大
 * @return ESP_OK 成功
 */
esp_err_t bno055_get_heading(float *heading_deg);

/**
 * @brief 读取横滚角（Roll）
 * @param[out] roll_deg  -180.0 ~ +180.0°（NDOF 模式片上融合，不漂移）
 * @return ESP_OK 成功
 */
esp_err_t bno055_get_roll(float *roll_deg);

/**
 * @brief 读取 X 轴角速度（与 Roll 对应轴，单位 °/s）
 * @param[out] gyrox_dps  角速度，正负号表示方向
 * @return ESP_OK 成功
 */
esp_err_t bno055_get_gyro_x(float *gyrox_dps);

/**
 * @brief 读取线性加速度（已去重力，body 系：X=船头，Y=左舷，Z=向上）
 *        NDOF 模式 UNIT_SEL bit0=0 时，单位 1/100 m/s²，换算后输出 m/s²
 * @param[out] ax  X 轴加速度 m/s²
 * @param[out] ay  Y 轴加速度 m/s²
 * @param[out] az  Z 轴加速度 m/s²
 * @return ESP_OK 成功
 */
esp_err_t bno055_get_linear_accel(float *ax, float *ay, float *az);

/**
 * @brief 读取 BNO055 的实时校准等级（每项 0~3，3=完全校准）
 * @param[out] sys/gyr/acc/mag  各子系统校准等级，传 NULL 可忽略
 * @return ESP_OK 成功
 */
esp_err_t bno055_get_calib_status(uint8_t *sys, uint8_t *gyr, uint8_t *acc, uint8_t *mag);

/**
 * @brief 读取当前 22 字节校准 profile（寄存器 0x55~0x6A）。
 *        调用方必须保证芯片已处于 CONFIG 模式。
 */
esp_err_t bno055_read_calib_profile(uint8_t buf[22]);

/**
 * @brief 把 22 字节校准 profile 写回芯片。必须在 CONFIG 模式下调用。
 */
esp_err_t bno055_write_calib_profile(const uint8_t buf[22]);

/**
 * @brief 一键：读当前 calib profile → 存 NVS。
 *        内部自动 CONFIG ↔ NDOF 切换。下次上电由 bno055_init() 自动恢复。
 *        建议先让用户做 8 字晃动使 Mag/Sys = 3 后再调用。
 */
esp_err_t bno055_calibrate_and_save(void);

/**
 * @brief 启动时从 NVS 加载 calib profile 并写回芯片。
 * @return ESP_OK 成功；ESP_ERR_NOT_FOUND 表示 NVS 中无数据。
 */
esp_err_t bno055_load_calib_from_nvs(void);

#endif // BNO055_DRIVER_H
