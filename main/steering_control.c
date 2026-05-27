#include "steering_control.h"
#include "motor_control.h"
#include "system_config.h"
#include "control_params.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#if ENC_USE_SPI
#include "driver/spi_master.h"
#include "freertos/semphr.h"
#endif

static const char *TAG = "STEERING";

// NVS namespace / key
#define STEER_NVS_NS       "steering"
#define STEER_NVS_KEY_OFFL "off_l"
#define STEER_NVS_KEY_OFFR "off_r"

// ============================================================
// 编码器硬件层：PWM 模式 与 SPI 模式 通过 ENC_USE_SPI 切换
// 对外接口：
//   enc_hw_init()          — 初始化硬件
//   enc_hw_read(side)      — 读取原始角度 [0, 360)，失败返回 -1.0f
//   enc_hw_is_valid(side)  — 当前读数是否有效
// ============================================================

#if !ENC_USE_SPI
// ============================================================
// PWM 模式（默认）：边沿中断捕获占空比
// ============================================================

// --- 编码器硬件捕获 (MT6826S PWM 模式) ---
typedef struct {
    uint32_t pin;
    volatile int64_t last_edge_time;
    volatile uint32_t high_us;
    volatile uint32_t period_us;
    volatile bool valid;
    volatile uint8_t bad_streak;     // 连续坏帧计数（去抖：连续 N 次才报无效）
    volatile int64_t last_valid_us;  // 最后一次收到合法帧的时间戳（用于超时断线检测）
    // ===== 原子快照 =====
    // ISR 在每个完整周期（上升沿）结束时，将 high_us/period_us 一次性打包写入 snap_*，
    // 主任务只读 snap_* 而不直接读 high_us/period_us，避免跨周期撕裂读。
    // 用 portMUX_TYPE 保护快照的写入与读取临界区。
    volatile uint32_t snap_high;
    volatile uint32_t snap_period;
    volatile bool     snap_valid;
} encoder_state_t;

static encoder_state_t enc_left  = { .pin = PIN_ENC_LEFT,  .valid = false, .snap_valid = false };
static encoder_state_t enc_right = { .pin = PIN_ENC_RIGHT, .valid = false, .snap_valid = false };

// 编码器快照的 spinlock（ISR 与主任务共用）
static portMUX_TYPE enc_mux = portMUX_INITIALIZER_UNLOCKED;

#else
// ============================================================
// SPI 模式：主动轮询，MT6826S SPI Mode 3 (CPOL=1, CPHA=1)
// 16-bit 帧格式：[15:2]=14bit角度, [1]=OCF(偏移补偿完成), [0]=奇偶校验
// ============================================================
static spi_device_handle_t spi_dev_left  = NULL;
static spi_device_handle_t spi_dev_right = NULL;
// SPI 总线互斥信号量：防止 control_core_task 和 encoder_spi_raw_print_task 并发调用 spi_device_transmit
static SemaphoreHandle_t s_spi_mutex = NULL;
// SPI 模式下编码器有效状态（上次读取结果）
static bool spi_valid_left  = false;
static bool spi_valid_right = false;
// init 期间已经至少读到一次有效数据（独立追踪左右）
static bool spi_present_left  = false;
static bool spi_present_right = false;
// init 完成标志：init 过程中不应用 spi_present 过滤
static bool spi_init_done = false;

#endif  // !ENC_USE_SPI

static float target_left = 180.0f;
static float target_right = 180.0f;

// 编码器初始校准偏移值
static float offset_left = 0.0f;
static float offset_right = 0.0f;


// 是否已经完成"竖直 → 180°"校准（NVS 加载成功 或 用户手动触发过）
static bool s_calibrated = false;

// --- ISR 外部中断处理函数 (仅 PWM 模式) ---
#if !ENC_USE_SPI
static void IRAM_ATTR encoder_isr_handler(void* arg) {
    encoder_state_t* st = (encoder_state_t*) arg;
    int level = gpio_get_level(st->pin);
    int64_t now = esp_timer_get_time();

    // 首次中断： last_edge_time==0 时 delta = now-0 是巨大值，会把
    // high_us / period_us 污染成几十万 us，bad_streak 连增使 valid 反复在 false。
    // 仅记录参考边沿时间，下一次中断才开始计算 delta。
    if (st->last_edge_time == 0) {
        st->last_edge_time = now;
        return;
    }
    int64_t delta = now - st->last_edge_time;

    if (delta < 5) return; // 5us 短期毛刺滤波

    if (level == 0) {
        // 下降沿：此时的 delta 是高电平时间
        st->high_us = (uint32_t)delta;
    } else {
        // 上升沿：此时的 delta 是低电平时间（周期完成）
        st->period_us = (uint32_t)delta;

        uint32_t total = st->high_us + st->period_us;
        if (total >= 100 && total <= 50000 && st->high_us > 0 && st->period_us > 0) {
            st->valid = true;
            st->bad_streak = 0;
            // 整个周期数据就绪，原子写入快照（主任务只读 snap_*）
            portENTER_CRITICAL_ISR(&enc_mux);
            st->snap_high   = st->high_us;
            st->snap_period = st->period_us;
            st->snap_valid  = true;
            portEXIT_CRITICAL_ISR(&enc_mux);
            st->last_valid_us = now;  // 记录最后一次合法帧时间戳
        } else {
            // 去抖：单次坏帧不立即报无效，连续 8 次才翻 valid=false
            // 一旦下一个完整周期正常，会自动恢复 valid=true（自纠正）
            if (st->bad_streak < 255) st->bad_streak++;
            if (st->bad_streak >= 8) {
                st->valid = false;
                portENTER_CRITICAL_ISR(&enc_mux);
                st->snap_valid = false;
                portEXIT_CRITICAL_ISR(&enc_mux);
            }
        }
    }
    st->last_edge_time = now;
}

// 占空比转角度 (0~360)
// MT6826S 编码器：占空比 ~1%~99% 对应 0~360°（数据手册端点附近留有最小高/低脉宽保护）
static float compute_angle(volatile uint32_t high_us, volatile uint32_t period_us, volatile bool valid) {
    if (!valid || period_us == 0) return 0.0f;
    
    uint32_t total_period = high_us + period_us;
    float duty_cycle = (float)high_us / (float)total_period;
    
    // 端点放宽到 MT6826S 真实范围：1% -> 0°, 99% -> 360°
    const float DC_MIN = 0.01f;
    const float DC_MAX = 0.99f;
    const float RANGE = DC_MAX - DC_MIN;  // 98%
    
    float angle = (duty_cycle - DC_MIN) / RANGE * 360.0f;
    
    // 端点饱和（clamp，不再 wrap）：避免边缘抖动跨 0/360 翻转
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 360.0f) angle = 360.0f;
    
    return angle;
}

#else  // ENC_USE_SPI

// SPI 连续读角度寄存器（数据手册 Section 8.6.8）
// MT6826S SPI Mode 3，帧格式：24-bit 命令头 + 流式数据
//
// TX (6字节):
//   Byte0: [1010][A11..A8] = 0xA0  (连续读命令 + 地址高4位)
//   Byte1: [A7..A0]        = 0x03  (寄存器地址 0x003)
//   Byte2~5: dummy 0x00           (提供时钟，接收数据)
//
// RX (6字节):
//   Byte0~1: Hi-Z (命令/地址阶段，忽略)
//   Byte2:   ANGLE[14:7]          (寄存器 0x003)
//   Byte3:   ANGLE[6:0] | 0       (寄存器 0x004，bit0 固定为 0)
//   Byte4:   00000 | STATUS[2:0]  (寄存器 0x005)
//   Byte5:   CRC[7:0]             (寄存器 0x006，可选校验)
//
// 角度公式：θ = ANGLE[14:0] / 32768 * 360°  (15-bit，数据手册 Section 8.6.7)

// CRC-8 (poly=0x07, init=0x00) — 覆盖 rx[2..4]，与 MT6826S 数据手册 Section 8.6.8 一致
static uint8_t calc_crc8(const uint8_t *data, int len) {
    uint8_t crc = 0x00;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
    }
    return crc;
}

static float spi_read_encoder(spi_device_handle_t dev, bool *valid_out) {
    if (dev == NULL) { *valid_out = false; return -1.0f; }
    // init 完成后，如果该侧编码器在 init 期间从未有过有效读数，则表明没有连接，直接跳过
    if (spi_init_done) {
        if (dev == spi_dev_left  && !spi_present_left)  { *valid_out = false; return -1.0f; }
        if (dev == spi_dev_right && !spi_present_right) { *valid_out = false; return -1.0f; }
    }

    static const uint8_t tx_buf[6] = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
    uint8_t rx_buf[6] = {0};

    spi_transaction_t t = {
        .length    = 48,       // 6 字节 = 48 bit
        .tx_buffer = tx_buf,
        .rx_buffer = rx_buf,
    };

    if (s_spi_mutex) xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(dev, &t);
    if (s_spi_mutex) xSemaphoreGive(s_spi_mutex);
    if (ret != ESP_OK) { *valid_out = false; return -1.0f; }

    // CRC 校验：rx[2..4] → 对比 rx[5]
    uint8_t crc_calc = calc_crc8(&rx_buf[2], 3);
    if (rx_buf[5] != crc_calc) {
        ESP_LOGW(TAG, "[ENC %s] CRC mismatch calc=%02X recv=%02X",
                 dev == spi_dev_left ? "L" : "R", crc_calc, rx_buf[5]);
        *valid_out = false;
        return -1.0f;
    }

    uint8_t angle_h = rx_buf[2];        // ANGLE[14:7]
    uint8_t angle_l = rx_buf[3];        // ANGLE[6:0] in bits[7:1], bit0 固定 0
    uint8_t status  = rx_buf[4] & 0x07; // STATUS[2:0]

    // 诊断打印：左右各自计数，避免混淆
    if (dev == spi_dev_left) {
        static int dbg_l = 0;
        if (++dbg_l >= 200) {
            dbg_l = 0;
            uint16_t raw15 = ((uint16_t)angle_h << 7) | (angle_l >> 1);
            ESP_LOGI(TAG, "[ENC L] h=%02X l=%02X STATUS=%d raw15=%u angle=%.1f",
                     angle_h, angle_l, status, raw15, raw15 / 32768.0f * 360.0f);
        }
    } else {
        static int dbg_r = 0;
        if (++dbg_r >= 200) {
            dbg_r = 0;
            uint16_t raw15 = ((uint16_t)angle_h << 7) | (angle_l >> 1);
            ESP_LOGI(TAG, "[ENC R] h=%02X l=%02X STATUS=%d raw15=%u angle=%.1f",
                     angle_h, angle_l, status, raw15, raw15 / 32768.0f * 360.0f);
        }
    }

    // STATUS[1]=1: 磁场过弱；STATUS[2]=1: 供电欠压 → 数据不可信
    if (status & 0x06) { *valid_out = false; return -1.0f; }

    // 重建 15-bit 角度值 ANGLE[14:0]
    uint16_t angle_raw = ((uint16_t)angle_h << 7) | (angle_l >> 1);
    float angle = (float)angle_raw / 32768.0f * 360.0f;
    *valid_out = true;
    return angle;
}

#endif  // !ENC_USE_SPI

// ---------- NVS 校准存取 ----------
static esp_err_t nvs_load_offsets(float *off_l, float *off_r) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(STEER_NVS_NS, NVS_READONLY, &h);
    if (err != ESP_OK) return err;

    union { uint32_t u; float f; } cvt_l, cvt_r;
    err = nvs_get_u32(h, STEER_NVS_KEY_OFFL, &cvt_l.u);
    if (err == ESP_OK) {
        err = nvs_get_u32(h, STEER_NVS_KEY_OFFR, &cvt_r.u);
    }
    nvs_close(h);
    if (err != ESP_OK) return err;

    *off_l = cvt_l.f;
    *off_r = cvt_r.f;
    return ESP_OK;
}

static esp_err_t nvs_save_offsets(float off_l, float off_r) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(STEER_NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) return err;

    union { uint32_t u; float f; } cvt_l = { .f = off_l }, cvt_r = { .f = off_r };
    err = nvs_set_u32(h, STEER_NVS_KEY_OFFL, cvt_l.u);
    if (err == ESP_OK) err = nvs_set_u32(h, STEER_NVS_KEY_OFFR, cvt_r.u);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    return err;
}

void steering_control_init(void) {
#if !ENC_USE_SPI
    // ===== PWM 模式：配置 GPIO 边沿中断 =====
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_ANYEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << PIN_ENC_LEFT) | (1ULL << PIN_ENC_RIGHT),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(PIN_ENC_LEFT, encoder_isr_handler, (void*)&enc_left);
    gpio_isr_handler_add(PIN_ENC_RIGHT, encoder_isr_handler, (void*)&enc_right);

    ESP_LOGI(TAG, "✓ Steering Encoder Interrupts Initialized.");
    ESP_LOGI(TAG, "  - Left Encoder GPIO: %d | Right Encoder GPIO: %d", PIN_ENC_LEFT, PIN_ENC_RIGHT);

    // ===== 等待编码器输出稳定 (防 boot 阶段误以为舵在 180°) =====
    // MT6826S 上电后需要几个 PWM 周期 (~5–20ms) 才输出稳定占空比。
    // 如果不等，enterprise 循环会看到 valid=false，raw 返回 0 、归一化后为 180°，
    // 并以为当前位置就是 180°。这里主动轮询最多 500ms，超时只告警不阻塞启动。
    {
        ESP_LOGI(TAG, "等待编码器输出稳定 (最多 500ms)...");
        TickType_t t0 = xTaskGetTickCount();
        const TickType_t TIMEOUT = pdMS_TO_TICKS(500);
        while (!enc_left.valid || !enc_right.valid) {
            if (xTaskGetTickCount() - t0 > TIMEOUT) {
                ESP_LOGW(TAG, "⚠ 编码器 500ms 内未稳定: L_valid=%d R_valid=%d。启动继续，但初始读数可能不准",
                         (int)enc_left.valid, (int)enc_right.valid);
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (enc_left.valid && enc_right.valid) {
            ESP_LOGI(TAG, "✓ 编码器已稳定 (耗时 %d ms)",
                     (int)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS));
        }
    }

#else  // ENC_USE_SPI
    // ===== SPI 模式：初始化 SPI 总线并添加两个从设备 =====
    spi_bus_config_t bus_cfg = {
        .miso_io_num     = PIN_SPI_MISO,
        .mosi_io_num     = PIN_SPI_MOSI,  // 必须接：发送连续读命令 0xA0 0x03
        .sclk_io_num     = PIN_SPI_SCLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 6,             // 每次 6 字节 = 48 bit
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_DISABLED));

    // 创建 SPI 总线互斥锁（必须在任何任务启动前就绪）
    s_spi_mutex = xSemaphoreCreateMutex();
    configASSERT(s_spi_mutex != NULL);

    // MISO 加软件上拉：编码器未接时 MISO 浮空，上拉后读到全 1，STATUS≠0，稳定报 invalid
    gpio_set_pull_mode(PIN_SPI_MISO, GPIO_PULLUP_ONLY);

    spi_device_interface_config_t dev_cfg = {
        .clock_speed_hz  = ENC_SPI_CLOCK_HZ,
        .mode            = 3,             // MT6826S: CPOL=1, CPHA=1
        .spics_io_num    = PIN_SPI_CS_LEFT,
        .queue_size      = 1,
        .command_bits    = 0,
        .address_bits    = 0,
        .cs_ena_pretrans = 1,             // CSN↓ 后至少 1 个时钟周期才发 SCK（满足 TL≥100ns）
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev_left));

    dev_cfg.spics_io_num = PIN_SPI_CS_RIGHT;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &dev_cfg, &spi_dev_right));

    ESP_LOGI(TAG, "✓ Steering Encoder SPI Initialized (Mode3, %d Hz).", ENC_SPI_CLOCK_HZ);
    ESP_LOGI(TAG, "  - MISO=%d MOSI=%d SCLK=%d CS_L=%d CS_R=%d",
             PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_SCLK, PIN_SPI_CS_LEFT, PIN_SPI_CS_RIGHT);

    // SPI 上电后等待 MT6826S OCF 就绪（通常 < 50ms）
    {
        // ESP_LOGI("SPI", "MT6826S raw: 0x%02X 0x%02X", rx[0], rx[1]);
        ESP_LOGI(TAG, "等待编码器就绪 (最多 500ms)...");
        TickType_t t0 = xTaskGetTickCount();
        const TickType_t TIMEOUT = pdMS_TO_TICKS(500);
        // 左右独立追踪「是否曾经读到过有效值」，避免右侧未接导致左侧超时被错误标记为不存在
        bool ever_vl = false, ever_vr = false;
        while (true) {
            bool vl, vr;
            spi_read_encoder(spi_dev_left,  &vl);
            spi_read_encoder(spi_dev_right, &vr);
            if (vl) ever_vl = true;
            if (vr) ever_vr = true;
            if ((ever_vl && ever_vr) || xTaskGetTickCount() - t0 > TIMEOUT) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (xTaskGetTickCount() - t0 > TIMEOUT) {
            ESP_LOGW(TAG, "⚠ 编码器 500ms 超时: L=%d R=%d", (int)ever_vl, (int)ever_vr);
        } else {
            ESP_LOGI(TAG, "✓ 两侧编码器均就绪");
        }
        // 记录哪侧连接了（不依赖最后一次读取结果，而是「幦是否曾有过」）
        spi_present_left  = ever_vl;
        spi_present_right = ever_vr;
        spi_init_done = true;
        ESP_LOGI(TAG, "编码器在线: L=%d R=%d", (int)spi_present_left, (int)spi_present_right);
    }
#endif  // !ENC_USE_SPI

    // 尝试从 NVS 加载上次保存的零点偏移
    float loaded_l = 0.0f, loaded_r = 0.0f;
    if (nvs_load_offsets(&loaded_l, &loaded_r) == ESP_OK) {
        offset_left  = loaded_l;
        offset_right = loaded_r;
        s_calibrated = true;
        ESP_LOGI(TAG, "✓ 已从 NVS 加载校准: offset_L=%.2f° offset_R=%.2f°", offset_left, offset_right);
    } else {
        s_calibrated = false;
        ESP_LOGW(TAG, "⚠ 未发现校准数据，舵机已锁定 PWM=1500。");
        ESP_LOGW(TAG, "  请把舵机摆到正下方（180° 竖直）并发送 'cal' 命令完成首次校准。");
    }
}

void steering_control_set_target(float target_left_deg, float target_right_deg) {
    // 注意：STEER_TARGET_RIGHT_REVERSE 与 main.c 中 STEER_SEND_RIGHT_REVERSE
    // 的镜像"双重抵消"后，PID 才能让右舵机走到正确位置。
    // 任何调用者都要意识到：写入的 R 值与读出的 R 值 (get_target) 不是同一帧。
    // 门控判据必须使用 get_target() 的返回值，而不是写入的原始 send_tgt_R。
#if STEER_TARGET_LEFT_REVERSE
    target_left  = 360.0f - target_left_deg;
#else
    target_left  = target_left_deg;
#endif
#if STEER_TARGET_RIGHT_REVERSE
    target_right = 360.0f - target_right_deg;
#else
    target_right = target_right_deg;
#endif
}
void steering_control_get_target(float *left_deg, float *right_deg) {
    if (left_deg)  *left_deg  = target_left;
    if (right_deg) *right_deg = target_right;
}
void steering_control_calibrate_and_save(void) {
    // 读取当前编码器的竖直状态值作为校准基准
    float cal_left, cal_right;

#if !ENC_USE_SPI
    // PWM 模式：原子读取快照
    uint32_t snap_high_L, snap_period_L; bool snap_valid_L;
    uint32_t snap_high_R, snap_period_R; bool snap_valid_R;
    portENTER_CRITICAL(&enc_mux);
    snap_high_L   = enc_left.snap_high;
    snap_period_L = enc_left.snap_period;
    snap_valid_L  = enc_left.snap_valid;
    snap_high_R   = enc_right.snap_high;
    snap_period_R = enc_right.snap_period;
    snap_valid_R  = enc_right.snap_valid;
    portEXIT_CRITICAL(&enc_mux);
    cal_left  = compute_angle(snap_high_L, snap_period_L, snap_valid_L);
    cal_right = compute_angle(snap_high_R, snap_period_R, snap_valid_R);
#else
    // SPI 模式：直接同步读取
    bool vl, vr;
    cal_left  = spi_read_encoder(spi_dev_left,  &vl);
    cal_right = spi_read_encoder(spi_dev_right, &vr);
    if (!vl || !vr) {
        ESP_LOGE(TAG, "✗ 校准失败：编码器读取无效 (L=%d R=%d)，放弃本次校准", (int)vl, (int)vr);
        return;
    }
#endif

    // 设置偏移使得初始竖直状态对应 180°（避免 0/360 边界抖动）
    offset_left  = cal_left  - 180.0f;
    offset_right = cal_right - 180.0f;

    s_calibrated = true;
    ESP_LOGI(TAG, "Encoder Calibration Complete. Offset Left: %.2f°, Offset Right: %.2f°", offset_left, offset_right);

    esp_err_t err = nvs_save_offsets(offset_left, offset_right);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "✓ 校准已保存到 NVS（重启后自动加载，无需再次校准）");
    } else {
        ESP_LOGE(TAG, "✗ 保存校准到 NVS 失败: %s", esp_err_to_name(err));
    }
}

bool steering_control_is_calibrated(void) {
    return s_calibrated;
}

// 编码器超时阈值：正常周期 ~5ms~50ms，150ms 内无合法帧视为断线 (仅 PWM 模式)
#define ENC_TIMEOUT_US  150000LL

void steering_control_get_current_angles(float *left_deg, float *right_deg) {
    bool cur_left_valid, cur_right_valid;
    float raw_left, raw_right;

#if !ENC_USE_SPI
    // ===== 超时断线检测 (PWM 模式) =====
    // ISR 断线后不再触发，bad_streak 永远不增，valid 永远停留 true。
    // 主动用时间戳判断：超过 150ms 无合法帧则主动置 valid=false，让 PID 停转。
    // 恢复时 ISR 好帧会重新置 valid=true，播种逻辑自动感知并重新播种。
    //
    // ⚠ 注意：不能在此处写 snap_valid！snap_valid 仅由 ISR 在临界区内写。
    //   主任务在临界区外写 snap_valid=false，会与 ISR 在临界区内写 snap_valid=true 形成竞态：
    //   若主任务的 false 覆盖了 ISR 刚写好的 true，下一帧 compute_angle() 收到 false 返回 0°，
    //   经 offset 归一化后变成 ~180°，错误地播种 filtered_angle=180°，导致重连后位置突变。
    {
        int64_t now = esp_timer_get_time();
        if (enc_left.valid && enc_left.last_valid_us > 0 &&
            (now - enc_left.last_valid_us) > ENC_TIMEOUT_US) {
            enc_left.valid = false;          // 仅改 valid，不碰 snap_*
            ESP_LOGW(TAG, "左编码器超时断线，停转保护");
        }
        if (enc_right.valid && enc_right.last_valid_us > 0 &&
            (now - enc_right.last_valid_us) > ENC_TIMEOUT_US) {
            enc_right.valid = false;         // 仅改 valid，不碰 snap_*
            ESP_LOGW(TAG, "右编码器超时断线，停转保护");
        }
    }
    cur_left_valid  = enc_left.valid;
    cur_right_valid = enc_right.valid;

    // 通过临界区原子读取 ISR 快照，避免读到 high_us/period_us 属于不同周期的数据
    uint32_t snap_high_L, snap_period_L; bool snap_valid_L;
    uint32_t snap_high_R, snap_period_R; bool snap_valid_R;
    portENTER_CRITICAL(&enc_mux);
    snap_high_L   = enc_left.snap_high;
    snap_period_L = enc_left.snap_period;
    snap_valid_L  = enc_left.snap_valid;
    snap_high_R   = enc_right.snap_high;
    snap_period_R = enc_right.snap_period;
    snap_valid_R  = enc_right.snap_valid;
    portEXIT_CRITICAL(&enc_mux);

    raw_left  = compute_angle(snap_high_L, snap_period_L, snap_valid_L);
    raw_right = compute_angle(snap_high_R, snap_period_R, snap_valid_R);

#else  // ENC_USE_SPI
    // SPI 模式：同步读取
    bool raw_left_valid, raw_right_valid;
    raw_left  = spi_read_encoder(spi_dev_left,  &raw_left_valid);
    raw_right = spi_read_encoder(spi_dev_right, &raw_right_valid);

    // 去抖：单次坏帧（EMI 偶发 STATUS≠0 / 磁铁高速旋转短暂弱场）不立即报 invalid，
    // 需连续 SPI_FAIL_THRESH 次失败才翻转 spi_valid_*，
    // 恢复时只要一次成功即立即恢复（不对称设计：快恢复、慢报警）。
    // 20帧 = 200ms @ 100Hz：足以覆盖高速旋转时短暂弱场，真正断线需持续 >200ms 才报警
    // 对称去抖：
    //   断线：连续 SPI_FAIL_THRESH 次失败 → spi_valid=false（慢报警，抗偶发坏帧）
    //   重连：连续 SPI_GOOD_THRESH 次成功 → spi_valid=true （慢恢复，防单帧噪声误触发）
    // 不对称的旧设计（断线慢/重连快）会导致：20坏帧→停→1好帧→立刻重启PID→角度激变→循环。
    #define SPI_FAIL_THRESH 20
    #define SPI_GOOD_THRESH  5
    static uint8_t fail_l = 0, fail_r = 0;
    static uint8_t good_l = 0, good_r = 0;
    if (raw_left_valid) {
        fail_l = 0;
        if (++good_l >= SPI_GOOD_THRESH) {
            good_l = SPI_GOOD_THRESH;   // 防溢出
            spi_valid_left = true;
        }
    } else {
        good_l = 0;
        if (++fail_l >= SPI_FAIL_THRESH) {
            fail_l = SPI_FAIL_THRESH;   // 防溢出
            spi_valid_left = false;
        }
    }
    if (raw_right_valid) {
        fail_r = 0;
        if (++good_r >= SPI_GOOD_THRESH) {
            good_r = SPI_GOOD_THRESH;
            spi_valid_right = true;
        }
    } else {
        good_r = 0;
        if (++fail_r >= SPI_FAIL_THRESH) {
            fail_r = SPI_FAIL_THRESH;
            spi_valid_right = false;
        }
    }
    // 角度更新门控：同时满足「去抖稳定」AND「本帧原始有效」。
    // 重连去抖窗口内（good < 5）spi_valid=false → last_* 不更新，
    // 防止单帧噪声角度在恢复瞬间写入 last_* 引发读数激变。
    cur_left_valid  = spi_valid_left  && raw_left_valid;
    cur_right_valid = spi_valid_right && raw_right_valid;
#endif  // !ENC_USE_SPI

    // 相对于初始校准点的角度
    raw_left  = raw_left  - offset_left;
    raw_right = raw_right - offset_right;

    // 应用编码器方向反转开关（由 system_config.h 集中配置）
    // 以 180° 为镜像中心，源于校准后初始位是 180°
#if ENC_LEFT_REVERSE
    raw_left = 360.0f - raw_left;
#endif
#if ENC_RIGHT_REVERSE
    raw_right = 360.0f - raw_right;
#endif

    // 将原始角度归一化到 [0, 360) 范围
    while (raw_left < 0.0f) raw_left += 360.0f;
    while (raw_left >= 360.0f) raw_left -= 360.0f;
    while (raw_right < 0.0f) raw_right += 360.0f;
    while (raw_right >= 360.0f) raw_right -= 360.0f;

    // 无效帧冻结：保持上一次有效角度，防止 PID 在去抖窗口内看到乱值
    static float last_left  = 180.0f;
    static float last_right = 180.0f;
    if (cur_left_valid)  last_left  = raw_left;
    if (cur_right_valid) last_right = raw_right;
    *left_deg  = last_left;
    *right_deg = last_right;
}

// 获取编码器健康状态
void steering_control_get_encoder_status(bool *left_ok, bool *right_ok) {
#if !ENC_USE_SPI
    *left_ok  = enc_left.valid;
    *right_ok = enc_right.valid;
#else
    *left_ok  = spi_valid_left;
    *right_ok = spi_valid_right;
#endif
}

bool steering_control_spi_read_raw(int side, uint8_t rx_out[6]) {
#if ENC_USE_SPI
    spi_device_handle_t dev = (side == 0) ? spi_dev_left : spi_dev_right;
    if (dev == NULL) return false;

    static const uint8_t tx_buf[6] = {0xA0, 0x03, 0x00, 0x00, 0x00, 0x00};
    spi_transaction_t t = {
        .length    = 48,
        .tx_buffer = tx_buf,
        .rx_buffer = rx_out,
    };
    if (s_spi_mutex) xSemaphoreTake(s_spi_mutex, portMAX_DELAY);
    esp_err_t ret = spi_device_transmit(dev, &t);
    if (s_spi_mutex) xSemaphoreGive(s_spi_mutex);
    return (ret == ESP_OK);
#else
    (void)side; (void)rx_out;
    return false;
#endif
}

// 提取单边 PID 计算
static float calculate_pid(float error, float *integral, float *prev_error, float *out_filt) {
    // PID 增益从全局变量读取（可通过 Blinker 实时调参）
    const float kp = g_steer_kp, ki = g_steer_ki, kd = g_steer_kd, dt = 0.01f;
    const float integral_max = 80.0f;

    // 注：外层 steering_control_update() 已经用 DEADZONE=3° 做了死区门控，
    // 这里不再叠加 smoothstep，避免 4°~6° 小误差被双重削弱后落到 0。
    float err_smooth = error;

    // P 项
    float p_out = kp * err_smooth;

    // D 项
    float derivative = 0.0f;
    if (dt > 0) {
        derivative = (error - *prev_error) / dt;
        if (derivative > 200.0f) derivative = 200.0f;
        if (derivative < -200.0f) derivative = -200.0f;
    }
    *prev_error = error;
    float d_out = kd * derivative;

    // I 项（带 anti-windup：仅在输出未饱和时才累积积分）
    // 避免大误差旋转段积分饱和 → 到达终点时积分爆冲导致振荡
    float pd_out = p_out + d_out;
    float i_candidate = *integral + err_smooth * dt * ki;
    if (i_candidate > integral_max) i_candidate = integral_max;
    if (i_candidate < -integral_max) i_candidate = -integral_max;
    // 只有 PD 输出本身未饱和时才允许积分朝同方向增长（back-calculation anti-windup）
    if (!((pd_out >= 500.0f && i_candidate > *integral) ||
          (pd_out <= -500.0f && i_candidate < *integral))) {
        *integral = i_candidate;
    }
    float i_out = *integral;

    float raw_out = pd_out + i_out;

    // 低通滤波（仅平滑 D 项高频噪声，不应大幅限制 P 项响应速度）
    // τ=0.03f → 截止~5.3Hz @ 100Hz
    float alpha = dt / (0.03f + dt);
    *out_filt = *out_filt + alpha * (raw_out - *out_filt);

    float final_out = *out_filt;
    if (final_out > 500.0f) final_out = 500.0f;
    if (final_out < -500.0f) final_out = -500.0f;
    
    // 输出死区：低于 30 的微弱信号清零，防止电机连续微弱运转损耗。
    // ⚠ 注意：不再做 30→60 的强制跳变。原因：当 PID 输出在 [30,60) 时若强推到 60，
    // 会在死区边缘（误差 2°~4°）产生速度不连续，引起 60→-60 反复振荡（bang-bang 效应）。
    // 去掉后 PID 输出连续变化，由 DEADZONE=3° 的外层门控负责抑制微小误差。
    if (fabsf(final_out) < 30.0f) {
        final_out = 0.0f;
    }
    

    return final_out;
}

// PID 状态
static float integral_left = 0.0f, integral_right = 0.0f;
static float prev_err_left = 0.0f, prev_err_right = 0.0f;
static float out_filt_left = 0.0f, out_filt_right = 0.0f;

void steering_control_update(void) {
    // 未校准 → 强制中立位 PWM=1500（最安全：360° 连续舵机此时不旋转）
    if (!s_calibrated) {
        motor_control_set_steering_pwm(1500, 1500);
        return;
    }

    float cur_left, cur_right;
    steering_control_get_current_angles(&cur_left, &cur_right);

    // 【编码器失效保护】
    // get_current_angles 在 invalid 时只是"冻结上一拍滤波值"，PID 自己感知不到失效。
    // 若不在这里拦截，PID 会按冻结的角度持续喷 PWM，360° 连续舵机会
    // 一路狂转过冲一整圈（实测 270→90 多走 360° = 540°）。
    // 失效时直接 PWM=1500（舵机不转），同时复位该侧 PID 状态，避免恢复后积分爆发。
#if !ENC_USE_SPI
    bool enc_l_ok = enc_left.valid;
    bool enc_r_ok = enc_right.valid;
#else
    bool enc_l_ok = spi_valid_left;
    bool enc_r_ok = spi_valid_right;
#endif

    // 线性误差（不做 ±180° 环形折叠），确保舵机始终沿 [80°, 280°] 弧内运动。
    // 若用 shortest_angle_error：当 target 与 cur 相距 >180°（如 82° vs 278°），
    // 会选择经 0° 的 160° 短路径，驱动舵机穿越禁区。
    // 线性减法：误差最大 ±200°（弧宽），方向始终沿弧，绝不经过 0°/360°。
    float tgt_L = target_left;
    if (tgt_L < 80.0f) tgt_L = 80.0f;
    if (tgt_L > 280.0f) tgt_L = 280.0f;
    float tgt_R = target_right;
    if (tgt_R < 80.0f) tgt_R = 80.0f;
    if (tgt_R > 280.0f) tgt_R = 280.0f;

    float err_L = tgt_L - cur_left;
    float err_R = tgt_R - cur_right;

    const float DEADZONE = 3.0f;
    float adjust_L = 0.0f, adjust_R = 0.0f;

#if STEERING_CONTROL_MODE == STEERING_MODE_PID
    // PID 模式
    if (!enc_l_ok) {
        // 编码器失效：停转 + 清状态，等编码器恢复
        adjust_L = 0.0f;
        integral_left = 0.0f;
        prev_err_left = 0.0f;
        out_filt_left = 0.0f;
    } else if (fabsf(err_L) > DEADZONE) {
        adjust_L = calculate_pid(err_L, &integral_left, &prev_err_left, &out_filt_left);
    } else {
        adjust_L = 0.0f;
        integral_left = 0.0f;
        out_filt_left = 0.0f;  // 清 LPF 残值，避免重新进入 PID 时启动抖动
    }

    if (!enc_r_ok) {
        adjust_R = 0.0f;
        integral_right = 0.0f;
        prev_err_right = 0.0f;
        out_filt_right = 0.0f;
    } else if (fabsf(err_R) > DEADZONE) {
        adjust_R = calculate_pid(err_R, &integral_right, &prev_err_right, &out_filt_right);
    } else {
        adjust_R = 0.0f;
        integral_right = 0.0f;
        out_filt_right = 0.0f;  // 清 LPF 残值，避免重新进入 PID 时启动抖动
    }
#else
    // 直接映射模式（中等响应速度）
    if (enc_l_ok && fabsf(err_L) > DEADZONE) {
        // 降低比例增益与最大转速限幅
        // PWM 范围限制在 1500 ± 150 (即 1200 到 1800)
        adjust_L = err_L * 5.0f; 
        if (adjust_L > 100.0f) adjust_L = 100.0f;
        if (adjust_L < -100.0f) adjust_L = -100.0f;
    }

    if (enc_r_ok && fabsf(err_R) > DEADZONE) {
        adjust_R = err_R * 5.0f;
        if (adjust_R > 100.0f) adjust_R = 100.0f;
        if (adjust_R < -100.0f) adjust_R = -100.0f;
    }
#endif

    uint32_t pwm_L = (uint32_t)(1500.0f - adjust_L);
    uint32_t pwm_R = (uint32_t)(1500.0f - adjust_R);

    motor_control_set_steering_pwm(pwm_L, pwm_R);
}