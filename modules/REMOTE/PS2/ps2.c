/**
 * @file    ps2.c
 * @brief   PS2 无线手柄解码 (REMOTE_SOURCE == 3)
 *
 *  一帧 9 字节, LSB First (主机发 → 手柄回):
 *    [0] 0x01 start   → 0xFF
 *    [1] 0x42 poll    → ID: 0x41=数字 / 0x73=模拟 / 0x79=模拟+振动
 *    [2] 0x00         → 0x5A 固定头
 *    [3] 振动字节     → 按键低字节 (bit=0 表示按下)
 *    [4] 振动字节     → 按键高字节
 *    [5] 0x00         → 右摇杆 X (RX)
 *    [6] 0x00         → 右摇杆 Y (RY)
 *    [7] 0x00         → 左摇杆 X (LX)
 *    [8] 0x00         → 左摇杆 Y (LY)
 *
 *  上电必须进"模拟模式并锁定", 否则手柄可能停在数字模式(摇杆读数固定), 且按 MODE 键会切走模式。
 */
#include "ps2.h"

#include "bsp_def.h"
#include "bsp_spi.h"
#include "spi.h"

#include <stdbool.h>
#include <stdlib.h>

#define LOG_TAG "remote_ps2"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#if defined(REMOTE_SOURCE) && (REMOTE_SOURCE == 3)

#define PS2_FRAME_LEN   9
#define PS2_SPI_TIMEOUT 100 /* BSP SPI 超时(tick, 1 tick = 1ms) */

#define PS2_CMD_START      0x01
#define PS2_CMD_POLL       0x42
#define PS2_ID_DIGITAL     0x41 /* 绿灯, 摇杆读数无效 */
#define PS2_ID_ANALOG      0x73 /* 红灯 */
#define PS2_ID_ANALOG_VIB  0x79
#define PS2_ID_HEADER      0x5A /* Data[2] 固定头, 用于判"数据有效" */

#define PS2_STICK_BIAS 128 /* 摇杆原始中位 */
#define PS2_STICK_MAX  660 /* 归一化量纲, 与 DT7/VT03 保持一致 */

/* 上电配置序列(字节值以 WHEELTEC 例程 pstwo.c 为准) */
static const uint8_t kShortPoll[PS2_FRAME_LEN]   = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kEnterConfig[PS2_FRAME_LEN] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kSetAnalog[PS2_FRAME_LEN]   = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00}; /* 0x03=锁定模拟模式 */
static const uint8_t kExitConfig[PS2_FRAME_LEN]  = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

/* SPI 收发缓冲: 阻塞模式下 BSP 逐字节"先读后写", 但沿用独立两块更保险 */
BUFFER_SECTION static uint8_t ps2_tx[PS2_FRAME_LEN];
BUFFER_SECTION static uint8_t ps2_rx[PS2_FRAME_LEN];

static SPI_Device *ps2_spi;
static bool        ps2_initialized;

/* 一帧收发: CS 由 BSP 包住整帧(transfer_begin 拉低 → transfer_end 拉高) */
static int8_t ps2_transfer(void)
{
    return (BSP_SPI_TransReceive(ps2_spi, ps2_tx, ps2_rx, PS2_FRAME_LEN, PS2_SPI_TIMEOUT) == 0) ? 0 : -1;
}

/* 发一帧固定命令(配置序列用) */
static void ps2_send(const uint8_t cmd[PS2_FRAME_LEN])
{
    for (uint8_t i = 0; i < PS2_FRAME_LEN; i++)
    {
        ps2_tx[i] = cmd[i];
    }
    ps2_transfer();
}

/**
 * @brief 进模拟模式并锁定; 之后读一帧校验 ID
 * @return true=已处于模拟模式
 */
static bool ps2_enter_analog(void)
{
    for (uint8_t attempt = 0; attempt < 3; attempt++)
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            ps2_send(kShortPoll); /* ShortPoll ×3 */
        }
        ps2_send(kEnterConfig);
        ps2_send(kSetAnalog);
        ps2_send(kExitConfig);
        ps2_send(kShortPoll); /* 校验帧 */

        if (ps2_rx[1] == PS2_ID_ANALOG || ps2_rx[1] == PS2_ID_ANALOG_VIB)
        {
            return true;
        }
        LOG_W("analog mode failed (ID=0x%02X), retry %u", ps2_rx[1], (unsigned)(attempt + 1));
    }
    return false;
}

/* 原始摇杆 0~255 → ±660, 带死区 */
static int16_t ps2_stick(uint8_t raw)
{
    int16_t v = (int16_t)raw - PS2_STICK_BIAS;
    if (abs(v) < REMOTE_PS2_STICK_DEADZONE_RAW) return 0;
    return (int16_t)(v * PS2_STICK_MAX / PS2_STICK_BIAS);
}

int8_t remote_ps2_init(Offline_Device **out_offline)
{
    /* PS2 是 mode3 + LSB + 低速; 同 sbus 重配 UART 波特率的做法, 这里直接重配 SPI */
    REMOTE_PS2_SPI.Init.CLKPolarity       = SPI_POLARITY_HIGH;
    REMOTE_PS2_SPI.Init.CLKPhase          = SPI_PHASE_2EDGE;
    REMOTE_PS2_SPI.Init.FirstBit          = SPI_FIRSTBIT_LSB;
    REMOTE_PS2_SPI.Init.NSS               = SPI_NSS_SOFT;
    REMOTE_PS2_SPI.Init.BaudRatePrescaler = REMOTE_PS2_SPI_PRESCALER;
    if (HAL_SPI_Init(&REMOTE_PS2_SPI) != HAL_OK)
    {
        LOG_E("SPI re-init error");
        return -1;
    }

    /* CS 由 BSP 管理, 整帧 9 字节期间保持低 */
    SPI_Device_Init_Config spi_cfg = {0};
    spi_cfg.hspi                   = &REMOTE_PS2_SPI;
    spi_cfg.cs_port                = REMOTE_PS2_CS_PORT;
    spi_cfg.cs_pin                 = REMOTE_PS2_CS_PIN;
    spi_cfg.tx_mode                = SPI_MODE_BLOCKING;
    spi_cfg.rx_mode                = SPI_MODE_BLOCKING;
    ps2_spi                        = BSP_SPI_Device_Init(&spi_cfg);
    if (ps2_spi == NULL)
    {
        LOG_E("BSP SPI device init failed");
        return -1;
    }

    Offline_Init_config_t offline_cfg = {
        .name       = "ps2",
        .timeout_ms = REMOTE_PS2_OFFLINE_TIMEOUT_MS,
        .beep_times = 0,
        .enable     = REMOTE_OFFLINE_ENABLE,
    };
    Offline_Device *offline = Module_Offline_register(&offline_cfg);
    if (offline == NULL)
    {
        LOG_E("offline register error");
        return -1;
    }
    if (out_offline) *out_offline = offline;

    if (!ps2_enter_analog())
    {
        LOG_E("enter analog mode failed, check wiring / 3.3V power / CS pin");
        return -1;
    }

    ps2_initialized = true;
    LOG_I("ps2 init success (analog mode)");
    return 0;
}

void remote_ps2_decode(Remote_Data_t *data, Offline_Device *offline)
{
    if (!ps2_initialized || data == NULL) return;

    /* 组 poll 帧; [3]/[4] 是振动强度, 第一版不做(固定 0 = 不震) */
    ps2_tx[0] = PS2_CMD_START;
    ps2_tx[1] = PS2_CMD_POLL;
    for (uint8_t i = 2; i < PS2_FRAME_LEN; i++)
    {
        ps2_tx[i] = 0x00;
    }

    if (ps2_transfer() != 0) return;

    const uint8_t id = ps2_rx[1];
    /* 非法帧(手柄关机/待机/未配对, 回包多为 0xFF / 0x00) → 不喂心跳, 交给 OFFLINE 判离线 */
    if ((id != PS2_ID_ANALOG && id != PS2_ID_ANALOG_VIB) || ps2_rx[2] != PS2_ID_HEADER) return;

    /* 摇杆 → ±660 (上推为正) */
    data->channels[0] = ps2_stick(ps2_rx[7]); /* ch1 = LX */
    data->channels[2] = ps2_stick(ps2_rx[5]); /* ch3 = RX */
#if REMOTE_PS2_INVERT_Y
    data->channels[1] = (int16_t)-ps2_stick(ps2_rx[8]); /* ch2 = LY, 实测反向时打开 */
    data->channels[3] = (int16_t)-ps2_stick(ps2_rx[6]); /* ch4 = RY */
#else
    data->channels[1] = ps2_stick(ps2_rx[8]); /* ch2 = LY */
    data->channels[3] = ps2_stick(ps2_rx[6]); /* ch4 = RY */
#endif

    /* PS2 专属数据: 按键原始低有效(0=按下), 归一化成 1=按下 */
    data->ps2.buttons = (uint16_t)~(((uint16_t)ps2_rx[4] << 8) | (uint16_t)ps2_rx[3]);
    data->ps2.lx      = ps2_rx[7];
    data->ps2.ly      = ps2_rx[8];
    data->ps2.rx      = ps2_rx[5];
    data->ps2.ry      = ps2_rx[6];
    data->ps2.mode    = id;

    /* 首次收到合法帧时打印原始值, 方便上板确认摇杆方向与中位 */
    static bool s_first_frame = false;
    if (!s_first_frame)
    {
        s_first_frame = true;
        LOG_I("raw: LX=%u LY=%u RX=%u RY=%u buttons=0x%04X", ps2_rx[7], ps2_rx[8], ps2_rx[5], ps2_rx[6], data->ps2.buttons);
    }

    Module_Offline_device_update(offline);
}

#else /* REMOTE_SOURCE != 3: 空桩, 保证其它模式下链接不报错 */

int8_t remote_ps2_init(Offline_Device **out_offline)
{
    (void)out_offline;
    return -1;
}

void remote_ps2_decode(Remote_Data_t *data, Offline_Device *offline)
{
    (void)data;
    (void)offline;
}

#endif
