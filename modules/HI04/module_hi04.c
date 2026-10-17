/*
 * @file    module_hi04.c
 * @brief   HI04(HiPNUC HI91) 外置 IMU —— UART 921600 + DMA 循环收 + 逐字节解码
 *
 * 数据流: USART1(PB7) --DMA2_Stream5 循环--> BSP_UART 的 kfifo
 *         --> 本任务 BSP_UART_Read() 取一批 --> hipnuc_input() 逐字节解码
 *         --> 命中 HIPNUC_ID_HI91 就更新 hi04_data
 *
 * 为什么一个任务里要抽干 FIFO: 921600bps ≈ 92KB/s, 2ms 周期就有 ~184 字节,
 * 单次只读 128 字节会积压并把环形缓冲撑爆。
 */
#include "module_hi04.h"

#include "bsp_def.h" /* APPS_STACK_SECTION 等板级宏 */
#include "bsp_uart.h"
#include "hipnuc_dec.h"
#include "tx_api.h"
#include "ulog_def.h"
#include <string.h>

/* huart1 的声明(CubeMX 生成, dji_c 在 Inc/usart.h) */
#include "usart.h"

#define LOG_TAG "module_hi04"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

/* DMA 环形缓冲, BSP_UART 要求 2 的幂。1024 字节 ≈ 11ms 的数据量。 */
static uint8_t hi04_dma_buf[1024];

static UART_Device *hi04_uart = NULL;
static hipnuc_raw_t hi04_decoder;
static Hi04_Data_t  hi04_data;

static TX_THREAD                  hi04_thread;
APPS_STACK_SECTION static uint8_t hi04_thread_stack[HI04_TASK_STACK_SIZE];

/* 逐字节喂解码器; 解出完整 HI91 帧才更新对外数据 */
static void hi04_parse_byte(uint8_t byte)
{
    if (hipnuc_input(&hi04_decoder, byte) <= 0) return;

    /* hipnuc_input() 返回 1 时, 恰好有一个 tag 非零的子包有效 */
    if (hi04_decoder.hi91.tag != HIPNUC_ID_HI91) return;

    hi04_data.roll_deg  = hi04_decoder.hi91.roll;
    hi04_data.pitch_deg = hi04_decoder.hi91.pitch;
    hi04_data.yaw_deg   = hi04_decoder.hi91.yaw;

    for (uint8_t i = 0; i < 3; i++)
    {
        hi04_data.acc_g[i] = hi04_decoder.hi91.acc[i];
        /* HI91 原始角速度是 deg/s */
        hi04_data.gyro_rad_s[i] = hi04_decoder.hi91.gyr[i] * HIPNUC_DEG2RAD;
    }
    for (uint8_t i = 0; i < 4; i++) hi04_data.quat[i] = hi04_decoder.hi91.quat[i];

    hi04_data.device_time_ms = hi04_decoder.hi91.system_time;
    hi04_data.frame_count++;
    hi04_data.received = 1;
}

static void hi04_task(ULONG arg)
{
    (void)arg;

    uint8_t  chunk[128];
    uint32_t n = 0;

    while (1)
    {
        /* 一次任务把 FIFO 抽干, 避免 921600 下积压 */
        while (BSP_UART_Read(hi04_uart, chunk, sizeof(chunk), &n, 0) > 0 && n > 0)
        {
            for (uint32_t i = 0; i < n; i++) hi04_parse_byte(chunk[i]);
        }

        hi04_data.crc_errors     = hi04_decoder.crc_error_count;
        hi04_data.invalid_frames = hi04_decoder.invalid_count;

        /* 【调试】500ms 打一次三轴角速度 + 欧拉角。
         * 用途: HI04 装在俯仰框上, 安装方向未知 —— 手动转 pitch, 看哪个分量的
         * rad/s 跟着变、正负怎么对应, 确定后删掉这段。 */
        static uint16_t log_div;
        if (++log_div >= 250) /* 2ms × 250 = 500ms */
        {
            log_div = 0;
            LOG_I("f=%u gyro=%.3f/%.3f/%.3f rad/s | rpy=%.1f/%.1f/%.1f deg | crc=%u bad=%u", (unsigned)hi04_data.frame_count,
                  (double)hi04_data.gyro_rad_s[0], (double)hi04_data.gyro_rad_s[1], (double)hi04_data.gyro_rad_s[2], (double)hi04_data.roll_deg,
                  (double)hi04_data.pitch_deg, (double)hi04_data.yaw_deg, (unsigned)hi04_data.crc_errors, (unsigned)hi04_data.invalid_frames);
        }

        tx_thread_sleep(2);
    }
}

void Module_HI04_init(void)
{
    UART_Device_init_config cfg = {
        .huart           = &huart1, /* dji_c: USART1 / PA9 TX / PB7 RX, 921600 (base.ioc 已配) */
        .rx_buf          = hi04_dma_buf,
        .rx_buf_size     = sizeof(hi04_dma_buf),
        .expected_rx_len = 0, /* 不定长: HI91 帧长不固定 */
        .rx_mode         = UART_MODE_DMA,
        .tx_mode         = UART_MODE_BLOCKING,
    };

    hi04_uart = BSP_UART_Device_Init(&cfg);
    if (hi04_uart == NULL)
    {
        LOG_E("uart init failed");
        return;
    }

    memset(&hi04_decoder, 0, sizeof(hi04_decoder));
    memset(&hi04_data, 0, sizeof(hi04_data));

    if (tx_thread_create(&hi04_thread, "hi04_thread", hi04_task, 0, hi04_thread_stack, HI04_TASK_STACK_SIZE, HI04_TASK_PRIORITY, HI04_TASK_PRIORITY,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("thread create failed");
        return;
    }

    LOG_I("HI04 init success (USART1 @921600)");
}

const Hi04_Data_t *Module_HI04_get(void) { return hi04_data.received ? &hi04_data : NULL; }
