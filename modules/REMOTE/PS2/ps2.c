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
 *  时序: 纯 GPIO bit-bang。空闲 CLK=高; 下降沿双方输出本位, 上升沿双方采样
 *        (等价 SPI 模式3 + LSB First); 每字节之间留 ~16us —— PS2 协议要求,
 *        连续 clock 会让接收器的字节位置飘
 *
 *  引脚: 4 个脚(CLK/DO/DI/CS)由**板级 CubeMX** 配好, 本模块只翻转电平。
 *        CLK/DO/CS = 推挽输出初始高;  DI = 上拉输入。
 *        必须与 REMOTE_PS2_{CLK,DO,DI,CS}_{PORT,PIN} 保持一致。
 *
 *  上电必须进"模拟模式并锁定", 否则手柄可能停在数字模式(摇杆读数固定), 且按 MODE 键会切走模式。
 */
#include "ps2.h"

#include "bsp_def.h"
#include "bsp_dwt.h"
#include "main.h"

#include <stdbool.h>
#include <stdlib.h>

#define LOG_TAG "remote_ps2"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

#if defined(REMOTE_SOURCE) && (REMOTE_SOURCE == 3)

#define PS2_FRAME_LEN 9

/* 每次重试握手前的间隔(ms): 给接收器留 MCU 启动 + 无线配对的时间 */
#define PS2_HANDSHAKE_RETRY_MS REMOTE_PS2_POWER_ON_DELAY_MS

/* 时序周期数(在 init 里按 SystemCoreClock 算好, 单位=CPU 周期) */
static uint32_t s_half_cycles;  /* 半周期 5us */
static uint32_t s_gap_cycles;   /* 字节间隔 16us */
static uint32_t s_setup_cycles; /* CS 建立 2us */

/* 忙等到走过 cycles 个 CPU 周期。
 *
 * 不用 BSP_DWT_Delay: 它内部是 float 运算(__aeabi_fmul / __aeabi_f2uiz /
 * __aeabi_ui2f), F103 没 FPU, 全部软件模拟 —— 每次调用要多花 ~4us!
 * 5us 的半周期会被拖成 ~9us, 帧长从 864us 变 1471us, 时钟只剩 ~54kHz。 */
static inline void ps2_delay_cycles(uint32_t cycles)
{
    const uint32_t t0 = DWT->CYCCNT;
    while ((DWT->CYCCNT - t0) < cycles)
    {
    }
}

/* 4 个脚全是普通 GPIO(bit-bang 不占 SPI 外设)。
 * 直接写 BSRR/读 IDR: HAL_GPIO_WritePin/ReadPin 是函数调用, 一帧 72 个时钟里
 * 会被调用近 200 次, 开销同样会拉长半周期。 */
#define PS2_CLK_HIGH() (REMOTE_PS2_CLK_PORT->BSRR = (uint32_t)REMOTE_PS2_CLK_PIN)
#define PS2_CLK_LOW()  (REMOTE_PS2_CLK_PORT->BSRR = (uint32_t)REMOTE_PS2_CLK_PIN << 16)
#define PS2_DO_HIGH()  (REMOTE_PS2_DO_PORT->BSRR = (uint32_t)REMOTE_PS2_DO_PIN)
#define PS2_DO_LOW()   (REMOTE_PS2_DO_PORT->BSRR = (uint32_t)REMOTE_PS2_DO_PIN << 16)
#define PS2_CS_LOW()   (REMOTE_PS2_CS_PORT->BSRR = (uint32_t)REMOTE_PS2_CS_PIN << 16)
#define PS2_CS_HIGH()  (REMOTE_PS2_CS_PORT->BSRR = (uint32_t)REMOTE_PS2_CS_PIN)
#define PS2_DI_READ()  ((REMOTE_PS2_DI_PORT->IDR & (uint32_t)REMOTE_PS2_DI_PIN) != 0U)

#define PS2_CMD_START      0x01
#define PS2_CMD_POLL       0x42
#define PS2_ID_DIGITAL     0x41 /* 绿灯, 摇杆读数无效 */
#define PS2_ID_ANALOG      0x73 /* 红灯 */
#define PS2_ID_ANALOG_VIB  0x79
#define PS2_ID_HEADER      0x5A /* Data[2] 固定头, 用于判"数据有效" */

#define PS2_STICK_BIAS 128 /* 摇杆原始中位 */
#define PS2_STICK_MAX  660 /* 归一化量纲, 与 DT7/VT03 保持一致 */

#define PS2_BAD_FRAME_LIMIT 20 /* 连续无效帧数达到就退回重新握手(100Hz 下约 200ms) */

/* 上电配置序列(字节值以 WHEELTEC 例程 pstwo.c 为准) */
static const uint8_t kShortPoll[PS2_FRAME_LEN]   = {0x01, 0x42, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kEnterConfig[PS2_FRAME_LEN] = {0x01, 0x43, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t kSetAnalog[PS2_FRAME_LEN]   = {0x01, 0x44, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00}; /* 0x03=锁定模拟模式 */
static const uint8_t kExitConfig[PS2_FRAME_LEN]  = {0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A};

/* 收发缓冲 */
BUFFER_SECTION static uint8_t ps2_tx[PS2_FRAME_LEN];
BUFFER_SECTION static uint8_t ps2_rx[PS2_FRAME_LEN];

static bool ps2_initialized;
static bool ps2_analog_ok; /* 是否已成功进入模拟模式 */

/* 一个字节: LSB first。CLK 空闲高, 下降沿双方输出本位, 上升沿双方采样。 */
static uint8_t ps2_transfer_byte(uint8_t out)
{
    uint8_t in = 0;

    for (uint8_t bit = 0; bit < 8; bit++)
    {
        PS2_CLK_LOW(); /* 下降沿: 主机输出本位 */
        if (out & 0x01)
            PS2_DO_HIGH();
        else
            PS2_DO_LOW();
        out >>= 1;
        ps2_delay_cycles(s_half_cycles);

        PS2_CLK_HIGH(); /* 上升沿: 双方采样 */
        if (PS2_DI_READ()) in |= (uint8_t)(1u << bit);
        ps2_delay_cycles(s_half_cycles);
    }
    return in;
}

/* 一帧 9 字节: CS 整帧保持低, 字节之间留 ~16us(PS2 协议要求) */
static void ps2_transfer(void)
{
    PS2_CLK_HIGH(); /* 空闲高 */
    PS2_CS_LOW();
    ps2_delay_cycles(s_setup_cycles);

    for (uint8_t i = 0; i < PS2_FRAME_LEN; i++)
    {
        ps2_rx[i] = ps2_transfer_byte(ps2_tx[i]);
        ps2_delay_cycles(s_gap_cycles);
    }

    PS2_CLK_HIGH();
    PS2_CS_HIGH();
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
        BSP_DWT_Delay(1e-3f); /* 给手柄状态机留时间 */
        ps2_send(kSetAnalog);
        BSP_DWT_Delay(1e-3f);
        ps2_send(kExitConfig);
        BSP_DWT_Delay(1e-3f);
        ps2_send(kShortPoll); /* 校验帧 */

        if (ps2_rx[1] == PS2_ID_ANALOG || ps2_rx[1] == PS2_ID_ANALOG_VIB)
        {
            return true;
        }
        LOG_W("analog fail %u: rx %02X %02X %02X %02X %02X %02X %02X %02X %02X", (unsigned)(attempt + 1), ps2_rx[0], ps2_rx[1],
              ps2_rx[2], ps2_rx[3], ps2_rx[4], ps2_rx[5], ps2_rx[6], ps2_rx[7], ps2_rx[8]);
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
    /* 4 个脚由**板级 CubeMX** 配置, 本模块只管翻转电平(同 LoRA 的 M0/M1/AUX)。
     * 要求(必须与 REMOTE_PS2_* 宏一致, 否则自己用了哪个脚都看不出来):
     *   CLK / DO / CS → GPIO_Output 推挽, 无上下拉, Speed High, 初始电平 High
     *   DI            → GPIO_Input  上拉(接收器不应答时 DI 悬空, 不拉会被相邻数据线串扰)
     * 这里不再 HAL_GPIO_Init 覆盖: 板子在 gpio.c 里看得见, 模块不要偷偷改別人配好的脚。 */

    /* 把 us 换算成 CPU 周期(整数, 避免运行时软浮点开销) */
    const uint32_t mhz = SystemCoreClock / 1000000U;
    s_half_cycles      = REMOTE_PS2_HALF_BIT_US * mhz;
    s_gap_cycles       = REMOTE_PS2_BYTE_GAP_US * mhz;
    s_setup_cycles     = 2U * mhz;

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

    /* 握手(进模拟模式)故意不在这里做:
     * 本函数跑在 tx_application_define 里, 调度器还没起来 ——
     *   - tx_thread_sleep() 会直接返回 TX_CALLER_ERROR(空操作)
     *   - 用 DWT 忙等又会把 IWDG 的喂狗窗口撑爆(offline 任务还没开始喂) → 开机复位循环
     * 所以推迟到解码任务里做(那里 tx_thread_sleep 有效, 且任务是周期性的能重试) */
    ps2_initialized = true;
    /* 打出实际编译进来的引脚: 板级覆盖 / module_remote.h 默认值 / 烧错 elf, 一眼能分清 */
    LOG_I("ps2 pins: CLK=%p:0x%04X DO=%p:0x%04X", (void *)REMOTE_PS2_CLK_PORT, (unsigned)REMOTE_PS2_CLK_PIN,
          (void *)REMOTE_PS2_DO_PORT, (unsigned)REMOTE_PS2_DO_PIN);
    LOG_I("ps2 pins: DI=%p:0x%04X CS=%p:0x%04X", (void *)REMOTE_PS2_DI_PORT, (unsigned)REMOTE_PS2_DI_PIN,
          (void *)REMOTE_PS2_CS_PORT, (unsigned)REMOTE_PS2_CS_PIN);
    LOG_I("ps2 init done, handshake deferred to task");
    return 0;
}

void remote_ps2_decode(Remote_Data_t *data, Offline_Device *offline)
{
    if (!ps2_initialized || data == NULL) return;

    /* 还没握上手: 先等接收器起来, 再重试(晚插/晚配对也能自恢复) */
    if (!ps2_analog_ok)
    {
        tx_thread_sleep(PS2_HANDSHAKE_RETRY_MS);
        if (ps2_enter_analog())
        {
            ps2_analog_ok = true;
            LOG_I("analog mode OK");
        }
        else
        {
            LOG_W("analog mode failed, will retry (check wiring / 3.3V / CS pin)");
        }
        return;
    }

    /* 组 poll 帧; [3]/[4] 是振动强度, 第一版不做(固定 0 = 不震) */
    ps2_tx[0] = PS2_CMD_START;
    ps2_tx[1] = PS2_CMD_POLL;
    for (uint8_t i = 2; i < PS2_FRAME_LEN; i++)
    {
        ps2_tx[i] = 0x00;
    }

    ps2_transfer();

    const uint8_t id = ps2_rx[1];
    /* 非法帧(手柄关机/待机/未配对, 回包多为 0xFF / 0x00) → 不喂心跳, 交给 OFFLINE 判离线。
     * 连续 PS2_BAD_FRAME_LIMIT 帧都无效就退回重新握手 —— 否则一次凑巧的 0x73 会卡死在
     * "已进模拟模式"的状态里, 拔插/掉电后再也恢复不了。 */
    static uint8_t s_bad_frames;
    if ((id != PS2_ID_ANALOG && id != PS2_ID_ANALOG_VIB) || ps2_rx[2] != PS2_ID_HEADER)
    {
        if (++s_bad_frames >= PS2_BAD_FRAME_LIMIT)
        {
            s_bad_frames  = 0;
            ps2_analog_ok = false;
        }
        return;
    }
    s_bad_frames = 0;

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
