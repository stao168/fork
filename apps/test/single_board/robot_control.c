/*
 * @file        robot_control.c
 * @brief       test 兵种 — NRF24L01 模块测试用例
 *
 * ────────────────────────────────────────────────────────────────
 *  前置: 先在板级 CubeMX 里配好 SPI2 与 CE/CSN/IRQ(EXTI), 见 module_nrf24l01.h
 *
 *  测试原理:
 *    能力注册式框架要求收发两端注册顺序完全一致:
 *    A 板的第 N 个变量 ↔ B 板的第 N 个变量。
 *    两块板注册相同变量, 一端只发、一端只收。
 *
 *  验证方法:
 *    - 两块板各接一个 nRF24L01, 注册项完全一致
 *    - 收发角色由 robot.cmake 里的 NRF24L01_TX_ENABLE / NRF24L01_RX_ENABLE 选定
 *      (二者互斥, 一块只开 TX、一块只开 RX, 各自编译烧录)
 *    - 串口日志观察接收端变量是否跟随发送端变化
 *    - 拔掉一块板电源, 另一块应在超时后由 ONLINE 变 OFFLINE
 * ────────────────────────────────────────────────────────────────
 */
#include "robot_control.h"
#include "module_nrf24l01.h"
#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "test_robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ========== 测试变量(两块板注册相同变量, 顺序一致) ========== */
static float    s_val_float  = 0.0f; /* 浮点计数器 */
static int16_t  s_val_int16  = 0;    /* 16位计数器 */
static uint8_t  s_val_uint8  = 0;    /* 8位计数器  */
static uint32_t s_val_uint32 = 0;    /* 32位计数器 */
static uint32_t s_loop_count = 0;    /* 循环计数   */

/* ========== 主循环任务 ========== */
static TX_THREAD                  g_test_loop_thread;
APPS_STACK_SECTION static uint8_t g_test_loop_stack[1024];

static void test_loop_task_entry(ULONG arg)
{
    (void)arg;

    /* 注册测试变量(收发两端顺序必须完全一致) */
    Module_NRF24L01_Register("val_float",  &s_val_float,  NRF24L01_TYPE_FLOAT);
    Module_NRF24L01_Register("val_int16",  &s_val_int16,  NRF24L01_TYPE_INT16);
    Module_NRF24L01_Register("val_uint8",  &s_val_uint8,  NRF24L01_TYPE_UINT8);
    Module_NRF24L01_Register("val_uint32", &s_val_uint32, NRF24L01_TYPE_UINT32);
    LOG_I("NRF24L01 test: 4 vars registered (11 bytes)");

    while (1)
    {
        s_loop_count++;
#if NRF24L01_TX_ENABLE
        /* 本板是发送角色时自增本地变量(会被自动发送到对端对应变量) */
        s_val_float  += 0.1f;
        s_val_int16  += 1;
        s_val_uint8  += 1;
        s_val_uint32 += 10;
#endif
        /* 每 500ms 打印一次本地值 + 在线状态 */
        if ((s_loop_count % 50) == 0)
        {
            uint8_t status = Module_NRF24L01_GetStatus();
            LOG_I("val: f=%.1f i16=%d u8=%u u32=%lu | %s", s_val_float, s_val_int16, s_val_uint8,
                  (unsigned long)s_val_uint32, status == 0 ? "ONLINE" : "OFFLINE");
        }

        tx_thread_sleep(10); /* 100Hz 循环 */
    }
}

/* ========== 入口 ========== */

void robot_control_init(void)
{
    /* 主循环任务 (模块注册需在 MODULE_Init 之后, 此处即可) */
    UINT status = tx_thread_create(&g_test_loop_thread, "test_loop", test_loop_task_entry, 0,
                                   g_test_loop_stack, sizeof(g_test_loop_stack),
                                   8, 8, TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("test loop thread create failed (0x%02x)", status);
        return;
    }

    LOG_I("test robot init done");
}