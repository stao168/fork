/*
 * @file    robot_control.c
 * @brief   target 兵种(靶车) — 单板主控
 *
 * 阶段一: 只接 REMOTE(PS2 手柄), 打印通道读数确认遥控链路
 * 阶段二: 加 MOTOR(DJI + 达妙) 和实际控制逻辑
 */
#include "robot_control.h"

#include "target_def.h"

#include "module_remote.h"

#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "target_robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ========== 遥控器 ========== */
static ps2_custom_t *ps2;

/* ========== 主控制任务 ========== */
static TX_THREAD                  g_ctrl_thread;
APPS_STACK_SECTION static uint8_t g_ctrl_stack[1024];

static void target_ctrl_task_entry(ULONG arg)
{
    (void)arg;

    uint16_t log_div = 0;

    while (1)
    {
        /* PS2 通道读数 (验证遥控链路用; 调好之后换成实际控制逻辑) */
        ps2 = Module_Remote_get_ps2_custom();

        if (++log_div >= TARGET_LOG_PERIOD_MS / TARGET_CONTROL_PERIOD_MS)
        {
            log_div = 0;
            if (ps2 != NULL)
            {
                LOG_I("PS2 mode=0x%02X ch=%d,%d,%d,%d btn=0x%04X | %s", ps2->mode, Module_Remote_get_channel(1),
                      Module_Remote_get_channel(2), Module_Remote_get_channel(3), Module_Remote_get_channel(4), ps2->buttons,
                      (Module_Remote_get_offline_status() & 0x01) ? "ONLINE" : "OFFLINE");
            }
        }

        /* TODO: 云台控制 */
        /* TODO: 底盘控制 */

        tx_thread_sleep(TARGET_CONTROL_PERIOD_MS);
    }
}

/* ========== 入口 ========== */
void robot_control_init(void)
{
    if (tx_thread_create(&g_ctrl_thread, "target_ctrl", target_ctrl_task_entry, 0, g_ctrl_stack, sizeof(g_ctrl_stack), 8, 8,
                         TX_NO_TIME_SLICE, TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("target control thread create failed");
        return;
    }

    LOG_I("target robot init done");
}
