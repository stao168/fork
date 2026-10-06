/*
 * @file    robot_control.c
 * @brief   target 兵种(靶车) — 单板主控
 *
 * 分工: 遥控器输入解析在 remote_func.c, 电机控制在 motor_func.c,
 *       本文件只负责初始化和主控制任务的调度。
 */
#include "robot_control.h"

#include "target_def.h"

#include "remote_func.h"
#include "motor_func.h"

#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "target_robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ========== 主控制任务 ========== */
static TX_THREAD                  g_ctrl_thread;
APPS_STACK_SECTION static uint8_t g_ctrl_stack[1024];

static Remote_Input_t remote_in; /* 本周期遥控输入 (按键事件 + 方向键电平) */

static void target_ctrl_task_entry(ULONG arg)
{
    (void)arg;

    while (1)
    {
        remote_func(&remote_in); /* 读手柄 → 按键事件 + 方向键 */
        motor_func(&remote_in);  /* 按键/方向键 → 电机动作 */

        /* TODO: 云台控制 */
        /* TODO: 底盘控制 */

        tx_thread_sleep(TARGET_CONTROL_PERIOD_MS);
    }
}

/* ========== 入口 ========== */
void robot_control_init(void)
{
    remote_func_init();
    motor_func_init();

    if (tx_thread_create(&g_ctrl_thread, "target_ctrl", target_ctrl_task_entry, 0, g_ctrl_stack, sizeof(g_ctrl_stack), 8, 8, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("target control thread create failed");
        return;
    }

    LOG_I("target robot init done");
}
