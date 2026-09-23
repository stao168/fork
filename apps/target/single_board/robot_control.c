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
#include "module_motor.h"

#include "tx_api.h"
#include "bsp_def.h"

#define LOG_TAG "target_robot"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ========== 遥控器 ========== */
static ps2_custom_t *ps2;

/* ========== 达妙 DM6220 ========== */
static DM_Motor_t *dm6220;
static bool        dm6220_on; /* 电机开关状态 */
static bool        btn_last;  /* 上一次按键状态 (取上升沿用) */

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
                LOG_I("PS2 mode=0x%02X ch=%d,%d,%d,%d btn=0x%04X | %s", ps2->mode, Module_Remote_get_channel(1), Module_Remote_get_channel(2),
                      Module_Remote_get_channel(3), Module_Remote_get_channel(4), ps2->buttons,
                      (Module_Remote_get_offline_status() & 0x01) ? "ONLINE" : "OFFLINE");
            }
        }

        /* PS2 按键开关电机 (上升沿翻转, 转速固定) */
        if (ps2 != NULL && dm6220 != NULL)
        {
            bool btn = (ps2->buttons & TARGET_MOTOR_TOGGLE_BTN) != 0;
            if (btn && !btn_last)
            {
                dm6220_on = !dm6220_on;
                if (dm6220_on)
                {
                    Motor_SetRef((Motor_Base *)dm6220, TARGET_MOTOR_SPEED_RAD_S);
                    Motor_Start((Motor_Base *)dm6220);
                }
                else
                {
                    Motor_Stop((Motor_Base *)dm6220);
                }
                LOG_I("dm6220 %s | motor %s", dm6220_on ? "ON" : "OFF",
                      (Module_Offline_get_device_status(dm6220->base.offline_dev) == STATE_ONLINE) ? "ONLINE" : "OFFLINE");
            }
            btn_last = btn;
        }

        /* TODO: 云台控制 */
        /* TODO: 底盘控制 */

        tx_thread_sleep(TARGET_CONTROL_PERIOD_MS);
    }
}

/* ========== 入口 ========== */
static void target_motor_init(void)
{
    Motor_Init_Config_s cfg = {
        .offline_init_config =
            {
                .name       = "dm6220",
                .timeout_ms = TARGET_MOTOR_OFFLINE_TIMEOUT_MS,
                .beep_times = 0,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = TARGET_MOTOR_CAN_TX_ID,
                .rx_id = TARGET_MOTOR_CAN_RX_ID,
            },
        /* 定速旋转交给电机内部速度环, 这里 OPEN_LOOP 只把 ref 透传给 speed_ctrl */
        .setting_init_config = {.algorithm_type = CONTROL_PID, .loop_type = OPEN_LOOP},
        .motor_init_info     = {.motor_type = DM6220, .gear_ratio = 1, .max_torque = 10.0f},
    };

    dm6220 = Motor_DM_Init(&cfg, DM_SPD_MODE);
    if (dm6220 == NULL)
    {
        LOG_E("dm6220 init failed");
        return;
    }

    Motor_Stop((Motor_Base *)dm6220); /* 上电默认停转, 等按键 */
}

void robot_control_init(void)
{
    target_motor_init();

    if (tx_thread_create(&g_ctrl_thread, "target_ctrl", target_ctrl_task_entry, 0, g_ctrl_stack, sizeof(g_ctrl_stack), 8, 8, TX_NO_TIME_SLICE,
                         TX_AUTO_START) != TX_SUCCESS)
    {
        LOG_E("target control thread create failed");
        return;
    }

    LOG_I("target robot init done");
}
