/*
 * @file    remote_func.c
 * @brief   target 兵种(靶车) — 遥控器(PS2 手柄)输入解析
 *
 * 只做"手柄原始数据 → 按键事件"这一件事, 不碰电机。
 * 按键位定义见 target_def.h, 按键表见 module_remote.h 的 ps2_custom_t。
 */

#include "remote_func.h"

#include "target_def.h"

#include "module_remote.h"

#include "bsp_def.h"

#define LOG_TAG "target_remote"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static ps2_custom_t *ps2;

/* 上一次按键状态 (取上升沿用) */
static bool last_dm_toggle;
static bool last_dm_arm;
static bool last_m3508_toggle;

void remote_func_init(void)
{
    ps2 = Module_Remote_get_ps2_custom();
    if (ps2 == NULL) LOG_E("ps2 remote not ready");
}

void remote_func(Remote_Btn_Event_t *evt)
{
    if (evt == NULL) return;

    evt->dm_toggle    = false;
    evt->dm_arm       = false;
    evt->m3508_toggle = false;

    ps2 = Module_Remote_get_ps2_custom();
    if (ps2 == NULL) return;

    bool dm_toggle    = (ps2->buttons & TARGET_DM_TOGGLE_BTN) != 0;
    bool dm_arm       = (ps2->buttons & TARGET_DM_ARM_BTN) != 0;
    bool m3508_toggle = (ps2->buttons & TARGET_3508_TOGGLE_BTN) != 0;

    /* 上升沿: 按一下只触发一次 */
    evt->dm_toggle    = dm_toggle && !last_dm_toggle;
    evt->dm_arm       = dm_arm && !last_dm_arm;
    evt->m3508_toggle = m3508_toggle && !last_m3508_toggle;

    last_dm_toggle    = dm_toggle;
    last_dm_arm       = dm_arm;
    last_m3508_toggle = m3508_toggle;
}
