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

/* 上一次按键状态 (取上升沿用) */
static bool last_m3508_toggle;
static bool last_dm_power;
static bool last_dm_clear;
static bool last_dm_fixed;
static bool last_dm_throttle;
static bool last_dm_dir;

void remote_func_init(void)
{
    if (Module_Remote_get_data() == NULL) LOG_E("remote not ready");
}

void remote_func(Remote_Input_t *in)
{
    if (in == NULL) return;

    in->m3508_toggle = false;
    in->dm_power     = false;
    in->dm_clear     = false;
    in->dm_fixed     = false;
    in->dm_throttle  = false;
    in->dm_dir       = false;
    in->key_fwd      = false;
    in->key_back     = false;
    in->key_acc      = false;
    in->key_dec      = false;

    Remote_Data_t *data = Module_Remote_get_data();
    if (data == NULL) return;

    /* 手柄掉线时 data 停在最后一帧: 方向键必须清零, 否则电机会一直按最后的方向跑 */
    if ((Module_Remote_get_offline_status() & 0x01) == 0) return;

    const uint16_t btn = data->ps2.buttons;

    bool m3508_toggle = (btn & TARGET_3508_ON_BTN) != 0;
    bool dm_power     = (btn & TARGET_DM_POWER_BTN) != 0;
    bool dm_clear     = (btn & TARGET_DM_CLEAR_BTN) != 0;
    bool dm_fixed     = (btn & TARGET_DM_FIXED_BTN) != 0;
    bool dm_throttle  = (btn & TARGET_DM_THROTTLE_BTN) != 0;
    bool dm_dir       = (btn & TARGET_DM_DIR_BTN) != 0;

    /* 上升沿: 按一下只触发一次 */
    in->m3508_toggle = m3508_toggle && !last_m3508_toggle;
    in->dm_power     = dm_power && !last_dm_power;
    in->dm_clear     = dm_clear && !last_dm_clear;
    in->dm_fixed     = dm_fixed && !last_dm_fixed;
    in->dm_throttle  = dm_throttle && !last_dm_throttle;
    in->dm_dir       = dm_dir && !last_dm_dir;

    last_m3508_toggle = m3508_toggle;
    last_dm_power     = dm_power;
    last_dm_clear     = dm_clear;
    last_dm_fixed     = dm_fixed;
    last_dm_throttle  = dm_throttle;
    last_dm_dir       = dm_dir;

    /* 方向键是电平: 按住就有效 */
    in->key_fwd  = (btn & TARGET_3508_FWD_BTN) != 0;
    in->key_back = (btn & TARGET_3508_BACK_BTN) != 0;
    in->key_acc  = (btn & TARGET_DM_ACC_BTN) != 0;
    in->key_dec  = (btn & TARGET_DM_DEC_BTN) != 0;
}
