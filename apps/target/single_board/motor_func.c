/*
 * @file    motor_func.c
 * @brief   target 兵种(靶车) — 电机 (DJI M3508 / 达妙 DM6220)
 *
 * 只做"按键事件 → 电机动作"这一件事。
 * 参数全部在 target_def.h, 电机统一由 M3508/MOTOR 模块的周期任务驱动出力。
 */

#include "motor_func.h"

#include "target_def.h"

#include "module_motor.h"
#include "module_offline.h"

#include "bsp_def.h"

#define LOG_TAG "target_motor"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

/* ========== DJI M3508 ========== */
static DJI_Motor_t *m3508;
static bool         m3508_on;

/* ========== 达妙 DM6220 (暂不接入) ========== */
static DM_Motor_t *dm6220;
static bool        dm6220_on;

/* ========== 初始化 ========== */

static void m3508_init(void)
{
    Motor_Init_Config_s cfg = {
        .offline_init_config =
            {
                .name       = "m3508",
                .timeout_ms = TARGET_3508_OFFLINE_TIMEOUT_MS,
                .beep_times = 0,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                /* 板级 CubeMX 只初始化了 CAN2 (PB12/PB13), 用 HANDLE1 会静默发不出去
                 * tx_id 填 C620 电调 ID (1~8), 驱动自动推导反馈 ID = 0x200 + tx_id */
                .hcan  = BSP_CAN_HANDLE2,
                .tx_id = TARGET_3508_CAN_ID,
            },
        /* 速度环: LQR 单状态 (纯比例, K 需实测调) */
        .controller_init_config = {.lqr_init = {.K = {TARGET_3508_LQR_K}, .state_dim = 1}},
        .setting_init_config =
            {
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .feedback_reverse_flag = 0,
                .algorithm_type        = CONTROL_LQR,
            },
        .motor_init_info =
            {
                .motor_type      = M3508,
                .gear_ratio      = TARGET_3508_GEAR_RATIO,
                .max_torque      = TARGET_3508_MAX_TORQUE,
                .torque_constant = TARGET_3508_TORQUE_CONSTANT,
            },
    };

    m3508 = Motor_DJI_Init(&cfg);
    if (m3508 == NULL)
    {
        LOG_E("m3508 init failed");
        return;
    }

    Motor_Stop((Motor_Base *)m3508); /* 上电默认停转, 等按键 */
}

static void dm6220_init(void)
{
    Motor_Init_Config_s cfg = {
        .offline_init_config =
            {
                .name       = "dm6220",
                .timeout_ms = TARGET_DM_OFFLINE_TIMEOUT_MS,
                .beep_times = 0,
                .enable     = 1,
            },
        .transport = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE2,
                .tx_id = TARGET_DM_CAN_TX_ID,
                .rx_id = TARGET_DM_CAN_RX_ID,
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

void motor_func_init(void)
{
    m3508_init();

    /* DM6220 暂不接入: SPD 模式下它的控制帧 ID = 0x01 + 0x200 = 0x201,
     * 与 M3508(C620 ID=1) 的反馈 ID 0x201 完全相同, 两条都在 CAN2 上会互相干扰。
     * 单独调 DM6220 时把下面这行打开即可 (同时建议把 M3508 或 DM 挪到 CAN1) */
#if 0
    dm6220_init();
#endif

    LOG_I("motor init done");
}

/* ========== 周期控制 ========== */

static void m3508_update(bool toggle)
{
    if (m3508 == NULL || !toggle) return;

    m3508_on = !m3508_on;
    if (m3508_on)
    {
        Motor_SetRef((Motor_Base *)m3508, TARGET_3508_SPEED_RAD_S);
        Motor_Start((Motor_Base *)m3508);
    }
    else
    {
        Motor_Stop((Motor_Base *)m3508);
    }
    LOG_I("m3508 %s | motor %s", m3508_on ? "ON" : "OFF",
          (Module_Offline_get_device_status(m3508->base.offline_dev) == STATE_ONLINE) ? "ONLINE" : "OFFLINE");
}

static void dm6220_update(bool toggle, bool arm)
{
    if (dm6220 == NULL) return;

    if (toggle)
    {
        dm6220_on = !dm6220_on;
        if (dm6220_on)
        {
            Motor_SetRef((Motor_Base *)dm6220, TARGET_DM_SPEED_RAD_S);
            Motor_Start((Motor_Base *)dm6220);
        }
        else
        {
            Motor_Stop((Motor_Base *)dm6220);
        }
        LOG_I("dm6220 %s | motor %s", dm6220_on ? "ON" : "OFF",
              (Module_Offline_get_device_status(dm6220->base.offline_dev) == STATE_ONLINE) ? "ONLINE" : "OFFLINE");
    }

    /* 电机晚于主控上电 / 初始两帧丢了 → 手动重发 清错+启动 */
    if (arm)
    {
        Motor_DM_Cmd(dm6220, DM_CMD_CLEAR_ERROR);
        Motor_DM_Cmd(dm6220, DM_CMD_MOTOR_START);
        LOG_I("dm6220 re-arm (clear error + start)");
    }
}

void motor_func(const Remote_Btn_Event_t *evt)
{
    if (evt == NULL) return;

    dm6220_update(evt->dm_toggle, evt->dm_arm);
    m3508_update(evt->m3508_toggle);

    /* 电机开着时每 TARGET_LOG_PERIOD_MS 打一次状态 (调试用, 查完可删):
     *   torque=0          → 电机掉线或被禁用 (Motor_ControlAll 会把它清零)
     *   torque 非 0 但 spd=0 → 控制帧没到电调 (C620 ID / 接线 / 终端电阻) */
    static uint16_t log_div;
    if (m3508_on && m3508 != NULL)
    {
        if (++log_div >= TARGET_LOG_PERIOD_MS / TARGET_CONTROL_PERIOD_MS)
        {
            log_div = 0;
            LOG_I("m3508 %s | torque=%dmNm spd=%dmrad/s",
                  (Module_Offline_get_device_status(m3508->base.offline_dev) == STATE_ONLINE) ? "ONLINE" : "OFFLINE",
                  (int)(m3508->base.controller.output_torque * 1000.0f), (int)(m3508->base.measure.speed_rad * 1000.0f));
        }
    }
}
