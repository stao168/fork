/*
 * @file    motor_func.c
 * @brief   target 兵种(靶车) — 电机 (DJI M3508 / 达妙 DM6220)
 *
 * 方向键 → 转速, 其余按键 → 启停/换向/模式。
 *   M3508: L1 使能开关, ↑ 按住前进(输出轴 +1 转/秒), ↓ 按住后退, 松开停
 *   达妙  : R1 总开关, R2 重发清错, △ 固定转速(0.4 转/秒), □ 油门模式, × 正反转
 *          油门模式下 ← 按住转速加 / → 按住转速减, 松开保持; 最大 ±1 转/秒
 *     两个模式可同时关、不能同时开; 总开关按下清零转速并复位模式。
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
static float        m3508_ref; /* 本周期目标转速 (rad/s, 转子侧), 只用于打印 */

/* ========== 达妙 DM6220 ========== */
static DM_Motor_t *dm6220;
static bool        dm6220_power;       /* R1 总开关 */
static bool        dm6220_fixed;       /* △ 固定转速模式 */
static bool        dm6220_throttle_on; /* □ 油门模式 */
static bool        dm6220_reverse;     /* × 正反转 */
static float       dm6220_throttle;    /* 油门值 (rad/s): 油门模式下由 ←/→ 增减, 也是模式切换时要清零的状态 */
static float       dm6220_ref;         /* 本周期下发的目标转速 (含正反转符号, rad/s) */

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
                /* CAN1 (105_rc: PA11=RX / PA12=TX, 默认引脚组, 不做重映射);
                 * tx_id = C620 电调 ID, 必须和电调拨码一致; 反馈 ID 自动 = 0x200 + tx_id */
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 8,
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
                /* MIT 模式: 控制帧 ID = tx_id + DM_MIT_MODE = 0x001, 反馈帧 ID = Master ID = 0xF1
                 * (电机内部就是 MIT 模式, 两边必须一致, 否则只发光不出力)。
                 * 0x001 和 M3508 的 0x1FF/0x200 控制帧、0x208 反馈帧都不冲突。 */
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = TARGET_DM_CAN_TX_ID,
                .rx_id = TARGET_DM_CAN_RX_ID,
            },
        /* MIT 模式下驱动只把力矩发出去(dm_apply → mit_ctrl(0,0,0,0,torque)),
         * 速度环由我们自己闭。这类电机内部自带闭环, 外面只要一个很小的 P “轻推”
         * 就行 —— Kp 大了(或加 Ki)会和电机内部环打架 ⇒ 左右振 + 转速失控。 */
        .controller_init_config =
            {
                .speed_PID =
                    {
                        /* Ki/Kd 故意不用: 若要加 Ki, 必须同时补 .Improve = 0x01 与
                         * .IntegralLimit, 否则积分饱和又会变成“越转越快” */
                        .Kp     = TARGET_DM_SPEED_KP,
                        .Ki     = TARGET_DM_SPEED_KI,
                        .Kd     = TARGET_DM_SPEED_KD,
                        .MaxOut = TARGET_DM_MAX_TORQUE,
                    },
            },
        .setting_init_config =
            {
                .speed_feedback_source = 0,
                .loop_type             = SPEED_LOOP,
                .algorithm_type        = CONTROL_PID,
            },
        .motor_init_info = {.motor_type = DM6220, .gear_ratio = 1, .max_torque = TARGET_DM_MAX_TORQUE},
    };

    dm6220 = Motor_DM_Init(&cfg, DM_MIT_MODE);
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
    dm6220_init();

    LOG_I("motor init done");
}

/* ========== 周期控制 ========== */

static void m3508_update(bool toggle, bool fwd, bool back)
{
    if (m3508 == NULL) return;

    if (toggle)
    {
        m3508_on = !m3508_on;
        if (m3508_on)
        {
            Motor_Start((Motor_Base *)m3508);
        }
        else
        {
            Motor_Stop((Motor_Base *)m3508);
        }
        LOG_I("m3508 %s | motor %s", m3508_on ? "ON" : "OFF",
              (Module_Offline_get_device_status(m3508->base.offline_dev) == STATE_ONLINE) ? "ONLINE" : "OFFLINE");
    }

    /* ↑ 前进 / ↓ 后退 / 都不按或两个都按 = 停; 关掉时 ref 也归零, 免得下次打开直接冲出去 */
    float dir = (fwd && !back) ? 1.0f : ((back && !fwd) ? -1.0f : 0.0f);
    m3508_ref = m3508_on ? dir * TARGET_3508_SPEED_RAD_S : 0.0f;
    Motor_SetRef((Motor_Base *)m3508, m3508_ref);
}

static void dm6220_update(const Remote_Input_t *in)
{
    if (dm6220 == NULL) return;

    float dt = TARGET_CONTROL_PERIOD_MS * 0.001f;

    /* 总开关: 按下清零转速, 顺便把两个模式也复位 —— 再打开时保持 0, 等模式键 */
    if (in->dm_power)
    {
        dm6220_power       = !dm6220_power;
        dm6220_fixed       = false;
        dm6220_throttle_on = false;
        dm6220_throttle    = 0.0f;

        if (dm6220_power)
        {
            /* 打开时顺手重发一次使能帧: 电机比主控晚上电时不用再专门按一下 */
            Motor_DM_Cmd(dm6220, DM_CMD_CLEAR_ERROR);
            Motor_DM_Cmd(dm6220, DM_CMD_MOTOR_START);
            Motor_Start((Motor_Base *)dm6220);
        }
        else
        {
            Motor_Stop((Motor_Base *)dm6220);
        }
        LOG_I("dm6220 power %s | motor %s", dm6220_power ? "ON" : "OFF",
              (Module_Offline_get_device_status(dm6220->base.offline_dev) == STATE_ONLINE) ? "ONLINE" : "OFFLINE");
    }

    /* 固定转速模式 (与油门模式互斥; 再按一下关掉) */
    if (in->dm_fixed)
    {
        dm6220_fixed = !dm6220_fixed;
        if (dm6220_fixed) dm6220_throttle_on = false;
        dm6220_throttle = dm6220_fixed ? TARGET_DM_FIXED_SPEED_RAD_S : 0.0f; /* 切进来直接变 0.4, 关掉则清零 */
        LOG_I("dm6220 fixed mode %s", dm6220_fixed ? "ON (0.4 rev/s)" : "OFF");
    }

    /* 油门模式 (与固定模式互斥; 再按一下关掉): 转速由 ←/→ 增减 */
    if (in->dm_throttle)
    {
        dm6220_throttle_on = !dm6220_throttle_on;
        if (dm6220_throttle_on) dm6220_fixed = false;
        dm6220_throttle = 0.0f; /* 进模式 / 关模式都从 0 重新开始 */
        LOG_I("dm6220 throttle mode %s", dm6220_throttle_on ? "ON" : "OFF");
    }

    if (in->dm_dir)
    {
        dm6220_reverse = !dm6220_reverse;
        LOG_I("dm6220 dir: %s", dm6220_reverse ? "REV" : "FWD");
    }

    /* 电机晚于主控上电 / 初始两帧丢了 → 手动重发 清错+启动 */
    if (in->dm_clear)
    {
        Motor_DM_Cmd(dm6220, DM_CMD_CLEAR_ERROR);
        Motor_DM_Cmd(dm6220, DM_CMD_MOTOR_START);
        LOG_I("dm6220 re-arm (clear error + start)");
    }

    /* 转速指令: 油门模式 = ←按住加/→按住减(松开保持), 固定模式 = 恒速, 都没开 = 0 */
    if (dm6220_throttle_on)
    {
        float dir = (in->key_acc && !in->key_dec) ? 1.0f : ((in->key_dec && !in->key_acc) ? -1.0f : 0.0f);
        dm6220_throttle += dir * TARGET_DM_THROTTLE_RAMP_RAD_S2 * dt;
        if (dm6220_throttle > TARGET_DM_THROTTLE_MAX_RAD_S) dm6220_throttle = TARGET_DM_THROTTLE_MAX_RAD_S;
        if (dm6220_throttle < -TARGET_DM_THROTTLE_MAX_RAD_S) dm6220_throttle = -TARGET_DM_THROTTLE_MAX_RAD_S;
    }
    else if (dm6220_fixed)
    {
        dm6220_throttle = TARGET_DM_FIXED_SPEED_RAD_S;
    }
    else
    {
        dm6220_throttle = 0.0f;
    }

    /* 总开关关着 → 一律 0 (此时 enableflag=0, 驱动本来也只发零力矩帧) */
    dm6220_ref = dm6220_power ? (dm6220_reverse ? -dm6220_throttle : dm6220_throttle) : 0.0f;
    Motor_SetRef((Motor_Base *)dm6220, dm6220_ref);
}

void motor_func(const Remote_Input_t *in)
{
    if (in == NULL) return;

    m3508_update(in->m3508_toggle, in->key_fwd, in->key_back);
    dm6220_update(in);

    /* 有电机开着时每 TARGET_LOG_PERIOD_MS 打一次方向键与目标转速 (调试用, 查完可删):
     * key 四位依次 = ↑ ↓ ← → (1=按住), 看它有反应就说明按键读到了 */
    static uint16_t log_div;
    if ((m3508_on || dm6220_power) && ++log_div >= TARGET_LOG_PERIOD_MS / TARGET_CONTROL_PERIOD_MS)
    {
        log_div = 0;
        LOG_I("key %d%d%d%d | m3508=%d dm=%d mrad/s | %s", in->key_fwd, in->key_back, in->key_acc, in->key_dec, (int)(m3508_ref * 1000.0f),
              (int)(dm6220_ref * 1000.0f), dm6220_throttle_on ? "THROTTLE" : (dm6220_fixed ? "FIXED" : "IDLE"));
    }
}
