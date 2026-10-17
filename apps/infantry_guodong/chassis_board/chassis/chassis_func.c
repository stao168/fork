/*
 * @file    chassis_func.c (底盘板)
 * @brief   四轮全向轮底盘: 摇杆意图 -> 模式决定自转 -> 坐标变换 -> 逆运动学
 *
 * 旧工程事实(CarBody/Mecanum.c + Mecanum.h):
 *   电机 ID: 1=LF(左前) 2=RF(右前) 3=RR(右后) 4=LR(左后), 全在 CAN1
 *   逆解   : 输出【转子 RPM】,旧工程再用 PID 追 RotorSpeed
 *
 * !! 关于逆解公式(重要):
 *   文件名虽然叫 Mecanum.c, 但实车是【全向轮】(轮子 ±45°, 40cm×40cm 正方轮距)。
 *   45° 全向轮有两种布局, 旧工程和框架的 Chassis_Omni_Calc 不是同一种:
 *     切向布局(旧工程, 也是本文件用的) : 自转项 wz*(a+b), 四轮【同号】 -> 能自转
 *     径向布局(框架 Chassis_Omni_Calc)  : 自转项 wz*(a-b), 四轮符号交替
 *                                        -> 正方轮距下 a-b≡0, 四轮 ω 项全抵消, 自转恒为 0
 *   旧工程公式四项我逐项验算过, 就是切向 X 型的标准逆解(力平衡能对上), 而且旧车
 *   确实有小陀螺功能 —— 所以本车是切向布局, 用本文件的 chassis_xdrive_calc。
 *   ⚠ 架空实测时若发现"推前进变成原地自转", 那就是径向布局, 换回 Chassis_Omni_Calc。
 *
 * !! 轮序: 本文件数组顺序 = [LF, RF, RR, LR], 与旧工程 Chassis_* 命名一一对应
 *   (所以 chassis_init 里就是 tx_id 1/2/3/4 按序)。因为已经不用框架的麦轮/全向轮
 *   逆解了, 这里【故意不采用】框架文档里 [LF, LB, RB, RF] 那套顺序, 以免两边混淆。
 */
#include "chassis_func.h"
#include "module_offline.h"
#include "motor_dji.h"
#include "user_lib.h"
#include <stdint.h>

#define LOG_TAG "app_chassis"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static DJI_Motor_t *chassis_motors[4]; /* [LF, RF, RR, LR] */

void chassis_init(void)
{
    Motor_Init_Config_s cfg = {
        .offline_init_config = {.name = "gd_wheel", .timeout_ms = 100, .beep_times = 1, .enable = 1},
        .transport           = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 1,
            },
        .controller_init_config =
            {
                /* ⚠️ Kp/MaxOut 必须都给非零值: 两个都是 0 时 PID 输出恒为 0(轮子一点都不转),
                 * 而且 MaxOut 会被 f_Output_Limit 用作硬限幅, 这里就取 max_torque。
                 * 增益换算依据见 guodong_def.h 的 CHASSIS_WHEEL_SPEED_KP 注释 */
                .speed_PID =
                    {
                        .Kp     = CHASSIS_WHEEL_SPEED_KP,
                        .Ki     = CHASSIS_WHEEL_SPEED_KI,
                        .Kd     = CHASSIS_WHEEL_SPEED_KD,
                        .MaxOut = TMAX_M3508,
                    },
            },
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .loop_type             = SPEED_LOOP,
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .feedback_reverse_flag = 0, /* TODO(阶段1):按实车确认 */
                .motor_reverse_flag    = 0, /* TODO(阶段1):按实车确认 */
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = CHASSIS_GEAR_RATIO, .max_torque = TMAX_M3508, .torque_constant = KT_M3508_ROTOR}};

    /* [0] LF(左前) = tx_id 1 */
    chassis_motors[0] = Motor_DJI_Init(&cfg);
    if (chassis_motors[0] == NULL)
    {
        LOG_E("LF init failed");
        return;
    }

    /* [1] RF(右前) = tx_id 2 */
    cfg.transport_config.can.tx_id     = 2;
    cfg.offline_init_config.name       = "gd_wheel_rf";
    cfg.offline_init_config.beep_times = 2;
    chassis_motors[1]                  = Motor_DJI_Init(&cfg);

    /* [2] RR(右后) = tx_id 3 */
    cfg.transport_config.can.tx_id     = 3;
    cfg.offline_init_config.name       = "gd_wheel_rr";
    cfg.offline_init_config.beep_times = 3;
    chassis_motors[2]                  = Motor_DJI_Init(&cfg);

    /* [3] LR(左后) = tx_id 4 */
    cfg.transport_config.can.tx_id     = 4;
    cfg.offline_init_config.name       = "gd_wheel_lr";
    cfg.offline_init_config.beep_times = 4;
    chassis_motors[3]                  = Motor_DJI_Init(&cfg);

    for (int i = 0; i < 4; i++)
    {
        if (chassis_motors[i] != NULL) Motor_Stop((Motor_Base *)chassis_motors[i]);
    }

    LOG_I("chassis init done");
}

/* 四轮断流(enableflag=0 -> 驱动发零电流帧, 电机自由滑行) */
static void chassis_stop_all(void)
{
    for (int i = 0; i < 4; i++)
    {
        if (chassis_motors[i] != NULL) Motor_Stop((Motor_Base *)chassis_motors[i]);
    }
}

/* 四轮是否都在线(STATE_ONLINE == 0, 所以取非即是"在线") */
static uint8_t chassis_all_online(void)
{
    for (int i = 0; i < 4; i++)
    {
        if (chassis_motors[i] == NULL) return 0;
        if (Module_Offline_get_device_status(chassis_motors[i]->base.offline_dev)) return 0;
    }
    return 1;
}

/* 4 个轮子的在线位图(bit0=LF, bit1=RF, bit2=RR, bit3=LR), 仅用于调试打印 */
uint8_t Chassis_Get_OnlineMask(void)
{
    uint8_t mask = 0;

    for (int i = 0; i < 4; i++)
    {
        if (chassis_motors[i] != NULL && !Module_Offline_get_device_status(chassis_motors[i]->base.offline_dev)) mask |= (uint8_t)(1u << i);
    }
    return mask;
}

/*
 * @brief 切向 X 型全向轮逆解(本车用这个, 不用框架的 Chassis_Omni_Calc)
 * @note  两个 45° 全向轮布局的自转项完全不同: 切向是 wz*(a+b) 四轮同号, 径向是
 *        wz*(a-b) 四轮交替。本车是切向(40cm 正方轮距), 所以能自转。
 *        四项与旧工程 CarBody/Mecanum.c 逐项一致。
 * @param vx/vy  底盘系速度 (m/s)
 * @param wz     自转角速度 (rad/s)
 * 输出: 直接 Motor_SetRef, 单位【转子 rad/s】(框架电机闭环 ref 的口径)
 */
static void chassis_xdrive_calc(float vx, float vy, float wz)
{
    const float a = CHASSIS_WHEEL_BASE_X_M / 2.0f; /* 前后半距 */
    const float b = CHASSIS_WHEEL_BASE_Y_M / 2.0f; /* 左右半距 */

    const float rot = wz * (a + b); /* 四轮同号 */
    /* 45° 全向轮的轮面线速度 = 括号项 × 0.7071(投影到轮面方向, 旧 Mecanum_InverseMotionControl 同款)。
     * ⚠ 不能写 /(r*0.7071): 那是 ×1.4142, 所有轮速指令会大 2 倍 —— 斜推满杆要 11000 RPM、
     *   小陀螺+满杆要 13000 RPM, 都超过 M3508 空载(≈9000 RPM), 电机饱和后车不按预期走。 */
    const float speed_factor = CHASSIS_GEAR_RATIO * 0.70710678f / CHASSIS_WHEEL_RADIUS_M;

    /* 数组顺序 = [LF, RF, RR, LR] */
    float wheel_speed[4];
    wheel_speed[0] = (-vx + vy + rot) * speed_factor; /* LF */
    wheel_speed[1] = (+vx + vy + rot) * speed_factor; /* RF */
    wheel_speed[2] = (+vx - vy + rot) * speed_factor; /* RR */
    wheel_speed[3] = (-vx - vy + rot) * speed_factor; /* LR */

    for (int i = 0; i < 4; i++) Motor_SetRef((Motor_Base *)chassis_motors[i], wheel_speed[i]);
}

void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd)
{
    float chassis_wz;

    /* 缺一个轮子的麦轮底盘是不受控的 -> 四轮必须都在线 */
    if (chassis_cmd == NULL || !chassis_all_online() || chassis_cmd->chassis_mode == chassis_zero_force)
    {
        chassis_stop_all();
        return;
    }

    for (int i = 0; i < 4; i++) Motor_Start((Motor_Base *)chassis_motors[i]);

    /* 根据控制模式设定旋转速度 */
    switch (chassis_cmd->chassis_mode)
    {
    case chassis_rotate: // 小陀螺: 固定自旋 + 全向移动
        chassis_wz = CHASSIS_SPIN_W_RAD_S;
        break;
    case chassis_rotate_reverse: // 反向小陀螺
        chassis_wz = -CHASSIS_SPIN_W_RAD_S;
        break;
    case chassis_follow_gimbal_yaw: // 跟随云台
        chassis_wz = 0.0f;              /* TODO(阶段2): PIDCalculate(&chassis_follow_pid, -chassis_cmd->offset_angle, 0); chassis_wz = chassis_follow_pid.Output; */
        break;
    default:
        chassis_wz = 0.0f;
        break;
    }

    /* 云台系 -> 底盘系(底盘逆时针为正, 云台命令以云台指向为 x, 右手系) */
    float total_angle_rad = chassis_cmd->offset_angle * DEGREE_2_RAD;
    float cos_theta       = arm_cos_f32(total_angle_rad);
    float sin_theta       = arm_sin_f32(total_angle_rad);
    float chassis_vx      = chassis_cmd->vx * cos_theta - chassis_cmd->vy * sin_theta;
    float chassis_vy      = chassis_cmd->vx * sin_theta + chassis_cmd->vy * cos_theta;

    chassis_xdrive_calc(chassis_vx, chassis_vy, chassis_wz);
}
