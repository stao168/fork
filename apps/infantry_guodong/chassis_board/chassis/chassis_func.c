/*
 * @file    chassis_func.c (底盘板)
 * @brief   四轮全向轮底盘: 摇杆意图 -> 模式决定自转 -> 坐标变换 -> 逆运动学
 *
 * 旧工程事实(CarBody/Mecanum.c + Mecanum.h):
 *   电机 ID: 1=LF(左前) 2=RF(右前) 3=RR(右后) 4=LR(左后), 全在 CAN1
 *   逆解   : 输出【转子 RPM】,旧工程再用 PID 追 RotorSpeed
 *
 * !! 逆运动学: 统一用模块层 modules/algorithm/chassis_type.c 的 Chassis_Omni_Calc。
 *   本车是切向 X 型布局(轮子 ±45°, 40cm×40cm), 自转项 vw*chassis_radius 四轮同号。
 *
 * !! 轮序: chassis_motors = [LF, LR, RR, RF], 直接对齐模块层 Chassis_Omni_Calc 的数组顺序,
 *   所以初始化时写的 tx_id 是 1/4/3/2(不是 1/2/3/4 按序) —— 每个轮子自己的 tx_id 没变,
 *   只是数组下标顺序变了。在线位图 wheels=0x.. 和日志也按这个位序解读。
 */
#include "chassis_func.h"
#include "chassis_type.h"
#include "module_offline.h"
#include "motor_dji.h"
#include "user_lib.h"
#include <stdint.h>

#define LOG_TAG "app_chassis"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static DJI_Motor_t *chassis_motors[4]; /* [LF, LR, RR, RF] —— 与模块层 Chassis_Omni_Calc 的轮序一致 */

/* 模块层逆解用的几何参数 */
static const Chassis_Diff_Config_s chassis_omni_cfg = {
    .wheel_base_x   = CHASSIS_WHEEL_BASE_X_M,
    .wheel_base_y   = CHASSIS_WHEEL_BASE_Y_M,
    .chassis_radius = CHASSIS_RADIUS_M,
    .wheel_radius   = CHASSIS_WHEEL_RADIUS_M,
    .decele_ratio   = CHASSIS_GEAR_RATIO,
};

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

    /* [1] LR(左后) = tx_id 4。数组顺序对齐 Chassis_Omni_Calc 的 [LF, LR, RR, RF], 所以是 1/4/3/2;
     * 蜂鸣次数仍按【轮位】给(LF=1 RF=2 RR=3 LR=4), 和报警名单一致 */
    cfg.transport_config.can.tx_id     = 4;
    cfg.offline_init_config.name       = "gd_wheel_lr";
    cfg.offline_init_config.beep_times = 4;
    chassis_motors[1]                  = Motor_DJI_Init(&cfg);

    /* [2] RR(右后) = tx_id 3 */
    cfg.transport_config.can.tx_id     = 3;
    cfg.offline_init_config.name       = "gd_wheel_rr";
    cfg.offline_init_config.beep_times = 3;
    chassis_motors[2]                  = Motor_DJI_Init(&cfg);

    /* [3] RF(右前) = tx_id 2 */
    cfg.transport_config.can.tx_id     = 2;
    cfg.offline_init_config.name       = "gd_wheel_rf";
    cfg.offline_init_config.beep_times = 2;
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

/* 4 个轮子的在线位图(bit0=LF, bit1=LR, bit2=RR, bit3=RF), 仅用于调试打印。
 * 位序跟 chassis_motors 走 = 模块层 Chassis_Omni_Calc 的轮序 */
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
 * @brief 全向轮逆解后下发到四个电机
 * @note  chassis_motors 的顺序已经是模块层 Chassis_Omni_Calc 要求的 [LF, LR, RR, RF], 直接传
 */
static void chassis_omni_calc(float vx, float vy, float wz) { Chassis_Omni_Calc(chassis_motors, &chassis_omni_cfg, vx, vy, wz); }

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
        chassis_wz =
            0.0f; /* TODO(阶段2): PIDCalculate(&chassis_follow_pid, -chassis_cmd->offset_angle, 0); chassis_wz = chassis_follow_pid.Output; */
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

    chassis_omni_calc(chassis_vx, chassis_vy, chassis_wz);
}
