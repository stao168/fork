/*
 * @file    shoot_func.c (云台板) —— 骨架
 * @brief   摩擦轮初始化已按旧工程事实填好;控制逻辑待实现
 *
 * 旧工程事实(CarBody/Gimbal.c):
 *   摩擦轮 = M3508 x2 @ CAN1(ESC ID 1/2 -> 反馈 0x201/0x202)
 *   旧 Gimbal.c 实际下发 ±5200 RPM(**转子口径**, 左负右正;Parameter.h 的 4200 无调用者)
 *   拨盘电机在【底盘板】,本板只负责给出"拨弹许可"
 *   !! FRICTION_WHEEL_RPM 是【转子】RPM, 换算成框架 ref 要 ×RPM_2_RAD_PER_SEC
 */
#include "shoot_func.h"

#if MODULE_MOTOR

#include "module_offline.h"
#include "motor_dji.h"
#include "user_lib.h"

#define LOG_TAG "app_shoot"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static DJI_Motor_t *friction_l = NULL;
static DJI_Motor_t *friction_r = NULL;

static uint8_t shoot_perm = 0;

void shoot_init(void)
{
    Motor_Init_Config_s cfg = {
        .offline_init_config = {.name = "gd_fric_l", .timeout_ms = 100, .beep_times = 4, .enable = 1},
        .transport           = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 1, /* 旧 FrictionWheel_Left = M3508_1 */
            },
        .controller_init_config = {.speed_PID = {.Kp = 0.0f}}, /* TODO(阶段3):标定 */
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .loop_type             = SPEED_LOOP,
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .motor_reverse_flag    = 0, /* TODO(阶段1):左右轮方向 */
            },
        .motor_init_info = {.motor_type = M3508, .gear_ratio = CHASSIS_GEAR_RATIO, .max_torque = TMAX_M3508, .torque_constant = KT_M3508_ROTOR}};
    friction_l = Motor_DJI_Init(&cfg);
    if (friction_l == NULL)
    {
        LOG_E("friction_l init failed");
        return;
    }

    cfg.transport_config.can.tx_id         = 2; /* 旧 FrictionWheel_Right = M3508_2 */
    cfg.offline_init_config.name           = "gd_fric_r";
    cfg.offline_init_config.beep_times     = 5;
    friction_r                             = Motor_DJI_Init(&cfg);
    if (friction_r == NULL)
    {
        LOG_E("friction_r init failed");
        return;
    }

    Motor_Stop((Motor_Base *)friction_l);
    Motor_Stop((Motor_Base *)friction_r);
}

/*
 * @brief 摩擦轮控制 —— TODO(阶段3)实现
 *   目标:shoot_on + friction_on 时按 FRICTION_WHEEL_RPM 设转速(换算见头文件说明),
 *        并据此给出 shoot_perm 供 0x310 下发。
 *   当前:仅安全停机。
 */
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    (void)shoot_cmd;
    shoot_perm = 0;

    if (friction_l != NULL) Motor_Stop((Motor_Base *)friction_l);
    if (friction_r != NULL) Motor_Stop((Motor_Base *)friction_r);
}

uint8_t Shoot_Get_Perm(void) { return shoot_perm; }

#else /* !MODULE_MOTOR —— 阶段1: 摩擦轮不参与 */

void shoot_init(void) {}

void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd) { (void)shoot_cmd; }

uint8_t Shoot_Get_Perm(void) { return 0; }

#endif
