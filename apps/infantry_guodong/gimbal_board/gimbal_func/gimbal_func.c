/*
 * @file    gimbal_func.c (云台板) —— 骨架
 * @brief   电机初始化已按旧工程事实填好;闭环逻辑待实现
 *
 * 旧工程事实(CarBody/Gimbal.c):
 *   Yaw  : GM6020(CAN2), 反馈用 IMU 角度/角速度 -> 位置环 -> 速度环
 *   Pitch: DM-J4310(CAN1, CAN ID 2, MIT 力矩), 反馈用电机编码器 -> 位置环 -> 速度环
 *          并有常数前馈力矩 -0.85 N·m
 *   !! 旧工程所有增益都是 RPM / 角度制口径, 且旧 PID 库与本框架 PID 库
 *      (improve 位掩码不同), 增益必须重标定, 故此处一律留 0.0f + TODO。
 */
#include "gimbal_func.h"

#include <stddef.h>

/* 云台闭环需要 INS(姿态) + BMI088(角速度) + MOTOR(电机) 三者同时在位;
 * 阶段1 只调底盘时三者都关着, 本文件退化成文件末尾的空实现。 */
#define GD_GIMBAL_READY (MODULE_MOTOR && MODULE_INS && MODULE_BMI088)

#if GD_GIMBAL_READY

#include "module_bmi088.h"
#include "module_ins.h"
#include "module_offline.h"
#include "motor_damiao.h"
#include "motor_dji.h"

#define LOG_TAG "app_gimbal"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static DJI_Motor_t *yaw_motor   = NULL; /* Yaw: GM6020 @ CAN2 */
static DM_Motor_t  *pitch_motor = NULL; /* Pitch: DM-J4310 @ CAN1 */

const static Ins_t           *ins        = NULL;
const static Bmi088_device_t *bmi088_dev = NULL;

void gimbal_init(void)
{
    ins = Module_INS_get();
    if (ins == NULL)
    {
        LOG_E("ins is null");
        return;
    }
    bmi088_dev = Module_BMI088_get_device();
    if (bmi088_dev == NULL)
    {
        LOG_E("bmi088_dev is null");
        return;
    }

    /* Yaw: GM6020, CAN2, ESC ID 1(反馈 0x205);外部反馈 = IMU */
    Motor_Init_Config_s yaw_config = {
        .offline_init_config = {.name = "gd_yaw_6020", .timeout_ms = 100, .beep_times = 2, .enable = 1},
        .transport           = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE2,
                .tx_id = 1,
            },
        .controller_init_config =
            {
                /* 角度反馈用【多圈累计角】而不是 euler_rad[2](±π 卷绕), 否则过 ±180° 会跳。
                 * 速度反馈这里用的是【机体系 z 轴】角速度; 旧工程用的是地球系 yaw 速率
                 * (sinRoll/cosPitch*gyro_x + cosRoll/cosPitch*gyro_z), 机身有 roll/pitch 时两者不等价;
                 * 标定增益时若发现跟随有偏差, 先把这一项换成地球系速率。 */
                .other_angle_feedback_ptr = &ins->YawTotalAngle_rad,
                .other_speed_feedback_ptr = &bmi088_dev->gyro[2],
                .angle_PID                = {.Kp = 0.0f}, /* TODO(阶段3):标定 */
                .speed_PID                = {.Kp = 0.0f}, /* TODO(阶段3):标定 */
            },
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .loop_type             = ANGLE_AND_SPEED_LOOP,
                .angle_feedback_source = 1,
                .speed_feedback_source = 1,
                .feedback_reverse_flag = 0, /* TODO(阶段1):按实车确认 */
                .motor_reverse_flag    = 0, /* TODO(阶段1):按实车确认 */
            },
        .motor_init_info = {.motor_type = GM6020_CURRENT, .gear_ratio = 1, .max_torque = TMAX_GM6020, .torque_constant = KT_GM6020}};
    yaw_motor = Motor_DJI_Init(&yaw_config);
    if (yaw_motor == NULL)
    {
        LOG_E("yaw_motor init failed");
        return;
    }

    /* Pitch: DM-J4310, CAN1, CAN ID 2(命令 0x02 / Master ID 0x12),MIT 模式 */
    Motor_Init_Config_s pitch_config = {
        .offline_init_config = {.name = "gd_pitch_4310", .timeout_ms = 100, .beep_times = 3, .enable = 1},
        .transport           = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 0x02,
                .rx_id = 0x12,
            },
        .controller_init_config =
            {
                .angle_PID = {.Kp = 0.0f}, /* TODO(阶段3):标定 */
                .speed_PID = {.Kp = 0.0f}, /* TODO(阶段3):标定 */
            },
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .loop_type             = ANGLE_AND_SPEED_LOOP,
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .feedback_reverse_flag = 0, /* TODO(阶段1):按实车确认 */
                .motor_reverse_flag    = 0, /* TODO(阶段1):按实车确认 */
            },
        .motor_init_info = {.motor_type = DM4310, .gear_ratio = 10, .max_torque = TMAX_DM4310, .torque_constant = KT_DM4310}};
    pitch_motor = Motor_DM_Init(&pitch_config, DM_MIT_MODE);
    if (pitch_motor == NULL)
    {
        LOG_E("pitch_motor init failed");
        return;
    }
    Motor_DM_Cmd(pitch_motor, DM_CMD_CLEAR_ERROR);
    Motor_DM_Cmd(pitch_motor, DM_CMD_MOTOR_START);

    /* 骨架:上电先停住,等闭环实现 */
    Motor_Stop((Motor_Base *)yaw_motor);
    Motor_Stop((Motor_Base *)pitch_motor);
}

/*
 * @brief 云台闭环 —— TODO(阶段3)实现
 *   目标:按 gimbal_cmd->gimbal_mode 分支(zero_force / gyro / auto),
 *        pitch 限位用 PITCH_LIMIT_DOWN_RAD / PITCH_LIMIT_UP_RAD,
 *        前馈用 Motor_SetForwardTorque(&pitch_motor->base, GIMBAL_PITCH_FF_NM),
 *        最后回填 *yaw_ecd = yaw_motor->measure.ecd(供板间夹角)。
 *   当前:仅安全停机。
 */
void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    if (gimbal_cmd == NULL) return;
    (void)yaw_ecd;

    if (yaw_motor != NULL) Motor_Stop((Motor_Base *)yaw_motor);
    if (pitch_motor != NULL) Motor_Stop((Motor_Base *)pitch_motor);
}

#else /* !GD_GIMBAL_READY —— 阶段1: 云台不参与 */

void gimbal_init(void) {}

void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    (void)gimbal_cmd;
    /* 阶段1 没有云台电机, 必须上报"云台与底盘对齐"(=YAW_CHASSIS_ALIGN_ECD),
     * 否则底盘板会拿 CalcOffsetAngle(0) = -3641 这个假夹角去旋转速度矢量(整整差 160°)。 */
    if (yaw_ecd != NULL) *yaw_ecd = (uint16_t)YAW_CHASSIS_ALIGN_ECD;
}

#endif
