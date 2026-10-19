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

#include "user_lib.h" /* VAL_LIMIT */

#include <math.h>
#include <stddef.h>

/* 云台闭环需要 INS(姿态) + BMI088(角速度) + HI04(pitch 角速度) + MOTOR(电机) 同时到位;
 * 都关着时本文件退化成文件末尾的空实现。 */
#define GD_GIMBAL_READY (MODULE_MOTOR && MODULE_INS && MODULE_BMI088 && MODULE_HI04)

#if GD_GIMBAL_READY

#include "module_bmi088.h"
#include "module_hi04.h"
#include "module_ins.h"
#include "module_offline.h"
#include "motor_damiao.h"
#include "motor_dji.h"

#define LOG_TAG "app_gimbal"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

/* 1 = 只初始化 yaw(GM6020), 跳过 pitch(DM4310)。现在两个轴都要调, 保持 0;
 * 只调 yaw 时临时改 1 可以避免 DM4310 通电(MIT 模式使能后 CAN 一断会保持力矩)。 */
#define GD_YAW_ONLY 0

static DJI_Motor_t *yaw_motor   = NULL; /* Yaw: GM6020 @ CAN2 */
static DM_Motor_t  *pitch_motor = NULL; /* Pitch: DM-J4310 @ CAN1(阶段2-B 不初始化) */

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
                /* ⚠ MaxOut 必须给: 框架 f_Output_Limit 是无条件限幅, MaxOut=0 时输出恒为 0 */
                /* 角度环输出单位是 rad/s(MaxOut=15.71), 送给速度环当 ref */
                .angle_PID = {.Kp = GIMBAL_YAW_ANGLE_KP, .MaxOut = GIMBAL_YAW_ANGLE_MAX_W},
                /* 【阶段2-C 速度环】Kp 由旧工程换算, 推导见 guodong_def.h 的 GIMBAL_YAW_SPEED_KP */
                .speed_PID = {.Kp = GIMBAL_YAW_SPEED_KP, .MaxOut = TMAX_GM6020},
            },
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .loop_type             = ANGLE_AND_SPEED_LOOP, /* 阶段2-D: 角度环串速度环 */
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

#if !GD_YAW_ONLY
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
                /* 速度反馈用 HI04 的 gyro[1](装在俯仰框上, 实测该分量即俯仰角速度);
                 * 角度反馈仍是 DM4310 输出轴编码器(旧工程同款) */
                .other_speed_feedback_ptr = Module_HI04_get_gyro_ptr() + 1,
                .angle_PID                = {.Kp = GIMBAL_PITCH_ANGLE_KP, .MaxOut = GIMBAL_PITCH_ANGLE_MAX_W},
                .speed_PID                = {.Kp            = GIMBAL_PITCH_SPEED_KP,
                                             .Ki            = GIMBAL_PITCH_SPEED_KI,
                                             .MaxOut        = TMAX_DM4310,
                                             .IntegralLimit = GIMBAL_PITCH_INTEGRAL_LIMIT,
                                             .Improve       = PID_Integral_Limit},
            },
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .loop_type             = ANGLE_AND_SPEED_LOOP, /* 阶段4: 角度环串速度环 */
                .angle_feedback_source = 0,
                .speed_feedback_source = 1, /* HI04 gyro[1](500Hz + 串口延迟);
                                             * 改回 0 则用 DM4310 自带 Speed(延迟低但噪声大) */
                .feedback_reverse_flag = 0, /* TODO(阶段3):按实测确认(HI04 方向) */
                .motor_reverse_flag    = 0, /* TODO(阶段3):按实测确认 */
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
#endif /* !GD_YAW_ONLY */

    /* 上电先停住, 等上层按模式使能 */
    Motor_Stop((Motor_Base *)yaw_motor);
    Motor_Stop((Motor_Base *)pitch_motor); /* GD_YAW_ONLY 时为 NULL, Motor_Stop 有内建保护 */
}

/*
 * @brief 云台控制 —— 【阶段2-D: yaw 角度+速度串级】
 *   摇杆累加到目标角(旧工程 Gimbal.c 同款): 按住持续转, 松手停在当前角, 外力推会回弹。
 *   角度环 measure = INS 多圈 yaw(rad) → 出 rad/s; 速度环 measure = 机体 gyro_z → 出 N·m。
 *   增益/限幅均由旧工程换算, 推导见 guodong_def.h。
 * TODO(阶段2-E): 按 gimbal_mode 分支 + pitch 轴 + 限位。
 */
void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd)
{
    if (gimbal_cmd == NULL) return;

    if (yaw_motor == NULL)
    {
        if (yaw_ecd != NULL) *yaw_ecd = (uint16_t)YAW_CHASSIS_ALIGN_ECD;
        return;
    }

    /* 板间夹角用 GM6020 自己的编码器(不是 IMU), 给底盘板做跟随 */
    if (yaw_ecd != NULL) *yaw_ecd = yaw_motor->measure.ecd;

    /* ⚠ 速度环的反馈是 IMU 陀螺: IMU 一旦挂了 gyro_z 会卡住, PID 以为"永远不够快"而满力矩飞车。
     * BMI088 / INS 都没有注册到 OFFLINE 模块(只注册了 sbus/boardcomm/电机), 只能在这里自己判。 */
    if (!ins->initialized || !bmi088_dev->initialized)
    {
        Motor_Stop((Motor_Base *)yaw_motor);
        return;
    }

    /* 摇杆累加目标角。gimbal_cmd->yaw 已经带符号(右推为负), 所以 += 就是右推时目标角减小。
     * 摇杆回中时 sbus 死区(REMOTE_DEAD_ZONE)保证 ch4 恰好为 0, 目标角不会慢慢漂。 */
    static float   yaw_target_rad;
    static uint8_t yaw_target_inited;

    if (!yaw_target_inited)
    {
        /* 第一次用当前姿态初始化, 避免目标 0 与当前角度差很多时云台猛转 */
        yaw_target_rad    = ins->YawTotalAngle_rad;
        yaw_target_inited = 1;
    }
    yaw_target_rad += GIMBAL_YAW_STICK_STEP_RAD * gimbal_cmd->yaw;

    Motor_Start((Motor_Base *)yaw_motor);
    Motor_SetRef((Motor_Base *)yaw_motor, yaw_target_rad); /* 角度环 ref = 目标角(rad) */

    /* ---- pitch: 速度环, 速度反馈 = HI04 gyro[1](装在俯仰框上) ---- */
    if (pitch_motor != NULL)
    {
        if (Module_HI04_get() == NULL)
        {
            /* HI04 没数据/掉线: 没有速度反馈就不能闭速度环, 直接断力
             * (否则 gyro 卡在旧值, PID 会一直满力矩推下去) */
            Motor_Stop((Motor_Base *)pitch_motor);
        }
        else
        {
            const Hi04_Data_t *hi04 = Module_HI04_get();
            const float        spd  = hi04->gyro_rad_s[1];
            const float        out  = pitch_motor->base.controller.output_torque;
            static uint16_t    blk_cnt;

            /* ---- 【阶段4 角度环】摇杆累加目标角, 目标角直接夹在限位里(旧 Gimbal.c 同款) ---- */
            static float   pitch_target_rad;
            static uint8_t pitch_target_inited;

            if (!pitch_target_inited)
            {
                /* 第一次用当前角度初始化, 避免目标与实测差很多时云台猛甩 */
                pitch_target_rad    = pitch_motor->base.measure.total_angle;
                pitch_target_inited = 1;
            }
            pitch_target_rad += GIMBAL_PITCH_STICK_STEP_RAD * gimbal_cmd->pitch;

            /* 限位: 旧工程把 Target 夹在 [-1.45, -0.72]; 用车上的 total_angle 时要减掉标定偏置 */
            VAL_LIMIT(pitch_target_rad, PITCH_LIMIT_UP_RAD - PITCH_ANGLE_OFFSET_RAD, PITCH_LIMIT_DOWN_RAD - PITCH_ANGLE_OFFSET_RAD);

            /* 堵转 / 撞机械限位保护: 角度环会持续出力试图到位, 卡住时表现为 "力矩很大却没转速"。
             * 不需要 ref 判断 —— 静止时输出只有前馈(-0.85), 远小于 0.5*TMAX, 不会误触发。 */
            if (fabsf(spd) < 0.15f && fabsf(out) > 0.5f * TMAX_DM4310)
            {
                if (++blk_cnt >= 50) /* 2ms × 50 = 100ms */
                {
                    blk_cnt = 0;
                    /* 先清前馈再停, 否则停机后前馈还在, 会被推着又撞一次 */
                    Motor_SetForwardTorque((Motor_Base *)pitch_motor, 0.0f);
                    Motor_Stop((Motor_Base *)pitch_motor);
                    LOG_W("pitch blocked (limit hit?), force stop");
                    return;
                }
            }
            else
            {
                blk_cnt = 0;
            }

            Motor_Start((Motor_Base *)pitch_motor);
            Motor_SetRef((Motor_Base *)pitch_motor, pitch_target_rad); /* 角度环 ref = 目标角(rad) */
            /* 重力前馈: 旧工程常数 -0.85 N·m, 减轻速度环和积分的负担。
             * ⚠ 符号按实车确认: 加上后若云台掉得更快, 把 GIMBAL_PITCH_FF_NM 取反。 */
            Motor_SetForwardTorque((Motor_Base *)pitch_motor, GIMBAL_PITCH_FF_NM);
        }
    }

    static uint16_t log_div;
    if (++log_div >= 250) /* 2ms × 250 = 500ms */
    {
        log_div                 = 0;
        const Hi04_Data_t *hi04 = Module_HI04_get();
        LOG_I("yaw tgt=%.2f ref=%.2f spd=%.2f out=%.2f | pitch tgt=%.2f ang=%.2f spd=%.2f out=%.2f r=%.1f%s", (double)yaw_target_rad,
              (double)yaw_motor->base.controller.angle_PID.Output, (double)bmi088_dev->gyro[2], (double)yaw_motor->base.controller.output_torque,
              (pitch_motor != NULL) ? (double)pitch_motor->base.controller.ref : 0.0,
              (pitch_motor != NULL) ? (double)pitch_motor->base.measure.total_angle : 0.0, hi04 ? (double)hi04->gyro_rad_s[1] : 0.0,
              (pitch_motor != NULL) ? (double)pitch_motor->base.controller.output_torque : 0.0, hi04 ? (double)hi04->roll_deg : 0.0,
              (hi04 != NULL) ? "" : " [HI04 LOST]");
    }
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
