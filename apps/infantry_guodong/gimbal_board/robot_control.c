/*
 * @file    robot_control.c (云台板) —— 骨架
 * @brief   线程与调用骨架已就位,业务逻辑待实现
 *
 * 目标数据流(尚未实现):
 *   遥控(SBUS huart3) -> RemoteControlSet() -> gimbal_func()/shoot_func() -> 板间下发 0x310
 */
#include "robot_control.h"
#include "module_boardcomm.h"
#include "module_remote.h"
#include "guodong_def.h"
#include "gimbal_func.h"
#include "shoot_func.h"
#include "robot_func.h"
#include "bsp_def.h"
#include "tx_api.h"
#include <math.h>

#if MODULE_INS && MODULE_BMI088
#include "module_bmi088.h"
#include "module_ins.h"
#endif

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static Gimbal_Ctrl_Cmd_t    gimbal_cmd;
static Shoot_Ctrl_Cmd_t     shoot_cmd;
static Chassis_Ctrl_Cmd_t   chassis_cmd;
static uint16_t             yaw_ecd;
static Gd_GimbalToChassis_t g2c; /* 0x310(帧格式为提案,见 guodong_def.h) */
static Gd_ChassisToGimbal_t c2g; /* 0x311 */

static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        /* 遥控器控制输入 */
        RemoteControlSet(&chassis_cmd, &shoot_cmd, &gimbal_cmd);

        /* 云台控制 */
        gimbal_func(&gimbal_cmd, &yaw_ecd);
        /* 发射机构控制 */
        shoot_func(&shoot_cmd);

        /* 归一化摇杆 → 物理量 */
        chassis_cmd.vx = chassis_cmd.vx * CHASSIS_MAX_SPEED_MPS;
        chassis_cmd.vy = chassis_cmd.vy * CHASSIS_MAX_SPEED_MPS;

        /* 平移量圆形限幅(旧 Mecanum.c 同做法): 斜推不超最大速度 */
        float mag = sqrtf(chassis_cmd.vx * chassis_cmd.vx + chassis_cmd.vy * chassis_cmd.vy);
        if (mag > CHASSIS_MAX_SPEED_MPS)
        {
            chassis_cmd.vx = chassis_cmd.vx / mag * CHASSIS_MAX_SPEED_MPS;
            chassis_cmd.vy = chassis_cmd.vy / mag * CHASSIS_MAX_SPEED_MPS;
        }

        /* 板间下发 0x310。vx/vy 用 mm/s、wz 用 0.1rad/s, 都是为了塞进 int16/int8 */
        g2c.vx_mmps    = (int16_t)(chassis_cmd.vx * 1000.0f);
        g2c.vy_mmps    = (int16_t)(chassis_cmd.vy * 1000.0f);
        g2c.wz_x10     = (int8_t)(chassis_cmd.wz / GD_WZ_UNIT_RAD_S);
        g2c.offset_ecd = CalcOffsetAngle((float)yaw_ecd);
        g2c.flags      = (uint8_t)(chassis_cmd.chassis_mode & GD_G2C_FLAG_MODE_MASK);
        if (shoot_cmd.shoot_mode == shoot_on) g2c.flags |= GD_G2C_FLAG_SHOOT_PERM;
        if (shoot_cmd.friction_mode == friction_on) g2c.flags |= GD_G2C_FLAG_FRICTION_ON;
        if (shoot_cmd.load_mode == load_burstfire) g2c.flags |= GD_G2C_FLAG_BURST;
        Module_BoardComm_Send((uint8_t *)&g2c, sizeof(g2c));

#if MODULE_INS && MODULE_BMI088
        /* 【阶段2-A 只读传感器】MOTOR 没开, 这里只有读, 不可能动电机。
         * 手转云台确认三件事: 1) yaw 连续变化不卷绕 2) gyro_z 符号符合右手系 3) 静止时 gyro_z≈0。
         * 这三条决定了速度环的 feedback_reverse_flag 要不要置 1。
         * 单位: YawTotalAngle_rad 是 rad(多圈累计); gyro[] 是 rad/s
         *   (module_bmi088.h 里注释写 "dps" 是错的: BMI088_GYRO_2000_SEN = 2000/32768×π/180
         *    已经是 rad/s/LSB, 和加速度乘 9.80665 是同一套做法) */
        static uint16_t imu_log_div;
        if (++imu_log_div >= 250) /* 任务 2ms × 250 = 500ms */
        {
            imu_log_div                = 0;
            const Ins_t           *ins = Module_INS_get();
            const Bmi088_device_t *imu = Module_BMI088_get_device();
            if (ins != NULL && imu != NULL)
            {
                LOG_I("imu=%d | yaw=%.4frad (%.1fdeg) euler=%.1f/%.1f/%.1fdeg | gyro_z=%.4frad/s | T=%.1fC", ins->initialized,
                      (double)ins->YawTotalAngle_rad, (double)(ins->YawTotalAngle_rad * 57.295779513f), (double)ins->euler_angle[0],
                      (double)ins->euler_angle[1], (double)ins->euler_angle[2], (double)imu->gyro[2], (double)imu->temperature);
            }
            else
            {
                LOG_W("ins/imu not available");
            }
        }
#endif
        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;

    /* 云台/摩擦轮只在启用 MOTOR 时才初始化; 阶段1 只调底盘, 这里整体跳过 */
#if MODULE_MOTOR
    gimbal_init();
    shoot_init();
#else
    LOG_W("MODULE_MOTOR off: skip gimbal/shoot init (阶段1 只调底盘)");
#endif

    /* 注册底盘板回传帧(0x311) */
    Module_BoardComm_RegisterRxBuffer(&c2g, sizeof(c2g));

    /* 两块板硬件完全相同, 烧错镜像只能靠日志认出来 */
    LOG_I("boot: infantry_guodong GIMBAL board firmware (MOTOR=%d)", MODULE_MOTOR);

    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
