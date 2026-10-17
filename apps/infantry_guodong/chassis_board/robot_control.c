/*
 * @file    robot_control.c (底盘板) —— 骨架
 * @brief   线程与调用骨架已就位,业务逻辑待实现
 *
 * 目标数据流(尚未实现):
 *   收 0x310(云台板命令) -> chassis_func()/shoot_func() -> 回传 0x311
 *   旧工程:底盘板无遥控、无 IMU,摇杆与开关全部来自云台板的板间帧
 */
#include "robot_control.h"
#include "chassis_func.h"
#include "module_boardcomm.h"
#include "module_offline.h"
#include "robot_func.h"
#include "shoot_func.h"
#include "guodong_def.h"
#include "bsp_def.h"
#include "tx_api.h"

#define LOG_TAG "app_robot_control"
#define LOG_LVL LOG_LVL_INFO
#include "ulog_def.h"

static TX_THREAD                  robot_control_thread;
APPS_STACK_SECTION static uint8_t robot_control_thread_stack[1024];

static Chassis_Ctrl_Cmd_t chassis_cmd;
static Shoot_Ctrl_Cmd_t   shoot_cmd;

static Gd_GimbalToChassis_t g2c; /* 0x310(帧格式为提案,见 guodong_def.h) */
static Gd_ChassisToGimbal_t c2g; /* 0x311 */

static void robot_control_task(ULONG thread_input)
{
    (void)thread_input;

    while (1)
    {
        /* 板间断链 -> 零力。底盘板自己没有遥控/IMU, 收不到命令就绝不能动 */
        if (Module_BoardComm_Get_Offline_State() == STATE_OFFLINE)
        {
            chassis_cmd.vx           = 0.0f;
            chassis_cmd.vy           = 0.0f;
            chassis_cmd.wz           = 0.0f;
            chassis_cmd.offset_angle = 0.0f;
            chassis_cmd.chassis_mode = chassis_zero_force;

            shoot_cmd.shoot_mode    = shoot_off;
            shoot_cmd.friction_mode = friction_off;
            shoot_cmd.load_mode     = load_stop;
        }
        else
        {
            /* 解析云台板下发的 0x310 */
            Gd_ParseGimbalFrame(&g2c, &chassis_cmd, &shoot_cmd);
        }

        /* 执行: 四轮麦轮(拨盘/摩擦轮后面阶段再做) */
        chassis_func(&chassis_cmd);
        shoot_func(&shoot_cmd);

        /* 状态回传 0x311, 让云台板能看到链路还活着 */
        handle_referee_data(&c2g);
        Module_BoardComm_Send((uint8_t *)&c2g, sizeof(c2g));

        /* TODO(debug): 每 500ms 打一行实收命令 + 链路状态, 查完删 */
        static uint16_t log_div;
        if (++log_div >= 250)
        {
            log_div = 0;
            LOG_I("link=%s wheels=0x%X | vx=%d vy=%d wz=%d mm/s | mode=%d", (Module_BoardComm_Get_Offline_State() == STATE_OFFLINE) ? "OFFLINE" : "ONLINE",
                  Chassis_Get_OnlineMask(), (int)(chassis_cmd.vx * 1000.0f), (int)(chassis_cmd.vy * 1000.0f), (int)(chassis_cmd.wz * 1000.0f),
                  (int)chassis_cmd.chassis_mode);
        }

        tx_thread_sleep(2);
    }
}

void robot_control_init(void)
{
    UINT status;

    chassis_init();
    shoot_init();

    /* 注册云台板命令帧(0x310) */
    Module_BoardComm_RegisterRxBuffer(&g2c, sizeof(g2c));

    /* 两块板硬件完全相同, 烧错镜像只能靠日志认出来 */
    LOG_I("boot: infantry_guodong CHASSIS board firmware");

    status = tx_thread_create(&robot_control_thread, "robot_control_thread", robot_control_task, 0, robot_control_thread_stack, 1024, 30, 30,
                              TX_NO_TIME_SLICE, TX_AUTO_START);
    if (status != TX_SUCCESS)
    {
        LOG_E("robot_control_task failed!");
        return;
    }

    LOG_I("robot_control init success!");
}
