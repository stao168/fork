/*
 * @file    shoot_func.c (底盘板) —— 骨架
 * @brief   拨盘(M2006)初始化已按旧工程事实填好;单发/连发逻辑待实现
 *
 * 旧工程事实(CarBody/Dial.c):
 *   拨盘 = M2006 @ CAN1, ESC ID 5(反馈 0x205, 命令帧 0x1FF)
 *   控制 = 单级速度环(Kp=5.0, 限幅 +/-10000, RPM 口径)追 RotorSpeed
 *   单发 = 记录 StartPosition,目标 6480RPM 驱动,【输出轴】转过 >43 度后停(自锁)
 *   连发 = SWC 为下(0x70F)时持续驱动
 *   !! 43 度是输出轴角, 框架 measure.total_angle 是转子角 => 阈值须 *REDUCTION_RATIO_LOADER(36)
 *      "一发对应的转子角"必须台架实测确认; 另建议加超时保护(如 0.5s 未到位即停)
 */
#include "shoot_func.h"
#include "module_offline.h"
#include "motor_dji.h"
#include "user_lib.h"

#define LOG_TAG "app_shoot"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

static DJI_Motor_t *loader = NULL;

void shoot_init(void)
{
    Motor_Init_Config_s cfg = {
        .offline_init_config = {.name = "gd_loader_2006", .timeout_ms = 100, .beep_times = 5, .enable = 1},
        .transport           = MOTOR_TRANSPORT_CAN,
        .transport_config.can =
            {
                .hcan  = BSP_CAN_HANDLE1,
                .tx_id = 5, /* 旧 Dail_ID = M2006_5 */
            },
        .controller_init_config = {.speed_PID = {.Kp = 0.0f}}, /* TODO(阶段3):标定 */
        .setting_init_config =
            {
                .algorithm_type        = CONTROL_PID,
                .loop_type             = SPEED_LOOP,
                .angle_feedback_source = 0,
                .speed_feedback_source = 0,
                .motor_reverse_flag    = 0, /* TODO(阶段1):按实车确认 */
            },
        .motor_init_info = {.motor_type = M2006, .gear_ratio = REDUCTION_RATIO_LOADER, .max_torque = TMAX_M2006, .torque_constant = KT_M2006_ROTOR}};

    loader = Motor_DJI_Init(&cfg);
    if (loader == NULL)
    {
        LOG_E("loader init failed");
        return;
    }
    Motor_Stop((Motor_Base *)loader);
}

/*
 * @brief 拨盘控制 —— TODO(阶段3)实现
 *   目标:实现单发状态机(记录起始角度 -> 驱动 -> 到达阈值/超时即停, 等操作手松开才复位)
 *        与连发/反转;阈值与转速换算见本文件头部说明。
 *   当前:仅安全停机。
 */
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    (void)shoot_cmd;

    if (loader != NULL) Motor_Stop((Motor_Base *)loader);
}
