#include "robot_func.h"
#include "module_remote.h"
#include <stdint.h>
#include <string.h>
#include "guodong_def.h"
#include "user_lib.h"

int16_t CalcOffsetAngle(float getyawangle)
{
    float offset_ecd;

    const float ECD_MAX  = 8191.0f;
    const float ECD_HALF = 4095.5f;

    offset_ecd = getyawangle - YAW_CHASSIS_ALIGN_ECD;

    while (offset_ecd > ECD_HALF) offset_ecd -= ECD_MAX;
    while (offset_ecd < -ECD_HALF) offset_ecd += ECD_MAX;

    return (int16_t)offset_ecd;
}

void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl)
{
    if (!Chassis_Ctrl || !Shoot_Ctrl || !Gimbal_Ctrl) return;

    uint8_t state = Module_Remote_get_offline_status();

    if (state & 0x01)
    {
        float ch_scale = 1.0f / (float)(SBUS_CHX_DOWN - SBUS_CHX_BIAS);

        /* 右摇杆 → 底盘 vx/vy(只出归一化 [-1,1], 由 robot_control.c 换算成 m/s) */
        Chassis_Ctrl->vx = (float)Module_Remote_get_channel(2) * ch_scale;  // 上推为正
        Chassis_Ctrl->vy = -(float)Module_Remote_get_channel(1) * ch_scale; // 右推为正

        /* SW4 (CH8) — 底盘模式
         * SBUS_CHX_UP(240)   = 零力, 当实体急停用
         * SBUS_CHX_BIAS(1024)= 跟随云台
         * SBUS_CHX_DOWN(1807)= 小陀螺
         * 三档之外的值一律零力。
         * ⚠️ 步兵三是 UP=跟随/BIAS=小陀螺; 本车把 UP 留给急停, 首次上电看日志 sw= 核对 */
        {
            int16_t ch8 = Module_Remote_get_channel(8);
            if (ch8 == SBUS_CHX_UP)
                Chassis_Ctrl->chassis_mode = chassis_zero_force;
            else if (ch8 == SBUS_CHX_BIAS)
                Chassis_Ctrl->chassis_mode = chassis_follow_gimbal_yaw;
            else if (ch8 == SBUS_CHX_DOWN)
                Chassis_Ctrl->chassis_mode = chassis_rotate;
            else
                Chassis_Ctrl->chassis_mode = chassis_zero_force;
        }

        /* 云台与发射机构: 阶段1 还没启用, 一律给安全值 */
        Gimbal_Ctrl->yaw         = 0.0f;
        Gimbal_Ctrl->pitch       = 0.0f;
        Gimbal_Ctrl->gimbal_mode = gimbal_zero_force;

        Shoot_Ctrl->shoot_mode    = shoot_off;
        Shoot_Ctrl->friction_mode = friction_off;
        Shoot_Ctrl->load_mode     = load_stop;
    }
    else
    {
        Gimbal_Ctrl->gimbal_mode   = gimbal_zero_force;
        Chassis_Ctrl->chassis_mode = chassis_zero_force;
        Shoot_Ctrl->shoot_mode     = shoot_off;
        Shoot_Ctrl->friction_mode  = friction_off;
        Shoot_Ctrl->load_mode      = load_stop;
        memset(Chassis_Ctrl, 0, sizeof(*Chassis_Ctrl));
    }
}
