/*
 * @file    robot_func.c (底盘板) —— 骨架
 * @brief   板间帧解析与状态打包待实现
 *
 * 旧工程对照:
 *   旧 CToC 用 4 个单向帧(0x149 摇杆 / 0x189 状态+开关 / 0x109 未用 / 0x119 拨弹许可),
 *   新方案是否沿用/合并由你确认(见 guodong_def.h 的【提案】段)。
 */
#include "robot_func.h"

#include "user_lib.h"

#define LOG_TAG "app_robot_func"
#define LOG_LVL LOG_LVL_DBG
#include "ulog_def.h"

/*
 * @brief 0x310 解析: mm/s -> m/s、wz_x10 -> rad/s、offset_ecd -> 度、flags -> 模式
 * @note  板间帧属于"外部输入", 这里做一次范围裁剪 —— 位翻转/半帧都不会让底盘飚出去。
 */
void Gd_ParseGimbalFrame(const Gd_GimbalToChassis_t *frame, Chassis_Ctrl_Cmd_t *chassis_cmd, Shoot_Ctrl_Cmd_t *shoot_cmd)
{
    if (frame == NULL || chassis_cmd == NULL || shoot_cmd == NULL) return;

    chassis_cmd->vx = (float)frame->vx_mmps * 0.001f;
    chassis_cmd->vy = (float)frame->vy_mmps * 0.001f;
    chassis_cmd->wz = (float)frame->wz_x10 * GD_WZ_UNIT_RAD_S;
    VAL_LIMIT(chassis_cmd->vx, -CHASSIS_MAX_SPEED_MPS, CHASSIS_MAX_SPEED_MPS);
    VAL_LIMIT(chassis_cmd->vy, -CHASSIS_MAX_SPEED_MPS, CHASSIS_MAX_SPEED_MPS);
    VAL_LIMIT(chassis_cmd->wz, -CHASSIS_SPIN_W_RAD_S, CHASSIS_SPIN_W_RAD_S);

    /* 编码器差值 -> 度(与步兵三一致: 用 8191, 不是 8192); 值域 ±4096 -> ±180° */
    chassis_cmd->offset_angle = (float)frame->offset_ecd * 360.0f / 8191.0f;

    /* 枚举越界 = 帧损坏 -> 零力 */
    chassis_cmd->chassis_mode = (chassis_mode_e)(frame->flags & GD_G2C_FLAG_MODE_MASK);
    if (chassis_cmd->chassis_mode > chassis_rotate_reverse) chassis_cmd->chassis_mode = chassis_zero_force;

    shoot_cmd->shoot_mode    = (frame->flags & GD_G2C_FLAG_SHOOT_PERM) ? shoot_on : shoot_off;
    shoot_cmd->friction_mode = (frame->flags & GD_G2C_FLAG_FRICTION_ON) ? friction_on : friction_off;
    shoot_cmd->load_mode     = (frame->flags & GD_G2C_FLAG_BURST) ? load_burstfire : load_stop;
}

/*
 * @brief 0x311 状态打包 —— TODO(阶段2)
 *   旧工程两板 RefereeSystem_Init 均未启用,此处先给安全默认值;
 *   启用 MODULE_REFEREE 后再改用 Module_Referee_Get_cmd_data(CMD_ID_*) 取真值。
 */
void handle_referee_data(Gd_ChassisToGimbal_t *tx)
{
    if (tx == NULL) return;

    tx->robot_color      = 0;
    tx->game_progress    = 0;
    tx->current_hp       = 0;
    tx->bullet_allow     = 0;
    tx->shooter_heat_pct = 0;
}
