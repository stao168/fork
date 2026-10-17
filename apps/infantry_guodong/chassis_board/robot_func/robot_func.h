#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include "guodong_def.h"

/* 0x310 云台板命令 → 底盘/发射命令(含单位还原) */
void Gd_ParseGimbalFrame(const Gd_GimbalToChassis_t *frame, Chassis_Ctrl_Cmd_t *chassis_cmd, Shoot_Ctrl_Cmd_t *shoot_cmd);

/* 0x311 底盘板 → 云台板:状态/裁判数据打包 */
void handle_referee_data(Gd_ChassisToGimbal_t *tx);

#endif // _ROBOT_FUNC_H_
