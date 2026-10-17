#ifndef _ROBOT_FUNC_H_
#define _ROBOT_FUNC_H_

#include <stdint.h>
#include "guodong_def.h"

/* 云台-底盘夹角:编码器差归一到 ±4096 */
int16_t CalcOffsetAngle(float yaw_ecd);

/* 遥控(SBUS)→ 底盘/发射/云台命令 */
void RemoteControlSet(Chassis_Ctrl_Cmd_t *Chassis_Ctrl, Shoot_Ctrl_Cmd_t *Shoot_Ctrl, Gimbal_Ctrl_Cmd_t *Gimbal_Ctrl);

#endif // _ROBOT_FUNC_H_
