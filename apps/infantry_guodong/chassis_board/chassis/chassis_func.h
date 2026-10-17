#ifndef _CHASSIS_FUNC_H_
#define _CHASSIS_FUNC_H_

#include "guodong_def.h"

/* 麦轮四轮初始化(M3508×4 @ CAN1) */
void chassis_init(void);

/* 底盘控制:速度解算 + 跟随/小陀螺 */
void chassis_func(Chassis_Ctrl_Cmd_t *chassis_cmd);

/* 4 个轮子的在线位图(bit0=LF ... bit3=LR, 0xF=全在线), 调试用 */
uint8_t Chassis_Get_OnlineMask(void);

#endif // _CHASSIS_FUNC_H_
