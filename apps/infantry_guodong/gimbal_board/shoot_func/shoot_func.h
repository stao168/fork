#ifndef _SHOOT_FUNC_H_
#define _SHOOT_FUNC_H_

#include <stdint.h>
#include "guodong_def.h"

/* 摩擦轮初始化(M3508×2 @ CAN1) */
void shoot_init(void);

/* 摩擦轮控制(拨盘在底盘板,本板只输出"拨弹许可") */
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd);

/* 拨弹许可:摩擦轮开启且请求发射时为 1(经 0x310 flags 下发底盘板) */
uint8_t Shoot_Get_Perm(void);

#endif // _SHOOT_FUNC_H_
