#ifndef _SHOOT_FUNC_H_
#define _SHOOT_FUNC_H_

#include "guodong_def.h"

/* 拨盘(M2006 @ CAN1)初始化 */
void shoot_init(void);

/* 拨盘控制:单发(角度自锁) / 连发 / 反转 */
void shoot_func(Shoot_Ctrl_Cmd_t *shoot_cmd);

#endif // _SHOOT_FUNC_H_
