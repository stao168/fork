#ifndef _GIMBAL_FUNC_H_
#define _GIMBAL_FUNC_H_

#include <stdint.h>
#include "guodong_def.h"

/* 云台电机初始化(Yaw: GM6020@CAN2, Pitch: DM-J4310@CAN1) */
void gimbal_init(void);

/* 云台闭环 + 输出电机编码器(供板间夹角用) */
void gimbal_func(Gimbal_Ctrl_Cmd_t *gimbal_cmd, uint16_t *yaw_ecd);

#endif // _GIMBAL_FUNC_H_
