#ifndef __CLOSELOOPCONTROL_H
#define __CLOSELOOPCONTROL_H

#include <stm32f4xx.h>

extern uint8_t CloseLoopControl_ErrorFlag;//闭环控制CAN设备故障标志位

void CloseLoopControl_Init(void);//闭环控制初始化

#endif
