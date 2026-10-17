#include <stm32f4xx.h>
#ifndef __LINKCHECK_H
#define __LINKCHECK_H

extern uint8_t LinkCheck_Error;//连接错误标志位

void LinkCheck_Init(void);//CAN设备连接检测初始化
void LinkCheck_ON(void);//开启掉线检查
void LinkCheck_OFF(void);//关闭掉线检测

#endif
