#include <stm32f4xx.h>
#ifndef __REMOTE_H
#define __REMOTE_H

typedef struct
{
	uint16_t Remote_R_RL;//通道0-右摇杆左右(右为大),范围364(最左端)~1684(最右端),默认值1024(中间)
	uint16_t Remote_R_UD;//通道1-右摇杆上下(上为大),范围364(最下端)~1684(最上端),默认值1024(中间)
	uint16_t Remote_L_RL;//通道2-左摇杆左右(右为大),范围364(最左端)~1684(最右端),默认值1024(中间)
	uint16_t Remote_L_UD;//通道3-左摇杆上下(上为大),范围364(最下端)~1684(最上端),默认值1024(中间)
	
	uint16_t Remote_SWA;
	uint16_t Remote_SWB;
	
	uint16_t Remote_SWC;
	uint16_t Remote_SWD;
	uint16_t Remote_VRA;
	uint16_t Remote_VRB;
	
	uint8_t Remote_Online;//遥控器在线标志位（按照文档添加）
}Remote_Data;//遥控器接收结构体

extern Remote_Data Remote_RxData;//遥控器接收数据
extern uint8_t Remote_Status;//遥控器连接状态,默认未连接(0)
extern uint8_t Remote_StartFlag;//遥控器启动标志位,0-未在启动阶段,1-准备启动,2-第一次接收到数据

void Remote_Init(void);//遥控器初始化
void Remote_ON(void);//遥控器开启
void Remote_OFF(void);//遥控器关闭

#endif
