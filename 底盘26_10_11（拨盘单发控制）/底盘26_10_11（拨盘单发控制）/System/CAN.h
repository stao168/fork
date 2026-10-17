#include <stm32f4xx.h>
#ifndef __CAN_H
#define __CAN_H

typedef enum{
	CAN1_M3508_1 = 0,
	CAN1_M3508_2,
	CAN1_M3508_3,
	CAN1_M3508_4,
	CAN1_M2006_5,
	CAN2_CToC_Slave,
//	CAN2_GM6020,
} CAN_Devices;

void MyCAN_Init(void);//CAN总线初始化

#endif
