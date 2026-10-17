#include <stm32f4xx.h>
#ifndef __CAN_H
#define __CAN_H

typedef enum
{
	CAN1_DM4310 = 0,
	CAN2_GM6020,
	CAN1_M3508_1,
	CAN1_M3508_2,
}CAN_MotorModel;//CAN总线设备分类枚举

extern uint8_t CAN_DeviceNumber;//CAN总线上设备数量
extern uint8_t CAN_IDSelect;//CAN总线上ID列表选择位

void MyCAN_Init(void);//CAN总线初始化

#endif
