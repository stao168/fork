#ifndef __DM_J4340_H
#define __DM_J4340_H

#include <stdint.h>

typedef enum
{
	DM_J4340_IDReceiveOffset=0x10,
	DM_J4340_1=0x01,
}DM_J4340_ID;

typedef struct
{
	uint8_t Status;
	
	uint8_t First_Flag;
	int64_t r;
	
	float Angle;
	float Position;
	float Speed;
	
	float T;
	float Power;
}DM_J4340_Motor;

extern DM_J4340_Motor DM_J4340_MotorStatus[];//M3508电机状态数组

void DM_J4340_Init(void);
uint8_t DM_J4340_CANSend(DM_J4340_ID ID,uint8_t *Data);
void DM_J4340_Set(DM_J4340_ID ID,float tau);
void DM_J4340_DataProcess(DM_J4340_ID ID,uint8_t *Data);

#endif
