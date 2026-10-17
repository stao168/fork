#ifndef __EPUSHROD_H
#define __EPUSHROD_H

typedef enum
{
	EPushRod_Long = 0,
	EPushRod_Ready,
	EpushRod_Short,
}E_Status;


void EPushRod_Init(void);
void EPushRod_Control(void);

#endif
