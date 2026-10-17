#ifndef __DIAL_H
#define __DIAL_H

#include <stm32f4xx.h>
#include "PID.h"

#define Dail_ID					M2006_5

extern PID_Typedef Dial_SpeedPID;
extern uint8_t Dial_SingleShootFlag, StartSingleShootFlag;

void Dial_Init(void);
void Dial_Control(void);

#endif
