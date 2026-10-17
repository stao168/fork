#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "M2006.h"
#include "Dial.h"
#include "PID.h"
#include "Remote.h"
#include "CToC.h"

uint8_t Dial_SingleShootFlag;

PID_Typedef Dial_SpeedPID, Dial_PositionPID;

void Dial_Init(void)
{
	PID_Init(&Dial_SpeedPID, 0, 5);
	PID_SetParameter(&Dial_SpeedPID, 5.0, 0, 0);
	PID_EnableOutputLimit(&Dial_SpeedPID, 10000);
}

void Dial_CleanPID(void)
{
	PID_Clean(&Dial_SpeedPID);
}

uint8_t StartSingleShootFlag = 0;
float StartPosition;

void Dial_Control(void)
{
	float position = M2006_MotorStatus[Dail_ID - 0x201].ShaftPosition;
	Dial_SpeedPID.Target = 0;
	
	if(Shoot_OpenFlag != 1)
	{
		Dial_SingleShootFlag = 0;
		StartSingleShootFlag = 0;
	}
	else
	{
		if(Dial_SingleShootFlag == 1 && StartSingleShootFlag == 0)
		{
			StartPosition = position;
			StartSingleShootFlag = 1;
		}
		if(StartSingleShootFlag == 1)
		{
			if(position - StartPosition <= 43.0f) Dial_SpeedPID.Target = 6480;
			else
			{
				Dial_SingleShootFlag = 0;
				StartSingleShootFlag = 0;
			}
		}
		else if(Remote_RxData.Remote_SWC == 0x70F) Dial_SpeedPID.Target = 6480;
	}
	
	PID_Calculate(&Dial_SpeedPID, M2006_MotorStatus[Dail_ID - 0x201].RotorSpeed);
	M2006_CANSetHIDCurrent(Dial_SpeedPID.OUT, 0, 0, 0);
}
