#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"

int main(void)
{
	Warming_Init();
	LED_BON();
	Delay_ms(500);
//	RefereeSy stem_Init();
	LinkCheck_Init();
	CloseLoopControl_Init();
//	Remote_Init();
	
//	UART2_Init();
	
	while(1)
	{
//		IWDG_ReloadCounter();//喂狗
//		
//		UART2_Printf("%f,%f\n", Dial_SpeedPID.Target, Dial_SpeedPID.Measure);
//		Delay_ms(10);
	}
}
