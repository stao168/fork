#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"
#include "HI04.h"

int main(void)
{
	Warming_Init();//报警初始化
	LED_BON();//蓝灯点亮表示代码在运行
	AttitudeAlgorithms_Init();//姿态解算初始化
	Delay_s(3);//延时,等待校准和模块启动
	LinkCheck_Init();
//	Visual_Init();//视觉初始化
	CloseLoopControl_Init();//闭环控制初始化
	Remote_Init();//遥控器初始化
	
	HI04_Init();
	
	while(1)
	{
		IWDG_ReloadCounter();//喂狗
		
		CToC_MasterSendControl();//CToC发送遥控器控制数据
		CToC_MasterSendData();//CToC发送遥控器摇杆数据
		CToC_MasterSendShootFlag();
		
		Delay_us(1500);//CToC周期1.5ms
		
//		UART2_Printf("%f,%f,%f,%f\n", Gimbal_Pitch4310AnglePositionPID.Target, Gimbal_Pitch4310AnglePositionPID.Measure, Gimbal_Pitch4310AngleSpeedPID.Target, Gimbal_Pitch4310AngleSpeedPID.Measure);
//		UART2_Printf("%.4f,%.4f,%.4f\n",Gimbal_Pitch4310AnglePositionPID.Measure,Gimbal_Pitch4310AngleSpeedPID.Measure,Gimbal_Pitch4310AngleSpeedPID.OUT);		
		
//		Delay_ms(10);
	}
}
