#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include "RM_C.h"
#include "HI04.h"

HI04_VisualAttitude_t HI04_VisualAttitude;
uint8_t HI04_VisualAttitudeValid;

int main(void)
{
	Warming_Init();//报警初始化
	LED_BON();//蓝灯点亮表示代码在运行
	AttitudeAlgorithms_Init();//姿态解算初始化
	Delay_s(3);//延时,等待校准和模块启动
	LinkCheck_Init();
	CloseLoopControl_Init();//闭环控制初始化
	Remote_Init();//遥控器初始化
	
	HI04_Init();
	Visual_Init();//视觉初始化
	
	while(1)
	{
		IWDG_ReloadCounter();//喂狗
		
		HI04_Poll();
		if (HI04_Data.received)
		{
			float quat[4];
		
			/*
			* 当前 HI04_Data 由主循环中的 HI04_Poll() 更新。
			* 在其返回后立即复制，得到完整的一帧四元数。
			*/
			quat[0] = HI04_Data.quat[0];
			quat[1] = HI04_Data.quat[1];
			quat[2] = HI04_Data.quat[2];
			quat[3] = HI04_Data.quat[3];
		
			HI04_VisualAttitudeValid =(uint8_t)HI04_QuaternionToVisual(quat, &HI04_VisualAttitude);
		}
		else HI04_VisualAttitudeValid = 0U;
		
		CToC_MasterSendControl();//CToC发送遥控器控制数据
		CToC_MasterSendData();//CToC发送遥控器摇杆数据
		CToC_MasterSendShootFlag();
		
		Delay_us(1500);//CToC周期1.5ms
		
//		UART2_Printf("%f,%f,%f,%f\n", Gimbal_Pitch4310AnglePositionPID.Target, Gimbal_Pitch4310AnglePositionPID.Measure, Gimbal_Pitch4310AngleSpeedPID.Target, Gimbal_Pitch4310AngleSpeedPID.Measure);
//		UART2_Printf("%.4f,%.4f,%.4f\n",Gimbal_Pitch4310AnglePositionPID.Measure,Gimbal_Pitch4310AngleSpeedPID.Measure,Gimbal_Pitch4310AngleSpeedPID.OUT);		
		
//		Delay_ms(10);
	}
}
