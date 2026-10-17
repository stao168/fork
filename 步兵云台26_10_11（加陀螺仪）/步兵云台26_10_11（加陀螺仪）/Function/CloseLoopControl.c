#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Remote.h"
#include "IMUTemperatureControl.h"
#include "LinkCheck.h"
#include "Warming.h"
#include "Gimbal.h"
#include "RefereeSystem.h"
#include "Dial.h"

/*
 *函数简介:闭环控制初始化
 *参数说明:无
 *返回类型:无
 *备注:使用定时器TIM6进行闭环,闭环周期2ms
 */
void CloseLoopControl_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM6);//选择时基单元的时钟(TIM6)
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元（配置参数）
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=500-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=336-1;//配置分频值PSC,默认定时2ms
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ClearFlag(TIM6,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;//配置NVIC（配置参数）
	NVIC_InitStructure.NVIC_IRQChannel=TIM6_DAC_IRQn;//选择中断通道为TIM6
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//TIM2的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=3;//TIM2的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	TIM_Cmd(TIM6,ENABLE);//启动定时器
	
	IMUTemperatureControl_Init();//恒温控制初始化
	Gimbal_Init();//云台初始化
}

/*
 *函数简介:TIM6定时器更新中断函数
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)==SET)//检测TIM6更新
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);//清除标志位
		
		/*===============恒温闭环控制===============*/
		if(IMUTemperatureControl_OpenFlag==1) IMUTemperatureControl_TemperatureControl();
		
		/*===============云台闭环控制===============*/
		if(Can_Error_Flag == 0) //CAN设备正常连接
		{
			if(Remote_Status == 1)
			{
				Gimbal_Control();
			}
			else
			{
				Gimbal_LinkFlag = 0;
				Shoot_OpenCount = 0;
				Shoot_OpenFlag = 0;
				Warming_MotorControl();
			}
		}
		else
		{
			Gimbal_LinkFlag = 0;
			Shoot_OpenCount = 0;
			Shoot_OpenFlag = 0;
			Warming_LinkError();
			Warming_MotorControl();
		}
	}
}
