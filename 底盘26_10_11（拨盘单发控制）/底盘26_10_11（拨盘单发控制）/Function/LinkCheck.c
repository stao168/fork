/*

连接检查初始化					  --->定时器超时--->连接错误,关闭遥控器---
开启定时器	   --->等待CAN接收---|										   |
						^		  --->关闭定时器,更改ID<---等待恢复连接<--
						|					|
						---开启定时器-------

*/

#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "TIM.h"
#include "Remote.h"
#include "CAN.h"
#include "Warming.h"

uint8_t LinkCheck_Error=0;//连接错误标志位

/*
 *函数简介:CAN设备连接检测初始化
 *参数说明:无
 *返回类型:无
 *备注:Freq=Sys_APB1TIM/(PSC+1)/(ARR+1)=84MHz/(PSC+1)/(ARR+1),T=1/Freq=(ARR+1)*(PSC+1)/84MHz
 *备注:默认定时1ms
 */
void LinkCheck_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM13);//选择时基单元的时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元（配置参数）
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=84-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=1000-1;//配置分频值PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM13,&TIM_TimeBaseInitStructure);//初始化TIM13
	
	TIM_ClearFlag(TIM13,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位
	
	TIM_ITConfig(TIM13,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;//配置NVIC（配置参数）
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_UP_TIM13_IRQn;//选择中断通道为TIM13
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//TIM13的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//TIM13的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	TIM_Cmd(TIM13,ENABLE);//启动定时器
	
	MyCAN_Init();//CAN通讯初始化
}

/*
 *函数简介:开启掉线检查
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LinkCheck_ON(void)
{
	TIM_SetCounter(TIM13,0);//复位计数器
	TIM_Cmd(TIM13,ENABLE);
}

/*
 *函数简介:关闭掉线检测
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LinkCheck_OFF(void)
{
	TIM_Cmd(TIM13,DISABLE);
}

/*
 *函数简介:TIM13定时器更新中断函数
 *参数说明:无
 *返回类型:无
 *备注:进入中断即发生CAN设备掉线
 */
void TIM8_UP_TIM13_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM13,TIM_IT_Update)==SET)//检测TIM2更新
	{
		TIM_ClearITPendingBit(TIM13,TIM_IT_Update);//清除标志位
		CAN_Check();
	}
}
