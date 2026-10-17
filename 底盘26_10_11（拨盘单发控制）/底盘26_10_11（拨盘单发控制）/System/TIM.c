#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "CAN.h"

/*====================	 定时器资源分配   ====================
	TIM1(高级)	················PWM1~PWM4		················APB2			
	TIM2		················定时器			················APB1
	TIM3		················激光			················APB1
	TIM4		················蜂鸣器发声      ················APB1
	TIM5		················混色LED三色控制 ················APB1
	TIM6(基本)	················PID闭环         ················APB1
	TIM7(基本)	················遥控器连接检测  ················APB1
	TIM8(高级)	················PWM5~PWM7       ················APB2
	TIM9		················                ················APB2
	TIM10		················陀螺仪加热电路  ················APB2
	TIM11		················陀螺仪姿态解算  ················APB2
	TIM12		················                ················APB1
	TIM13		················CAN总线设备连接检测 ············APB1
	TIM14		················                ················APB1
  =============================================================*/

/*
（1）高级定时器timer1, timer8以及通用定时器timer9, timer10, timer11的时钟来源是APB2总线
（2）通用定时器timer2~timer5，通用定时器timer12~timer14以及基本定时器timer6,timer7的时钟来源是APB1总线
*/

/*
 *函数简介:TIM2定时器初始化
 *参数说明:无
 *返回类型:无
 *备注:默认定时1ms
 *备注:Freq=Sys_APB1TIM/(PSC+1)/(ARR+1)=84MHz/(PSC+1)/(ARR+1)
 */
void TIM_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM2);//选择时基单元的时钟(此处为内部时钟),TIM_ITRxExternalClockConfig()接ITRx时钟,TIM_TIxExternalClockConfig()接TIx捕获通道时钟,TIM_ETRClockMode1Config()接ETR时钟(选择外部时钟模式1),TIM_ETRClockMode2Config()接ETR时钟(选择外部时钟模式2)
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元（配置参数）
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=200-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=420-1;//配置分频值PSC,默认频率800Hz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM2
	
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;//配置NVIC（配置参数）
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;//选择中断通道为TIM2
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//TIM2的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//TIM2的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	TIM_Cmd(TIM2,ENABLE);//启动定时器
}
