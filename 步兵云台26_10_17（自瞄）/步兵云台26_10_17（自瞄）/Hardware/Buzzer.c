#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Buzzer.h"
#include "Delay.h"

int16_t Buzzer_ToneFreq[37]=
{
	0,
	32107,30305,28604,26999,25483,24053,22703,21429,20226,19091,18019,17008,
	16053,15152,14302,13499,12742,12026,11352,10714,10113,9545,9010,8504,
	8027,7576,7151,6750,6371,6013,5676,5357,5056,4773,4505,4252,
};//蜂鸣器音调频率表

/*
 *函数简介:蜂鸣器初始化
 *参数说明:无
 *返回类型:无
 *备注:无源蜂鸣器
 *备注:默认TIM4-CH3(PD14)
 */
void Buzzer_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM4);//选择时基单元TIM4
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14,GPIO_AF_TIM4);//开启PD14的TIM4复用模式
	
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_InitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_InitStructure.TIM_Period=10-1;//ARR,PWM为十分位
	TIM_InitStructure.TIM_Prescaler=19091-1;//PSC,默认频率低音6(440Hz)
	TIM_InitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM4,&TIM_InitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//配置输出比较模式
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//配置输出比较的极性
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出使能
	TIM_OCInitStructure.TIM_Pulse=0;//配置输出比较寄存器CCR的值,默认占空比0%
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);//配置PD14输出PWM
	
	TIM_Cmd(TIM4,ENABLE);//启动定时器
}

/*
 *函数简介:蜂鸣器发声
 *参数说明:枚举音调
 *参数说明:持续时间,单位ms
 *返回类型:无
 *备注:无
 */
void Buzzer_Time(Buzzer_Tone Tone,uint16_t ms)
{
	if(Tone!=P)//非停止调
	{
		TIM_PrescalerConfig(TIM4,Buzzer_ToneFreq[Tone],TIM_PSCReloadMode_Update);//配置频率
		TIM_SetCompare3(TIM4,5);//占空比50%
	}
	else//停止调(即空拍)
		TIM_SetCompare3(TIM4,0);//占空比0%,即关闭PWM
	
	Delay_ms(ms);
	TIM_SetCompare3(TIM4,0);//占空比0%,即关闭PWM
}

/*
 *函数简介:蜂鸣器打开
 *参数说明:枚举音调
 *返回类型:无
 *备注:无
 */
void Buzzer_ON(Buzzer_Tone Tone)
{
	TIM_Cmd(TIM4,ENABLE);//打开定时器
	
	if(Tone!=P)//非停止调
	{
		TIM_PrescalerConfig(TIM4,Buzzer_ToneFreq[Tone],TIM_PSCReloadMode_Update);//配置频率
		TIM_SetCompare3(TIM4,5);//占空比50%
	}
	else//停止调(即空拍)
		TIM_SetCompare3(TIM4,0);//占空比0%,即关闭PWM
}

/*
 *函数简介:蜂鸣器关闭
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Buzzer_OFF(void)
{
	TIM_SetCompare3(TIM4,0);//占空比0%,即关闭PWM
	TIM_Cmd(TIM4,DISABLE);//关闭定时器
}
