#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"

/*
 *函数简介:激光初始化
 *参数说明:无
 *返回类型:无
 *备注:规定引脚为PC8,高电平点亮
 */
void Laser_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);//默认关闭
}

/*
 *函数简介:激光普通模式打开激光
 *参数说明:无
 *返回类型:无
 *备注:规定引脚为PC8,高电平点亮
 */
void Laser_ON(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_8);
}

/*
 *函数简介:激光普通模式关闭激光
 *参数说明:无
 *返回类型:无
 *备注:规定引脚为PC8,低电平关闭
 */
void Laser_OFF(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);
}

/*
 *函数简介:激光PWM模式初始化
 *参数说明:无
 *返回类型:无
 *备注:规定引脚为PC8(TIM3_CH3),占空比越高越亮
 */
void Laser_PWMInit(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM3);//选择时基单元TIM3
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;   
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM3);//开启PC8的TIM3复用模式
	
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_InitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_InitStructure.TIM_Period=1000-1;//ARR,PWM为千分位1ms
	TIM_InitStructure.TIM_Prescaler=84-1;//PSC
	TIM_InitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM3,&TIM_InitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//配置输出比较模式
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//配置输出比较的极性
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出使能
	TIM_OCInitStructure.TIM_Pulse=0;//配置输出比较寄存器CCR的值,默认占空比0
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);//配置CH3(PC8)输出PWM
	
	TIM_Cmd(TIM3,DISABLE);//默认关闭定时器
}

/*
 *函数简介:激光PWM模式以一定亮度打开激光
 *参数说明:亮度百分比,范围0~100,支持一位小数
 *返回类型:无
 *备注:规定引脚为PC8(TIM3_CH3),占空比越高越亮
 */
void Laser_PWMON(float Brightness)
{
	TIM_SetCompare3(TIM3,Brightness*10.0f);//设置占空比
	TIM_Cmd(TIM3,ENABLE);//启动定时器
}

/*
 *函数简介:激光PWM模式关闭激光
 *参数说明:无
 *返回类型:无
 *备注:规定引脚为PC8(TIM3_CH3)
 */
void Laser_PWMOFF(void)
{
	TIM_SetCompare3(TIM3,0);//设置占空比为0
	TIM_Cmd(TIM3,DISABLE);//关闭定时器
}
