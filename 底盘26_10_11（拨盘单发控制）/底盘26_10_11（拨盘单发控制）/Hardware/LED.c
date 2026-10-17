#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"

/*
 *函数简介:三色单色LED初始化
 *参数说明:无
 *返回类型:无
 *备注:蓝色-PH10,绿色-PH11,红色-PH12
 */
void LED_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOH,GPIO_Pin_10);
	GPIO_ResetBits(GPIOH,GPIO_Pin_11);
	GPIO_ResetBits(GPIOH,GPIO_Pin_12);//默认三灯均熄灭
}

/*
 *函数简介:蓝色单色LED初始化
 *参数说明:无
 *返回类型:无
 *备注:引脚PH10
 */
void LED_BInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOH,GPIO_Pin_10);//默认熄灭
}

/*
 *函数简介:蓝灯点亮
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_BON(void)
{
	GPIO_SetBits(GPIOH,GPIO_Pin_10);
}

/*
 *函数简介:蓝灯熄灭
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_BOFF(void)
{
	GPIO_ResetBits(GPIOH,GPIO_Pin_10);
}

/*
 *函数简介:蓝灯亮灭反转
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_BTurn(void)
{
	if(GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_10)==0)//当前蓝灯灭
		GPIO_SetBits(GPIOH,GPIO_Pin_10);
	else//当前蓝灯亮
		GPIO_ResetBits(GPIOH,GPIO_Pin_10);
}

/*
 *函数简介:绿色单色LED初始化
 *参数说明:无
 *返回类型:无
 *备注:引脚PH11
 */
void LED_GInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOH,GPIO_Pin_11);//默认熄灭
}

/*
 *函数简介:绿灯点亮
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_GON(void)
{
	GPIO_SetBits(GPIOH,GPIO_Pin_11);
}

/*
 *函数简介:绿灯熄灭
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_GOFF(void)
{
	GPIO_ResetBits(GPIOH,GPIO_Pin_11);
}

/*
 *函数简介:绿灯亮灭反转
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_GTurn(void)
{
	if(GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_11)==0)//当前绿灯灭
		GPIO_SetBits(GPIOH,GPIO_Pin_11);
	else//当前绿灯亮
		GPIO_ResetBits(GPIOH,GPIO_Pin_11);
}

/*
 *函数简介:红色单色LED初始化
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_RInit(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&GPIO_InitStructure);
	
	GPIO_ResetBits(GPIOH,GPIO_Pin_12);//默认熄灭
}

/*
 *函数简介:红灯点亮
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_RON(void)
{
	GPIO_SetBits(GPIOH,GPIO_Pin_12);
}

/*
 *函数简介:红灯熄灭
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_ROFF(void)
{
	GPIO_ResetBits(GPIOH,GPIO_Pin_12);
}

/*
 *函数简介:红灯亮灭反转
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void LED_RTurn(void)
{
	if(GPIO_ReadInputDataBit(GPIOH,GPIO_Pin_12)==0)//当前红灯灭
		GPIO_SetBits(GPIOH,GPIO_Pin_12);
	else//当前红灯亮
		GPIO_ResetBits(GPIOH,GPIO_Pin_12);
}

/*
 *函数简介:RGB混色LED初始化
 *参数说明:无
 *返回类型:无
 *备注:蓝色-PH10,绿色-PH11,红色-PH12
 */
void LED_MaxInit(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM5);//选择时基单元TIM5
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;   
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOH,&GPIO_InitStructure);//配置TIM5的CH1-3
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5);
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5);//开启PH10-12的TIM5复用模式
	
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_InitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_InitStructure.TIM_Period=1000-1;//ARR,PWM为千分位
	TIM_InitStructure.TIM_Prescaler=84-1;//PSC,默认1000Hz
	TIM_InitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM5,&TIM_InitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//配置输出比较模式
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//配置输出比较的极性
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出使能
	TIM_OCInitStructure.TIM_Pulse=0;//配置输出比较寄存器CCR的值
	TIM_OC1Init(TIM5,&TIM_OCInitStructure);
	TIM_OC2Init(TIM5,&TIM_OCInitStructure);
	TIM_OC3Init(TIM5,&TIM_OCInitStructure);
	
	TIM_Cmd(TIM5,ENABLE);//启动定时器
}

/*
 *函数简介:设置混色LED的RGB
 *参数说明:三个8位整型,表示RGB的三个值
 *返回类型:无
 *备注:无
 */
void LED_SetColor(uint8_t R,uint8_t G,uint8_t B)
{
	TIM_SetCompare1(TIM5,B*1000/255);
	TIM_SetCompare2(TIM5,G*1000/255);
	TIM_SetCompare3(TIM5,R*1000/255);
}
