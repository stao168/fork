#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "PWM.h"

#define PWM_DutyRange		10000//PWM1占空比量程

/*
 *函数简介:PWM初始化
 *参数说明:PWM编号枚举
 *参数说明:PWM频率
 *参数说明:浮点型PWM占空比
 *返回类型:无
 *备注:PWM的PCLK频率为168MHz
 *备注:PWM1~4为TIM1,分别为CH1~4,PWM5~7为TIM8,分别为CH1~3
 *备注:ARR=PWM_DutyRange,PSC=PCLK/PWM_DutyRange/Freq,CCR=PWM_DutyRange*Duty/100
 *备注:对于一个定时器(TIM1或TIM8),只能有一个确定的频率
 *备注:占空比范围0~100
 *备注:	PWM1的C1为TIM1CH1(PE9)
 *		PWM2的C2为TIM1CH2(PE11)
 *		PWM3的C3为TIM1CH3(PE13)
 *		PWM4的C4为TIM1CH4(PE14)
 *		PWM5的C5为TIM8CH1(PC6)
 *		PWM6的C6为TIM8CH2(PI6)
 *		PWM7的C7为TIM8CH3(PI7)
*/
void PWM_Init(PWM_TypeDef PWMx,uint16_t Freq,float Duty)
{
	TIM_TypeDef *TIMx;//TIM号
	if(PWMx==PWM1 || PWMx==PWM2 || PWMx==PWM3 || PWMx==PWM4)//PWM1~4
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);//开启时钟
	
		TIM_InternalClockConfig(TIM1);//选择时基单元TIM1
		TIMx=TIM1;//TIM为TIM1
	}
	else//PWM5~7
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
		if(PWMx==PWM5)RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		else RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);//开启时钟
		
		TIM_InternalClockConfig(TIM8);//选择时基单元TIM8
		TIMx=TIM8;//TIM为TIM8
	}
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;   
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	switch(PWMx)//配置引脚
	{
		case PWM1:
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
			GPIO_Init(GPIOE,&GPIO_InitStructure);//配置C1-PE9
			GPIO_PinAFConfig(GPIOE,GPIO_PinSource9,GPIO_AF_TIM1);//开启C1的TIM1复用模式
			break;
		case PWM2:
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
			GPIO_Init(GPIOE,&GPIO_InitStructure);//配置C2-PE11
			GPIO_PinAFConfig(GPIOE,GPIO_PinSource11,GPIO_AF_TIM1);//开启C2的TIM1复用模式
			break;
		case PWM3:
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;
			GPIO_Init(GPIOE,&GPIO_InitStructure);//配置C3-PE13
			GPIO_PinAFConfig(GPIOE,GPIO_PinSource13,GPIO_AF_TIM1);//开启C3的TIM1复用模式
			break;
		case PWM4:
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
			GPIO_Init(GPIOE,&GPIO_InitStructure);//配置C4-PE14
			GPIO_PinAFConfig(GPIOE,GPIO_PinSource14,GPIO_AF_TIM1);//开启C4的TIM1复用模式
			break;
		case PWM5:
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
			GPIO_Init(GPIOC,&GPIO_InitStructure);//配置C5-PC6
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);//开启C5的TIM8复用模式
			break;
		case PWM6:
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
			GPIO_Init(GPIOI,&GPIO_InitStructure);//配置C6-PI6
			GPIO_PinAFConfig(GPIOI,GPIO_PinSource6,GPIO_AF_TIM8);//开启C6的TIM8复用模式
			break;
		case PWM7:
			GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
			GPIO_Init(GPIOI,&GPIO_InitStructure);//配置C7-PI7
			GPIO_PinAFConfig(GPIOI,GPIO_PinSource7,GPIO_AF_TIM8);//开启C7的TIM8复用模式
			break;
	}
	
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_InitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_InitStructure.TIM_Period=PWM_DutyRange-1;//ARR,PWM为百分位
	TIM_InitStructure.TIM_Prescaler=(uint16_t)(168000000/PWM_DutyRange/Freq)-1;//PSC,确定频率
	TIM_InitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIMx,&TIM_InitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//配置输出比较模式
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//配置输出比较的极性
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出使能
	TIM_OCInitStructure.TIM_Pulse=(uint32_t)(PWM_DutyRange*Duty/100);//配置输出比较寄存器CCR的值,确定占空比
	switch(PWMx)
	{
		case PWM1:TIM_OC1Init(TIM1,&TIM_OCInitStructure);break;//配置C1输出PWM
		case PWM2:TIM_OC2Init(TIM1,&TIM_OCInitStructure);break;//配置C2输出PWM
		case PWM3:TIM_OC3Init(TIM1,&TIM_OCInitStructure);break;//配置C3输出PWM
		case PWM4:TIM_OC4Init(TIM1,&TIM_OCInitStructure);break;//配置C4输出PWM
		case PWM5:TIM_OC1Init(TIM8,&TIM_OCInitStructure);break;//配置C5输出PWM
		case PWM6:TIM_OC2Init(TIM8,&TIM_OCInitStructure);break;//配置C6输出PWM
		case PWM7:TIM_OC3Init(TIM8,&TIM_OCInitStructure);break;//配置C7输出PWM
	}
	
	TIM_Cmd(TIMx,ENABLE);//启动定时器
	TIM_CtrlPWMOutputs(TIMx,ENABLE);//开启TIM1或TIM8的PWM输出
}

/*
 *函数简介:PWM设置频率
 *参数说明:PWM编号枚举
 *参数说明:PWM频率
 *返回类型:无
 *备注:对于一个定时器(TIM1或TIM8),只能有一个确定的频率
 */
void PWM_SetFreq(PWM_TypeDef PWMx,uint16_t Freq)
{
	if(PWMx==PWM1 || PWMx==PWM2 || PWMx==PWM3 || PWMx==PWM4)//PWM1~4
		TIM_PrescalerConfig(TIM1,(uint16_t)(168000000/PWM_DutyRange/Freq)-1,TIM_PSCReloadMode_Update);
	else//PWM5~7
		TIM_PrescalerConfig(TIM8,(uint16_t)(168000000/PWM_DutyRange/Freq)-1,TIM_PSCReloadMode_Update);
}

/*
 *函数简介:PWM设置占空比
 *参数说明:PWM编号枚举
 *参数说明:浮点型PWM占空比
 *返回类型:无
 *备注:占空比范围0~100
 */
void PWM_SetDuty(PWM_TypeDef PWMx,float Duty)
{
	switch(PWMx)
	{
		case PWM1:TIM_SetCompare1(TIM1,(uint32_t)(PWM_DutyRange*Duty/100));break;//配置C1输出PWM
		case PWM2:TIM_SetCompare2(TIM1,(uint32_t)(PWM_DutyRange*Duty/100));break;//配置C1输出PWM
		case PWM3:TIM_SetCompare3(TIM1,(uint32_t)(PWM_DutyRange*Duty/100));break;//配置C1输出PWM
		case PWM4:TIM_SetCompare4(TIM1,(uint32_t)(PWM_DutyRange*Duty/100));break;//配置C1输出PWM
		case PWM5:TIM_SetCompare1(TIM8,(uint32_t)(PWM_DutyRange*Duty/100));break;//配置C1输出PWM
		case PWM6:TIM_SetCompare2(TIM8,(uint32_t)(PWM_DutyRange*Duty/100));break;//配置C1输出PWM
		case PWM7:TIM_SetCompare3(TIM8,(uint32_t)(PWM_DutyRange*Duty/100));break;//配置C1输出PWM
	}
}
