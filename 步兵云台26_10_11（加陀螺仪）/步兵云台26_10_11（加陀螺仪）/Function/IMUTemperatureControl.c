#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "BMI088.h"
#include "PID.h"
#include "Warming.h"

PID_Typedef IMUTemperatureControl_PID;//IMU恒温控制PID
uint8_t IMUTemperatureControl_OpenFlag;//IMU恒温控制打开标志位

/*
 *函数简介:IMU恒温控制PID初始化
 *参数说明:无
 *返回类型:无
 *备注:用于CloseLoopControl文件中的CloseLoopControl_Init函数以初始化PID
 *备注:默认恒温控制到40℃
 */
void IMUTemperatureControl_PIDInit(void)
{
	PID_Init(&IMUTemperatureControl_PID,40, 1);
	PID_SetParameter(&IMUTemperatureControl_PID,20,0.0003,0);
	PID_EnableOutputLimit(&IMUTemperatureControl_PID,100);
}

/*
 *函数简介:IMU恒温控制初始化
 *参数说明:无
 *返回类型:无
 *备注:使用定时器TIM10的通道1(PF6)产生PWM控制加热功率
 *备注:PWM频率为1000Hz
 */
void IMUTemperatureControl_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM10);//选择时基单元TIM10
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;   
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOF,GPIO_PinSource6,GPIO_AF_TIM10);//开启PF6的TIM10复用模式
	
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	TIM_InitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_InitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_InitStructure.TIM_Period=1000-1;//ARR,PWM为千分位1ms
	TIM_InitStructure.TIM_Prescaler=168-1;//PSC
	TIM_InitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM10,&TIM_InitStructure);
	
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;//配置输出比较模式
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;//配置输出比较的极性
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;//输出使能
	TIM_OCInitStructure.TIM_Pulse=0;//配置输出比较寄存器CCR的值
	TIM_OC1Init(TIM10,&TIM_OCInitStructure);//配置PF6输出PWM
	
	TIM_Cmd(TIM10,ENABLE);//启动定时器
	
	IMUTemperatureControl_PIDInit();
	IMUTemperatureControl_OpenFlag=1;//IMU恒温控制打开
}

/*
 *函数简介:IMU恒温控制
 *参数说明:无
 *返回类型:无
 *备注:当温度高于50℃时会报警并且程序停止,具体报警现象见Warming.h
 */
void IMUTemperatureControl_TemperatureControl(void)
{
	if(BMI088_Temperature>50)//温度高于50℃
		while(1) Warming_IMUTemperatureTooHigh();//温度过高报警并停止加热
		
	PID_Calculate(&IMUTemperatureControl_PID,BMI088_Temperature);//PID计算
	TIM_SetCompare1(TIM10,IMUTemperatureControl_PID.OUT*10.0f);//PID补偿
}
