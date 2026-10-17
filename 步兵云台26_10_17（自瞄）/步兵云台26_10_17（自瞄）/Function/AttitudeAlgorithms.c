#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "IST8310.h"
#include "BMI088.h"
#include "ins_task.h"
#include "arm_math.h"

float AttitudeAlgorithms_q[4];//姿态解算四元数
float AttitudeAlgorithms_RadYaw,AttitudeAlgorithms_RadPitch,AttitudeAlgorithms_RadRoll;//弧度制角度
float AttitudeAlgorithms_DegYaw,AttitudeAlgorithms_DegPitch,AttitudeAlgorithms_DegRoll;//角度制角度
int64_t AttitudeAlgorithms_YawR=0;//角度制偏航角圈数
float AttitudeAlgorithms_RaddYaw_E,AttitudeAlgorithms_RaddPitch_E;

/*
 *函数简介:姿态解算初始化
 *参数说明:无
 *返回类型:无
 *备注:定时器定时1ms,更新四元数并解算角度
 */
void AttitudeAlgorithms_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,ENABLE);//开启时钟
	
	TIM_InternalClockConfig(TIM11);//选择时基单元的时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=500-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=336-1;//配置分频值PSC,默认频率1000Hz
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM11,&TIM_TimeBaseInitStructure);//初始化TIM2
	
	TIM_ClearFlag(TIM11,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位
	
	TIM_ITConfig(TIM11,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;//配置NVIC（配置参数）
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_TRG_COM_TIM11_IRQn;//选择中断通道为TIM11
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//TIM2的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//TIM2的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	
	//IST8310_Init();//初始化IST8310
	BMI088_Init();//初始化BMI088
	//AHRS_init(AttitudeAlgorithms_q,BMI088_Accel,IST8310_MagneticField);//AHRS初始化
	INS_Init();

	TIM_Cmd(TIM11,ENABLE);//启动定时器
}

float AttitudeAlgorithms_LastDegYaw,AttitudeAlgorithms_ThisDegYaw;//上一次角度制偏航角,本次角度制偏航角
/*
 *函数简介:TIM11定时器更新中断函数
 *参数说明:无
 *返回类型:无
 *备注:定时1ms更新四元数并且解算角度
 */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
	static uint8_t AttitudeAlgorithms_YawFirstFlag=1;//第一次接收数据标志位
	
	if(TIM_GetITStatus(TIM11,TIM_IT_Update)==SET)//检测TIM2更新
	{
		TIM_ClearITPendingBit(TIM11,TIM_IT_Update);//清除标志位
		
//		AHRS_update(AttitudeAlgorithms_q,0.001f,BMI088_Gyro,BMI088_Accel,IST8310_MagneticField);
//		get_angle(AttitudeAlgorithms_q,&AttitudeAlgorithms_RadYaw,&AttitudeAlgorithms_RadPitch,&AttitudeAlgorithms_RadRoll);
//		
//		AttitudeAlgorithms_LastDegYaw=AttitudeAlgorithms_ThisDegYaw;
//		AttitudeAlgorithms_ThisDegYaw=AttitudeAlgorithms_RadYaw*57.295779513082320876798154814105f;//转换为角度制
//		AttitudeAlgorithms_DegPitch=AttitudeAlgorithms_RadPitch*57.295779513082320876798154814105f;//转换为角度制
//		AttitudeAlgorithms_DegRoll=AttitudeAlgorithms_RadRoll*57.295779513082320876798154814105f;//转换为角度制
//		
//		if(AttitudeAlgorithms_YawFirstFlag==0)//获取带圈数的角度制偏航角
//		{
//			if(AttitudeAlgorithms_ThisDegYaw-AttitudeAlgorithms_LastDegYaw>180.0f)AttitudeAlgorithms_YawR--;
//			else if(AttitudeAlgorithms_LastDegYaw-AttitudeAlgorithms_ThisDegYaw>180.0f)AttitudeAlgorithms_YawR++;
//		}
//		else AttitudeAlgorithms_YawFirstFlag=0;
//		AttitudeAlgorithms_DegYaw=AttitudeAlgorithms_YawR*360.0f+AttitudeAlgorithms_ThisDegYaw;

		INS_Task();
		AttitudeAlgorithms_LastDegYaw=AttitudeAlgorithms_ThisDegYaw;
		AttitudeAlgorithms_ThisDegYaw=INS.Yaw;//转换为角度制
		AttitudeAlgorithms_DegPitch=INS.Pitch;//转换为角度制
		AttitudeAlgorithms_DegRoll=INS.Roll;//转换为角度制
		AttitudeAlgorithms_RadYaw=AttitudeAlgorithms_ThisDegYaw*0.01745329251994329576923690768489f;//pi/180
		AttitudeAlgorithms_RadPitch=AttitudeAlgorithms_DegPitch*0.01745329251994329576923690768489f;//pi/180
		AttitudeAlgorithms_RadRoll=AttitudeAlgorithms_DegRoll*0.01745329251994329576923690768489f;//pi/180
		
		if(AttitudeAlgorithms_YawFirstFlag==0)//获取带圈数的角度制偏航角
		{
			if(AttitudeAlgorithms_ThisDegYaw-AttitudeAlgorithms_LastDegYaw>180.0f)AttitudeAlgorithms_YawR--;
			else if(AttitudeAlgorithms_LastDegYaw-AttitudeAlgorithms_ThisDegYaw>180.0f)AttitudeAlgorithms_YawR++;
		}
		else AttitudeAlgorithms_YawFirstFlag=0;
		AttitudeAlgorithms_DegYaw=AttitudeAlgorithms_YawR*360.0f+AttitudeAlgorithms_ThisDegYaw;
		AttitudeAlgorithms_RadYaw=AttitudeAlgorithms_DegYaw/180.0f*3.1415926f;
		
		float dPitch_B=INS.Gyro[0];
		float dYaw_B=INS.Gyro[2];
		AttitudeAlgorithms_RaddYaw_E=arm_sin_f32(AttitudeAlgorithms_RadRoll)/arm_cos_f32(AttitudeAlgorithms_RadPitch)*dPitch_B+arm_cos_f32(AttitudeAlgorithms_RadRoll)/arm_cos_f32(AttitudeAlgorithms_RadPitch)*dYaw_B;
		AttitudeAlgorithms_RaddPitch_E=arm_cos_f32(AttitudeAlgorithms_RadRoll)*dPitch_B-arm_sin_f32(AttitudeAlgorithms_RadRoll)*dYaw_B;
	}
}
