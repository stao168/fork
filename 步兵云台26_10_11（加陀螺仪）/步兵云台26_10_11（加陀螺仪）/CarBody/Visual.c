#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "UART.h"
#include "AttitudeAlgorithms.h"
#include "DM_J4310.h"
#include "arm_math.h"
#include "USB.h"
#include "Visual.h"
#include <string.h>
#include "ins_task.h"

float Visual_Yaw,Visual_Pitch,Visual_Fire;//视觉数据偏航角,视觉数据俯仰角
uint8_t Visual_ReceiveFlag;//视觉数据接收缓冲区,视觉数据接收完成标志位

SendPacket Visual_TxData = {
    .header = 0xAA, 
    .mode   = 1,
	.q = {0.0f, 0.0f, 0.0f, 0.0f},
    .tail   = 0x5A
};
ReceivePacket Visual_RxData;

uint8_t VisualErrorFlag=0;

void Visual_Init(void)
{
	/*===============配置时钟===============*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,ENABLE);//开启时钟

	/*===============配置定时器===============*/
	TIM_InternalClockConfig(TIM12);//选择时基单元的时钟(此处为内部时钟),TIM_ITRxExternalClockConfig()接ITRx时钟,TIM_TIxExternalClockConfig()接TIx捕获通道时钟,TIM_ETRClockMode1Config()接ETR时钟(选择外部时钟模式1),TIM_ETRClockMode2Config()接ETR时钟(选择外部时钟模式2)
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元（配置参数）
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=2000-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=84-1;//配置分频值PSC
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM12,&TIM_TimeBaseInitStructure);//初始化TIM12

	TIM_ClearFlag(TIM12,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位

	/*===============配置接收中断和定时器中断===============*/
	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE);//使能更新中断
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=TIM8_BRK_TIM12_IRQn;//选择中断通道为TIM12
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//TIM12的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//TIM12的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	/*===============使能===============*/
	TIM_Cmd(TIM12,ENABLE);//启动定时器
	
	
	USB_Init();
}

void Visual_DataProcess(void)
{
	uint8_t *Data=USB_ReceiveBuff;//选择存储器
	
	if(Data[0] == 0xBB && Data[22] == 0x5B)
	{
		memcpy(&Visual_RxData, Data, sizeof(ReceivePacket));
        
        Visual_ReceiveFlag = 1;
        VisualErrorFlag = 0;
        
        Visual_Fire = Visual_RxData.fire_advice; 
        
        Data[0] = 0x00;
        Data[22] = 0x00;
	}
	else
		VisualErrorFlag=1;
}

uint16_t Visual_Flag;

void TIM8_BRK_TIM12_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM12,TIM_IT_Update)==SET)//检测TIM2更新
	{
		Visual_Flag ++;
		TIM_ClearITPendingBit(TIM12,TIM_IT_Update);//清除标志位
		
		Visual_TxData.mode=0;
		
        Visual_TxData.q[0] = INS.q[0];
        Visual_TxData.q[1] = INS.q[1];
        Visual_TxData.q[2] = INS.q[2];
        Visual_TxData.q[3] = INS.q[3];

        USB_Send((uint8_t*)&Visual_TxData, sizeof(SendPacket));
		
		Visual_DataProcess();
	}
}
