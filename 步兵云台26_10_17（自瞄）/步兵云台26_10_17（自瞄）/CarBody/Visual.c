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
#include "crc_rm.h"
#include "HI04.h"

float Visual_Yaw,Visual_Pitch;//视觉数据偏航角,视觉数据俯仰角
uint8_t Visual_ReceiveFlag, Visual_Fire;//视觉数据接收完成标志位,视觉开火建议

SendPacket Visual_TxData = {
    .header   = 0x5A,
    .mode     = 0,
    .q        = {1.0f, 0.0f, 0.0f, 0.0f},
    .crc_low  = 0,
    .crc_high = 0
};
ReceivePacket Visual_RxData;

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
	USB_Init();
	TIM_ClearITPendingBit(TIM12, TIM_IT_Update);
	TIM_Cmd(TIM12, ENABLE);
}

void Visual_DataProcess(void)
{
    uint8_t *Data = USB_ReceiveBuff;

    if (Data[0] != 0xA5) return;

    /* 长度为完整帧 12 字节，函数内部校验前 10 字节 */
    if (Verify_CRC16_Check_Sum(Data, sizeof(ReceivePacket)))
    {
        memcpy(&Visual_RxData, Data, sizeof(ReceivePacket));

        Visual_Yaw  = Visual_RxData.target_yaw;   /* rad */
        Visual_Pitch = Visual_RxData.target_pitch; /* rad */
        Visual_Fire = Visual_RxData.fire_advice;

        Visual_ReceiveFlag = 1;
    }
    else Visual_ReceiveFlag = 0;

    Data[0] = 0;
}

uint16_t Visual_Flag; //中断计数，调试用

void TIM8_BRK_TIM12_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM12,TIM_IT_Update)==SET)//检测TIM2更新
	{
		Visual_Flag ++;
		TIM_ClearITPendingBit(TIM12,TIM_IT_Update);//清除标志位
		
        if (HI04_Data.received)
        {
            Visual_TxData.mode = 0;
            Visual_TxData.q[0] = HI04_Data.quat[0];
            Visual_TxData.q[1] = HI04_Data.quat[1];
            Visual_TxData.q[2] = HI04_Data.quat[2];
            Visual_TxData.q[3] = HI04_Data.quat[3];

            Append_CRC16_Check_Sum((uint8_t *)&Visual_TxData, sizeof(Visual_TxData));
            USB_Send((uint8_t *)&Visual_TxData, sizeof(Visual_TxData));
        }

		Visual_DataProcess();
	}
}
