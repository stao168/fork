#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Remote.h"
#include "EPushRod.h"

uint8_t TurntoLong_Flag, TurntoShort_Flag;
uint8_t Turn_Cnt;
uint8_t EPushRod_Status = EPushRod_Ready;

/*
 *函数简介:电推杆初始化
 *参数说明:无
 *返回类型:无
 *备注:7-PB14(伸), 5-PB15(缩), 3-PB13(伸), 1-PB12(缩)
 */
void EPushRod_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
}

void EPushRod_LongClear(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	GPIO_SetBits(GPIOB,GPIO_Pin_14);
}

void EPushRod_ShortClear(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	GPIO_SetBits(GPIOB,GPIO_Pin_15);
}

void EPushRod_TruntoLong(void)
{
	EPushRod_ShortClear();
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	GPIO_ResetBits(GPIOB,GPIO_Pin_14);
	TurntoLong_Flag = 1;
}

void EPushRod_TruntoShort(void)
{
	EPushRod_LongClear();
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	TurntoShort_Flag = 1;
}

void EPushRod_Control(void)
{
	if(EPushRod_Status == EPushRod_Ready)
	{
		Turn_Cnt = 0;
		if(Remote_RxData.Remote_SWA==0x70F)
		{
			EPushRod_TruntoLong();
			EPushRod_Status = EPushRod_Long;
		}
		else if(Remote_RxData.Remote_SWA==0x400)
		{
			EPushRod_TruntoShort();
			EPushRod_Status = EpushRod_Short;
		}
	}
	else
	{
		Turn_Cnt ++;
		if(Turn_Cnt >= 10)
		{
			EPushRod_LongClear();
			EPushRod_ShortClear();
		}
		if(Turn_Cnt >= 20)
		{
			Turn_Cnt = 20;
			TurntoLong_Flag = 0;
			TurntoShort_Flag = 0;
			EPushRod_Status = EPushRod_Ready;
		}
	}
}
