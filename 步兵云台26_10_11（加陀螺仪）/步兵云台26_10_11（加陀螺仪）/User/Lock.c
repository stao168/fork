#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Delay.h"

/*
 *函数简介:安全锁初始化
 *参数说明:无
 *返回类型:无
 *备注:默认安全锁为PF1和PB15,用跳线帽将两个引脚连接即可解锁
 */
void Lock_Init(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_1;
	GPIO_Init(GPIOF,&GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_15);//初始化测试信号引脚电平
}

/*
 *函数简介:安全锁检测
 *参数说明:无
 *返回类型:0-解锁,1-加锁
 *备注:引脚电平上电默认为高电平
 */
uint8_t Lock_Check(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_15);
	Delay_ms(100);
	return GPIO_ReadInputDataBit(GPIOF,GPIO_Pin_1);
}

/*
 *函数简介:安全锁触发
 *参数说明:无
 *返回类型:无
 *备注:清除全部FLASH
 */
void Lock_EraseFLASH(void)
{
	FLASH_Unlock();
	FLASH_EraseAllSectors(VoltageRange_3);
	FLASH_Lock();
}
