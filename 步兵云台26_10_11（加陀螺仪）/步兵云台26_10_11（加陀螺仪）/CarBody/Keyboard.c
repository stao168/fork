#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "UART.h"
#include "Remote.h"

/*
 *函数简介:键盘初始化
 *参数说明:无
 *返回类型:无
 *备注:初始化UART1(USART6)
 */
void Keyboard_Init(void)
{
	UART1_Init();
}

/*
 *函数简介:键盘数据处理
 *参数说明:接收数据
 *返回类型:无
 *备注:无
 */
void Keyboard_DataProcess(uint8_t *Data)
{
}
