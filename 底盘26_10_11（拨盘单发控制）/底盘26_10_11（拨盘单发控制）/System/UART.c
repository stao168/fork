#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include <stdarg.h>

//重定向串口选择,选择哪个串口就把哪个取消注释
//#define UART1_Redirect//选择UART1重定向
#define UART2_Redirect//选择UART2重定向

uint8_t UART1_RxData;//接收数据缓存区
uint8_t UART1_RxFlag;//接收完成标志位

uint8_t UART2_RxData;//接收数据缓存区
uint8_t UART2_RxFlag;//接收完成标志位


/*
 *函数简介:串口专用指数函数
 *参数说明:底数x
 *参数说明:指数y
 *返回类型:x^y
 *备注:x,y均为整数
 */
uint32_t USART_Pow(uint32_t x,uint32_t y)
{
	uint32_t Result=1;
	while(y--)
		Result*=x;
	return Result;
}

/*
 *函数简介:UART1串口发送初始化
 *参数说明:无
 *返回类型:无
 *备注:UART1为USART6,默认Tx引脚PG14
 */
void UART1_SendInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);//初始化UART1-Tx(PG14)
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);//开启PG14的USART6复用模式
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//配置波特率115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Tx;//配置为发送模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//配置为无校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//配置字长8bit
	USART_Init(USART6,&USART_InitStructure);//初始化USART6
	
	USART_Cmd(USART6,ENABLE);//启动USART6
}

/*
 *函数简介:UART1串口接收初始化
 *参数说明:无
 *返回类型:无
 *备注:UART1为USART6,默认Rx引脚PG9
 */
void UART1_ReceiveInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);//开启时钟

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);//初始化UART1-Rx(PG9)
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);//开启PG9的USART6复用模式
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//配置波特率115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Rx;//配置为接收模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//配置为无校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//配置字长8bit
	USART_Init(USART6,&USART_InitStructure);//初始化USART6
	
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);//打通USART6到NVIC的串口接收中断通道
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级，2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART6_IRQn;//选择USART6中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化USART6的NVIC
	
	USART_Cmd(USART6,ENABLE);//启动USART6
}

/*
 *函数简介:UART1串口初始化
 *参数说明:无
 *返回类型:无
 *备注:UART1为USART6,默认Tx引脚PG14,Rx引脚PG9
 */
void UART1_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);//初始化UART1-Tx(PG14),UART1-Rx(PG9)
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);//开启PG14的USART6复用模式
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource9,GPIO_AF_USART6);//开启PG9的USART6复用模式
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//配置波特率115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;//配置为接发模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//配置为无校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//配置字长8bit
	USART_Init(USART6,&USART_InitStructure);//初始化USART6
		
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);//打通USART6到NVIC的串口接收中断通道

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级，2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART6_IRQn;//选择USART6中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化USART6的NVIC
	
	USART_Cmd(USART6,ENABLE);//启动USART6
}

/*
 *函数简介:UART1发送一个字节
 *参数说明:8bits数据
 *返回类型:无
 *备注:无
 */
void UART1_SendByte(uint8_t Byte)
{
	USART_SendData(USART6,Byte);
	while(USART_GetFlagStatus(USART6,USART_FLAG_TXE)==RESET);//等待发送完成
}

/*
 *函数简介:UART1发送一个数组
 *参数说明:8bits数组
 *参数说明:数组长度
 *返回类型:无
 *备注:无
 */
void UART1_SendArray(uint8_t *Array,uint16_t Length)
{
	for(uint16_t i=0;i<Length;i++)
		UART1_SendByte(Array[i]);//依次发送数组的每一位
}

/*
 *函数简介:UART1发送一个字符串
 *参数说明:字符串
 *返回类型:无
 *备注:无
 */
void UART1_SendString(char *String)
{
	for(uint8_t i=0;String[i]!='\0';i++)
		UART1_SendByte(String[i]);//依次发送字符串的每一位
}

/*
 *函数简介:UART1以文本形式发送一个数字
 *参数说明:整型数字
 *参数说明:数字长度(Length>=Number位数)
 *返回类型:无
 *备注:无
 */
void UART1_SendNumber(uint32_t Number,uint8_t Length)
{
	for(uint8_t i=0;i<Length;i++)
		UART1_SendByte(Number/USART_Pow(10,Length-i-1)%10+'0');//以文本形式发送数字每一位
}

#ifdef UART1_Redirect//选择UART1重定向
	/*
	 *函数简介:重写fputc函数(重定向printf，即移植printf)
	 *参数说明:无
	 *返回类型:无
	 *备注:看不懂
	 *备注:默认重定向UART1(即USART6),若想修改重定向串口,请更改最上方宏定义的注释
	 */
	int fputc(int ch,FILE *f)
	{
		USART6->DR=ch;
		while(!(USART6->SR&(1<<7))){}
		return ch;
	}

	/*
	 *函数简介:重写fgetc函数(重定向scanf，即移植scanf)
	 *参数说明:无
	 *返回类型:无
	 *备注:看不懂
	 *备注:默认重定向UART1(即USART6),若想修改重定向串口,请更改最上方宏定义的注释
	 */
	int fgetc(FILE *stream)
	{
		while(!(USART6->SR&1<<5)){}
		return USART6->DR;
	}
#endif

/*
 *函数简介:UART1的printf函数
 *参数说明:无
 *返回类型:无
 *备注:等价printf函数,支持多串口
 *备注:看不懂
 */
void UART1_Printf(char *format,...)
{
	char Strings[100];
	va_list arg;//定义参数列表变量arg
	va_start(arg,format);//从format位置接收参数，放在arg里
	vsprintf(Strings,format,arg);//将格式化字符串打印到Strings
	va_end(arg);//释放参数表arg
	UART1_SendString(Strings);//发送字符串Strings
}

/*
 *函数简介:UART1获取接收完成标志位
 *参数说明:无
 *返回类型:1-接收完成,0-接收未完成
 *备注:接收完成会软件清零标志位
 */
uint8_t UART1_GetRxFlag(void)
{
	if(UART1_RxFlag==1)
	{
		UART1_RxFlag=0;//复位
		return 1;
	}
	return 0;
}

/*
 *函数简介:UART2串口发送初始化
 *参数说明:无
 *返回类型:无
 *备注:UART2为USART1,默认Tx引脚PA9
 */
void UART2_SendInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化UART2-Tx(PA9)
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);//开启PA9的USART1复用模式
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//配置波特率115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Tx;//配置为发送模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//配置为无校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//配置字长8bit
	USART_Init(USART1,&USART_InitStructure);//初始化USART1
	
	USART_Cmd(USART1,ENABLE);//启动USART1
}

/*
 *函数简介:UART2串口接收初始化
 *参数说明:无
 *返回类型:无
 *备注:UART2为USART1,默认Rx引脚PB7
 */
void UART2_ReceiveInit(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//开启时钟

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化UART2-Rx(PB7)
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);//开启PB7的USART1复用模式
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//配置波特率115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Rx;//配置为接收模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//配置为无校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//配置字长8bit
	USART_Init(USART1,&USART_InitStructure);//初始化USART1
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打通USART1到NVIC的串口接收中断通道
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级，2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//选择USART1中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化USART1的NVIC
	
	USART_Cmd(USART1,ENABLE);//启动USART1
}

/*
 *函数简介:UART2串口初始化
 *参数说明:无
 *返回类型:无
 *备注:UART2为USART1,默认Tx引脚PA9,Rx引脚PB7
 */
void UART2_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//初始化UART2-Tx(PA9)
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//初始化UART2-Rx(PB7)
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);//开启PA9的USART1复用模式
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);//开启PB7的USART1复用模式
	
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=115200;//配置波特率115200
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Tx | USART_Mode_Rx;//配置为接发模式
	USART_InitStructure.USART_Parity=USART_Parity_No;//配置为无校验位
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;//配置字长8bit
	USART_Init(USART1,&USART_InitStructure);//初始化USART1
		
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);//打通USART1到NVIC的串口接收中断通道
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级，2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;//选择USART1中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化USART1的NVIC
	
	USART_Cmd(USART1,ENABLE);//启动USART1
}

/*
 *函数简介:UART2发送一个字节
 *参数说明:8bits数据
 *返回类型:无
 *备注:无
 */
void UART2_SendByte(uint8_t Byte)
{
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//等待发送完成
}

/*
 *函数简介:UART2发送一个数组
 *参数说明:8bits数组
 *参数说明:数组长度
 *返回类型:无
 *备注:无
 */
void UART2_SendArray(uint8_t *Array,uint16_t Length)
{
	for(uint16_t i=0;i<Length;i++)
		UART2_SendByte(Array[i]);//依次发送数组的每一位
}

/*
 *函数简介:UART2发送一个字符串
 *参数说明:字符串
 *返回类型:无
 *备注:无
 */
void UART2_SendString(char *String)
{
	for(uint8_t i=0;String[i]!='\0';i++)
		UART2_SendByte(String[i]);//依次发送字符串的每一位
}

/*
 *函数简介:UART2以文本形式发送一个数字
 *参数说明:整型数字
 *参数说明:数字长度(Length>=Number位数)
 *返回类型:无
 *备注:无
 */
void UART2_SendNumber(uint32_t Number,uint8_t Length)
{
	for(uint8_t i=0;i<Length;i++)
		UART2_SendByte(Number/USART_Pow(10,Length-i-1)%10+'0');//以文本形式发送数字每一位
}

#ifdef UART2_Redirect//选择UART2重定向
	/*
	 *函数简介:重写fputc函数(重定向printf，即移植printf)
	 *参数说明:无
	 *返回类型:无
	 *备注:看不懂
	 *备注:默认重定向UART1(即USART6),若想修改重定向串口,请更改最上方宏定义的注释
	 */
	int fputc(int ch,FILE *f)
	{
		USART1->DR=ch;
		while(!(USART1->SR&(1<<7))){}
		return ch;
	}

	/*
	 *函数简介:重写fgetc函数(重定向scanf，即移植scanf)
	 *参数说明:无
	 *返回类型:无
	 *备注:看不懂
	 *备注:默认重定向UART1(即USART6),若想修改重定向串口,请更改最上方宏定义的注释
	 */
	int fgetc(FILE *stream)
	{
		while(!(USART1->SR&1<<5)){}
		return USART1->DR;
	}
#endif

/*
 *函数简介:UART2的printf函数
 *参数说明:无
 *返回类型:无
 *备注:等价printf函数,支持多串口
 *备注:看不懂
 */
void UART2_Printf(char *format,...)
{
	char Strings[100];
	va_list arg;//定义参数列表变量arg
	va_start(arg,format);//从format位置接收参数，放在arg里
	vsprintf(Strings,format,arg);//将格式化字符串打印到Strings
	va_end(arg);//释放参数表arg
	UART2_SendString(Strings);//发送字符串Strings
}

/*
 *函数简介:UART2获取接收完成标志位
 *参数说明:无
 *返回类型:1-接收完成,0-接收未完成
 *备注:接收完成会软件清零标志位
 */
uint8_t UART2_GetRxFlag(void)
{
	if(UART2_RxFlag==1)
	{
		UART2_RxFlag=0;//复位
		return 1;
	}
	return 0;
}

/*
 *函数简介:UART1普通串口中断函数
 *参数说明:无
 *返回类型:无
 *备注:接收完成置UART1_RxFlag为1,需软件清0
 */
/**************************************************
void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)//查询接收中断标志位
	{
		UART1_RxData=USART_ReceiveData(USART6);//将数据存入缓存区
		//函数体
		UART1_RxFlag=1;//置接收完成标志位
	}
	
	USART_ClearITPendingBit(USART6,USART_IT_RXNE);//清除接收中断标志位
}
**************************************************/

/*
 *函数简介:UART2普通串口中断函数
 *参数说明:无
 *返回类型:无
 *备注:接收完成置UART2_RxFlag为1,需软件清0
 */
/**************************************************
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)//查询接收中断标志位
	{
		UART2_RxData=USART_ReceiveData(USART1);//将数据存入缓存区
		//函数体
		UART2_RxFlag=1;//置接收完成标志位
	}
	
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);//清除接收中断标志位
}
**************************************************/
