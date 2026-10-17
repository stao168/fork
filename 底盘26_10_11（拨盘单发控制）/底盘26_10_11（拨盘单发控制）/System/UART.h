#ifndef __UART_H
#define __UART_H

extern uint8_t UART1_RxData;//接收数据缓存区
extern uint8_t UART1_RxFlag;//接收完成标志位
extern uint8_t UART2_RxData;//接收数据缓存区
extern uint8_t UART2_RxFlag;//接收完成标志位

void UART1_SendInit(void);//UART1串口发送初始化
void UART1_ReceiveInit(void);//UART1串口接收初始化
void UART1_Init(void);//UART1串口初始化
void UART1_SendByte(uint8_t Byte);//UART1发送一个字节
void UART1_SendArray(uint8_t *Array,uint16_t Length);//UART1发送一个数组
void UART1_SendString(char *String);//UART1发送一个字符串
void UART1_SendNumber(uint32_t Number,uint8_t Length);//UART1以文本形式发送一个数字
void UART1_Printf(char *format,...);//UART1的printf函数
uint8_t UART1_GetRxFlag(void);//UART1获取接收完成标志位
void UART2_SendInit(void);//UART2串口发送初始化
void UART2_ReceiveInit(void);//UART2串口接收初始化
void UART2_Init(void);//UART2串口初始化
void UART2_SendByte(uint8_t Byte);//UART2发送一个字节
void UART2_SendArray(uint8_t *Array,uint16_t Length);//UART2发送一个数组
void UART2_SendString(char *String);//UART2发送一个字符串
void UART2_SendNumber(uint32_t Number,uint8_t Length);//UART2以文本形式发送一个数字
void UART2_Printf(char *format,...);//UART2的printf函数
uint8_t UART2_GetRxFlag(void);//UART2获取接收完成标志位

#endif
