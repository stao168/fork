#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "USB.h"

void DebugPort_Init(void)
{
	USB_Init();
}

void DebugPort_SendArray(uint8_t *Array,uint16_t Length)
{
	USB_Send(Array,Length);
}

void DebugPort_SendString(char *String)
{
	USB_Send((uint8_t *)String,strlen(String));
}

void DebugPort_Printf(char *format,...)
{
	char Strings[100];
	va_list arg;//定义参数列表变量arg
	va_start(arg,format);//从format位置接收参数，放在arg里
	vsprintf(Strings,format,arg);//将格式化字符串打印到Strings
	va_end(arg);//释放参数表arg
	DebugPort_SendString(Strings);//发送字符串Strings
}
