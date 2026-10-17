#ifndef __LED_H
#define __LED_H

void LED_Init(void);//三色单色LED初始化
void LED_BInit(void);//蓝色单色LED初始化
void LED_BON(void);//蓝灯点亮
void LED_BOFF(void);//蓝灯熄灭
void LED_BTurn(void);//蓝灯亮灭反转
void LED_GInit(void);//绿色单色LED初始化
void LED_GON(void);//绿灯点亮
void LED_GOFF(void);//绿灯熄灭
void LED_GTurn(void);//绿灯亮灭反转
void LED_RInit(void);//红色单色LED初始化
void LED_RON(void);//红灯点亮
void LED_ROFF(void);//红灯熄灭
void LED_RTurn(void);//红灯亮灭反转
void LED_MaxInit(void);//RGB混色LED初始化
void LED_SetColor(uint8_t R,uint8_t G,uint8_t B);//设置混色LED的RGB

#endif
