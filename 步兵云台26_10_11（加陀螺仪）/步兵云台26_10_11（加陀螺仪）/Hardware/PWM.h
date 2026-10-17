#ifndef __PWM_H
#define __PWM_H

typedef enum
{
	PWM1=0,PWM2,PWM3,PWM4,PWM5,PWM6,PWM7
}PWM_TypeDef;//PWM编号枚举

void PWM_Init(PWM_TypeDef PWMx,uint16_t Freq,float Duty);//PWM初始化
void PWM_SetFreq(PWM_TypeDef PWMx,uint16_t Freq);//PWM设置频率
void PWM_SetDuty(PWM_TypeDef PWMx,float Duty);//PWM设置占空比

#endif
