#ifndef __BUZZER_H
#define __BUZZER_H

typedef enum
{
	P=0,//空拍
	L1,L1_,L2,L2_,L3,L4,L4_,L5,L5_,L6,L6_,L7,//低音
	M1,M1_,M2,M2_,M3,M4,M4_,M5,M5_,M6,M6_,M7,//中音
	H1,H1_,H2,H2_,H3,H4,H4_,H5,H5_,H6,H6_,H7,//高音
}Buzzer_Tone;//蜂鸣器音调枚举

void Buzzer_Init(void);//蜂鸣器初始化
void Buzzer_Time(Buzzer_Tone Tone,uint16_t ms);//蜂鸣器发声
void Buzzer_ON(Buzzer_Tone Tone);//蜂鸣器打开
void Buzzer_OFF(void);//蜂鸣器关闭

#endif
