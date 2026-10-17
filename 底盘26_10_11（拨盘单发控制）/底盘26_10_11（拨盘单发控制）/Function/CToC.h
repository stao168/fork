#ifndef __CTOC_H
#define __CTOC_H

#include <stm32f4xx.h>

#define CToC_MasterID1	0x019//主机ID1

#define CToC_SlaveID1	0x149//从机ID1
#define CToC_SlaveID2	0x189//从机ID2
#define CToC_SlaveID3	0x109//从机ID3

#define CToC_SlaveID4  0x119 // 与云台发送端一致

extern float target_speed, actual_speed;
extern volatile uint8_t Shoot_OpenFlag;

void CToC_SlaveInit(void);//板间通讯从机初始化
uint8_t CToC_SlaveSendRefereeSystemData(void);//板间通讯从机发送裁判系统数据
void CToC_CANDataProcess(uint32_t ID,uint8_t *Data);//板间通讯数据处理

#endif
