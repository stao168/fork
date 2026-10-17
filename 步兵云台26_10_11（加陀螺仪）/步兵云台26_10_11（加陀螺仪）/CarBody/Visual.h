#ifndef __VISUAL_H
#define __VISUAL_H

#include <stm32f4xx.h>

// 主控 -> miniPC 结构体
typedef struct __attribute__((packed)) SendPacket
{
    uint8_t header;// = 0xAA
    uint8_t mode;
    float   q[4]; // 四元数 wxyz 顺序
    uint8_t tail;// = 0X5A
} SendPacket;


// miniPC -> 主控 结构体
typedef struct __attribute__((packed)) ReceivePacket
{
    uint8_t header;// = 0XBB
    float   target_yaw;
    float   target_pitch;
    float   distance;
    uint8_t fire_advice;
    float   vx;
    float   vy;
    uint8_t tail;// = 0X5B
}ReceivePacket;

extern SendPacket Visual_TxData;
extern ReceivePacket Visual_RxData;

extern float Visual_Yaw,Visual_Pitch,Visual_Fire;//视觉数据偏航角,视觉数据俯仰角
extern uint8_t Visual_ReceiveFlag;//视觉数据接收完成标志位

void Visual_Init(void);//视觉初始化
void Visual_SendData(void);

#endif
