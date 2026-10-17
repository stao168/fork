#ifndef __VISUAL_H
#define __VISUAL_H

#include <stm32f4xx.h>

/* 电控 -> 视觉：20 字节 */
typedef struct __attribute__((packed)) SendPacket
{
    uint8_t header;       /* 固定 0x5A，偏移 0 */
    uint8_t mode;         /* 偏移 1 */
    float   q[4];         /* w、x、y、z，偏移 2～17 */
    uint8_t crc_low;      /* 偏移 18 */
    uint8_t crc_high;     /* 偏移 19 */
} SendPacket;


/* 视觉 -> 电控：12 字节 */
typedef struct __attribute__((packed)) ReceivePacket
{
    uint8_t header;       /* 固定 0xA5，偏移 0 */
    float   target_yaw;   /* 绝对目标角，rad，偏移 1～4 */
    float   target_pitch; /* 绝对目标角，rad，偏移 5～8 */
    uint8_t fire_advice;  /* 0 禁止开火，1 允许开火，偏移 9 */
    uint8_t crc_low;      /* 偏移 10 */
    uint8_t crc_high;     /* 偏移 11 */
} ReceivePacket;

/* 编译时检查包长度；长度不正确会报错 */
typedef char Visual_SendPacket_SizeCheck[(sizeof(SendPacket) == 20U) ? 1 : -1];
typedef char Visual_ReceivePacket_SizeCheck[(sizeof(ReceivePacket) == 12U) ? 1 : -1];

extern SendPacket Visual_TxData;
extern ReceivePacket Visual_RxData;

extern float Visual_Yaw,Visual_Pitch;//视觉数据偏航角,视觉数据俯仰角
extern uint8_t Visual_ReceiveFlag,Visual_Fire;//视觉数据接收完成标志位

void Visual_Init(void);//视觉初始化
void Visual_SendData(void);

#endif
