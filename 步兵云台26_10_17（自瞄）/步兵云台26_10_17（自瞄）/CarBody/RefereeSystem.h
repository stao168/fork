#ifndef __REFEREESYSTEM_H
#define __REFEREESYSTEM_H

extern uint8_t RefereeSystem_ShooterStatus;//裁判系统接收数据缓冲区

extern uint8_t RefereeSystem_ShooterOpenFlag,RefereeSystem_ShooterCloseFlag;//发射机构上电标志位,1-发射机构正在上电,0-其他
extern uint16_t RefereeSystem_ShooterOpenCounter,RefereeSystem_ShooterCloseCounter;//发射机构上电读秒等待设备启动
extern uint16_t RefereeSystem_Ref,RefereeSystem_Buffer;//底盘功率上限,底盘功率缓冲能量
extern uint8_t RefereeSystem_Status;//图传链路连接状态
extern uint8_t RefereeSystem_RobotID;//机器人ID
extern uint8_t RefereeSystem_GameStatus;

uint8_t RefereeSystem_GetCRC8CheckSum(uint8_t *Data,uint16_t Length,uint8_t Initial);//裁判系统CRC8查表计算
uint16_t RefereeSystem_GetCRC16CheckSum(uint8_t *Data,uint32_t Length,uint16_t Initial);//裁判系统CRC16查表计算
void RefereeSystem_Init(void);//裁判系统接收初始化

#endif
