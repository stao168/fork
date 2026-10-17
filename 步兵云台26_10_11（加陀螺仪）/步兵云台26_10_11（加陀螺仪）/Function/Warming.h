#ifndef __WARMING_H
#define __WARMING_H

#include <stm32f4xx.h>

/*====================	 报警列表	====================
	遥控器未连接	················1s里红灯连闪两下
	遥控器数据错误报警	············红灯常亮
	CAN总线设备连接异常	············2s内蜂鸣器以高音6响n下,n为CAN.c文件里ID列表的索引
	IST8310连接错误	················绿灯以1s为周期闪烁
	BMI088连接错误	················1s里绿灯连闪两下
	陀螺仪温度过高报警	············绿灯常亮
	电机报警状态	················电机静止
	FLASH清除警告	················白灯常亮
  ======================================================*/

#define DEVICES 4

extern uint8_t Can_ErrorDevice;
extern uint8_t Can_Error_Flag;
extern uint16_t Can_Test[DEVICES];

void Warming_Init(void);//报警初始化
void Warming_LEDClean(void);//报警LED清理
void Warming_BuzzerClean(void);//报警蜂鸣器清理
void Warming_RemoteNoCheck(void);//遥控器未连接报警
void Warming_RemoteDataERROR(void);//遥控器数据错误报警
void Warming_LinkError(void);//CAN总线设备连接异常报警
void Warming_IST8310LinkError(void);//IST8310连接错误
void Warming_BMI088LinkError(void);//BMI088连接错误
void Warming_IMUTemperatureTooHigh(void);//陀螺仪温度过高报警并停止加热
void Warming_MotorControl(void);//电机报警状态
void Warming_EraseFLASH(void);//FLASH清除警告
void CAN_Check(void);

#endif
