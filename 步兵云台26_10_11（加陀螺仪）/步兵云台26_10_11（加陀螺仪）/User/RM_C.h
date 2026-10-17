#ifndef __RM_C_H
#define __RM_C_H

#include <stm32f4xx.h>

/*==========延时==========*/
#include "Delay.h"						//延时

/*==========硬件驱动==========*/
#include "TIM.h"						//定时器
#include "LED.h"						//LED
#include "Buzzer.h"						//蜂鸣器
#include "Remote.h"						//遥控器
#include "BMI088.h"						//陀螺仪
#include "IST8310.h"					//磁力计
#include "Laser.h"						//激光

/*==========电机==========*/
#include "M3508.h"						//M3508
#include "GM6020.h"						//GM6020
#include "M2006.h"						//M3508
#include "DM_J4310.h"					//DM4310

/*==========通讯协议==========*/
#include "UART.h"						//串口
#include "MyI2C.h"						//软件I2C
#include "CAN.h"						//CAN

/*==========控制算法==========*/
#include "PID.h"						//PID

/*==========功能==========*/
#include "LinkCheck.h"					//CAN连接检测
#include "CloseLoopControl.h"			//闭环控制
#include "CToC.h"						//板间通讯
#include "IMUTemperatureControl.h"		//陀螺仪恒温控制
#include "AttitudeAlgorithms.h"			//姿态解算
#include "Warming.h"					//报警
#include "DebugPort.h"					//调试端口

/*==========车体=========*/
#include "Parameter.h"					//参数
#include "Gimbal.h"						//云台
#include "Visual.h"						//视觉
#include "RefereeSystem.h"				//裁判系统
#include "Keyboard.h"					//键盘

/*==========安全==========*/
#include "Lock.h"						//安全锁

#endif
