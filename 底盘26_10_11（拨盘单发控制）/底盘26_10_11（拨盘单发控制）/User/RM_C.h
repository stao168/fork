#include <stm32f4xx.h>
#ifndef __RM_C_H
#define __RM_C_H

/*==========延时==========*/
#include "Delay.h"						//延时

/*==========硬件驱动==========*/
#include "TIM.h"						//定时器
#include "LED.h"						//LED
#include "Buzzer.h"						//蜂鸣器
#include "Remote.h"						//遥控器
#include "M3508.h"						//M3508
#include "GM6020.h"						//GM6020
#include "M2006.h"						//M2006
#include "EPushRod.h"                   //电推杆

/*==========通讯协议==========*/
#include "UART.h"						//串口
#include "CAN.h"						//CAN

/*==========控制算法==========*/
#include "PID.h"						//PID

/*==========功能==========*/
#include "Warming.h"					//报警
#include "LinkCheck.h"                  //电机连接检测
#include "CloseLoopControl.h"           //闭环控制
#include "CToC.h"						//板间通讯

/*==========车体=========*/
#include "Dial.h"                    	//拨盘
#include "Mecanum.h"                    //麦轮
#include "RefereeSystem.h"				//裁判系统

/*==========用户=========*/
#include "Parameter.h"					//参数

#endif
