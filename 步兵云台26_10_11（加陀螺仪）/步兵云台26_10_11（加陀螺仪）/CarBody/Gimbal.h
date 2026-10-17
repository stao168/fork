#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stm32f4xx.h>
#include "PID.h"

#define Gimbal_YawMotor										GM6020_1 //Yaw轴电机
#define Gimbal_PitchMotor									DM_J4310_2 //拨盘
#define FrictionWheel_Left									M3508_1 //左摩擦轮
#define FrictionWheel_Right									M3508_2 //右摩擦轮

extern PID_Typedef Gimbal_YawAnglePositionPID,Gimbal_YawAngleSpeedPID, Gimbal_Pitch4310AnglePositionPID, Gimbal_Pitch4310AngleSpeedPID;
extern PID_Typedef FrictionWheel_SpeedPID[];
extern uint8_t Gimbal_LinkFlag;
extern uint8_t Shoot_OpenCount, Shoot_OpenFlag;

void Gimbal_Init(void);//云台初始化
void Gimbal_Control(void);

#endif
