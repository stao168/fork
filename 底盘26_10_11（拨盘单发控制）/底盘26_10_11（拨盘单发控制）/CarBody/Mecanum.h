#ifndef __MECANUM_H
#define __MECANUM_H

#include <stm32f4xx.h>
#include "PID.h"

#define Chassis_LeftFront									M3508_1
#define Chassis_RightFront									M3508_2
#define Chassis_RightRear									M3508_3
#define Chassis_LeftRear									M3508_4
#define Gimbal_YawMotor										GM6020_1//Yaw轴电机


extern float Mecanum_Power;//底盘功率(软件计算值)
extern float Mecanum_YawTheta;//底盘云台相对偏航角度
extern PID_Typedef Mecanum_SpeedPID[];

void Mecanum_Init(void);//麦轮初始化
void Mecanum_CleanPID(void);//麦轮PID清理
void Mecanum_ControlSpeed(int16_t RightFrontSpeed, int16_t RightRearSpeed, int16_t LeftRearSpeed, int16_t LeftFrontSpeed);
void Mecanum_InverseMotionControl(float v_x,float v_y,float w);//麦轮逆运动解算
void Mecanum_MoveControl(void);
void Mecanum_PowerMoveControl(void);//麦轮功率控制
void Mecanum_Control(void);

#endif
