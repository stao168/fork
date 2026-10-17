#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Gimbal.h"
#include "Remote.h"
#include "DM_J4310.h"
#include "GM6020.h"
#include "M3508.h"
#include "PID.h"
#include "AttitudeAlgorithms.h"
#include "Parameter.h"
#include "BMI088.h"

PID_Typedef Gimbal_YawAnglePositionPID, Gimbal_YawAngleSpeedPID;//Yaw轴GM6020电机PID
PID_Typedef Gimbal_Pitch4310AnglePositionPID, Gimbal_Pitch4310AngleSpeedPID;//Pitch轴DM4310电机PID
PID_Typedef FrictionWheel_SpeedPID[2];//摩擦轮M3508电机PID

#define Gimbal_PitchLeverSpeedMapRate 0.0015f

#define PitchDownLimt	-0.72f	//-0.681595266
#define PitchUpLimt		-1.45f	//-1.49566412

void Gimbal_Init(void)
{
	DM_J4310_Init();
	
	//DM_4310,自身编码器闭环
	PID_Init(&Gimbal_Pitch4310AnglePositionPID, 0, 0);
	PID_SetParameter(&Gimbal_Pitch4310AnglePositionPID, 45, 0, 0);
	PID_EnableOutputLimit(&Gimbal_Pitch4310AnglePositionPID, 12.566f);
	
	PID_Init(&Gimbal_Pitch4310AngleSpeedPID, 0, 0);
	PID_SetParameter(&Gimbal_Pitch4310AngleSpeedPID, 0.45, 0, 0);
	PID_EnableOutputLimit(&Gimbal_Pitch4310AngleSpeedPID, DM_J4310_TMAX);
	
	//YAW轴GM6020,陀螺仪闭环	
	PID_Init(&Gimbal_YawAnglePositionPID, 0, 0);
	PID_SetParameter(&Gimbal_YawAnglePositionPID, 0.55, 0, 0);
	PID_EnableOutputLimit(&Gimbal_YawAnglePositionPID, 15.71f);
	PID_Init(&Gimbal_YawAngleSpeedPID, 0, 0);
	PID_SetParameter(&Gimbal_YawAngleSpeedPID, 4000, 0, 0);
	PID_EnableOutputLimit(&Gimbal_YawAngleSpeedPID, 16384);
	
	//M3508,摩擦轮
	PID_Init(&FrictionWheel_SpeedPID[FrictionWheel_Left-0x201], 0, 5);
	PID_SetParameter(&FrictionWheel_SpeedPID[FrictionWheel_Left-0x201], 11.0, 0, 0);
	PID_EnableOutputLimit(&FrictionWheel_SpeedPID[FrictionWheel_Left-0x201], 16384);
	PID_Init(&FrictionWheel_SpeedPID[FrictionWheel_Right-0x201], 0, 5);
	PID_SetParameter(&FrictionWheel_SpeedPID[FrictionWheel_Right-0x201], 11.0, 0, 0);
	PID_EnableOutputLimit(&FrictionWheel_SpeedPID[FrictionWheel_Right-0x201], 16384);
}

void Gimbal_CleanPID(void)
{
	PID_Clean(&Gimbal_Pitch4310AnglePositionPID);
	PID_Clean(&Gimbal_Pitch4310AngleSpeedPID);
	PID_Clean(&Gimbal_YawAnglePositionPID);
	PID_Clean(&Gimbal_YawAngleSpeedPID);
	PID_Clean(&FrictionWheel_SpeedPID[FrictionWheel_Left-0x201]);
	PID_Clean(&FrictionWheel_SpeedPID[FrictionWheel_Right-0x201]);
}

uint8_t Gimbal_LinkFlag = 0;

void Gimbal_YawControl(void)
{
	if(Gimbal_LinkFlag == 0)
	{
		Gimbal_YawAnglePositionPID.Target=AttitudeAlgorithms_DegYaw;
		Gimbal_LinkFlag = 1;
	}
	if(Remote_RxData.Remote_L_RL>1075)//根据摇杆改变偏航
	{
		Gimbal_YawAnglePositionPID.Target-=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*Gimbal_YawPitchSpeedRate*0.0439453125f*((Remote_RxData.Remote_L_RL-1024)/784.0f);
	}
	else if(Remote_RxData.Remote_L_RL<975)
	{
		Gimbal_YawAnglePositionPID.Target+=Gimbal_LeverSpeedMapRate*Gimbal_YawPitchSpeedRate*Gimbal_YawPitchSpeedRate*0.0439453125f*((1024-Remote_RxData.Remote_L_RL)/784.0f);
	}
	PID_Calculate(&Gimbal_YawAnglePositionPID,AttitudeAlgorithms_DegYaw);
	Gimbal_YawAngleSpeedPID.Target=Gimbal_YawAnglePositionPID.OUT;
	
	PID_Calculate(&Gimbal_YawAngleSpeedPID, AttitudeAlgorithms_RaddYaw_E);
	GM6020_CAN2SetLIDCurrent(Gimbal_YawAngleSpeedPID.OUT, 0, 0, 0);
}

#define pitch_feedforward	-0.85f

void Gimbal_PitchControl(void)
{	
	if(Remote_RxData.Remote_L_UD < 1010) Gimbal_Pitch4310AnglePositionPID.Target += Gimbal_PitchLeverSpeedMapRate;
	else if(Remote_RxData.Remote_L_UD > 1040) Gimbal_Pitch4310AnglePositionPID.Target -= Gimbal_PitchLeverSpeedMapRate;
	if(Gimbal_Pitch4310AnglePositionPID.Target > PitchDownLimt) Gimbal_Pitch4310AnglePositionPID.Target = PitchDownLimt;
	else if(Gimbal_Pitch4310AnglePositionPID.Target < PitchUpLimt) Gimbal_Pitch4310AnglePositionPID.Target = PitchUpLimt;
		
	PID_Calculate(&Gimbal_Pitch4310AnglePositionPID, DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Angle);	
	Gimbal_Pitch4310AngleSpeedPID.Target = Gimbal_Pitch4310AnglePositionPID.OUT;
	PID_Calculate(&Gimbal_Pitch4310AngleSpeedPID, DM_J4310_MotorStatus[Gimbal_PitchMotor-0x01].Speed);
	
	float torque = Gimbal_Pitch4310AngleSpeedPID.OUT + pitch_feedforward;
	if (torque > DM_J4310_TMAX)  torque = DM_J4310_TMAX;
	if (torque < -DM_J4310_TMAX) torque = -DM_J4310_TMAX;
	
	DM_J4310_Set(DM_J4310_2, torque);
//	DM_J4310_Set(DM_J4310_2, 0);
}

#define FrictionWheel_SpeedLimit	7003 //转子转速对应线速度约22m/s
uint8_t Shoot_OpenCount = 0;
uint8_t Shoot_OpenFlag = 0;

void Gimbal_ShootControl(void)
{
	if(Remote_RxData.Remote_SWA == 0x400 || Remote_RxData.Remote_SWA == 0x70F)
	{
		FrictionWheel_SpeedPID[FrictionWheel_Left-0x201].Target = -5200;
		FrictionWheel_SpeedPID[FrictionWheel_Right-0x201].Target = 5200;
		Shoot_OpenCount ++;
		if(Shoot_OpenCount > 250)
		{
			Shoot_OpenCount = 250;
			Shoot_OpenFlag = 1;
		}
	}
	else
	{
		FrictionWheel_SpeedPID[FrictionWheel_Left-0x201].Target = 0;
		FrictionWheel_SpeedPID[FrictionWheel_Right-0x201].Target = 0;
		Shoot_OpenCount = 0;
		Shoot_OpenFlag = 0;
	}
	PID_Calculate(&FrictionWheel_SpeedPID[FrictionWheel_Left-0x201], M3508_MotorStatus[FrictionWheel_Left-0x201].RotorSpeed);
	PID_Calculate(&FrictionWheel_SpeedPID[FrictionWheel_Right-0x201], M3508_MotorStatus[FrictionWheel_Right-0x201].RotorSpeed);
	M3508_CAN1SetLIDCurrent(FrictionWheel_SpeedPID[FrictionWheel_Left-0x201].OUT, FrictionWheel_SpeedPID[FrictionWheel_Right-0x201].OUT, 0, 0);
}

void Gimbal_Control(void)
{
	Gimbal_YawControl();
	Gimbal_PitchControl();
	Gimbal_ShootControl();
}
