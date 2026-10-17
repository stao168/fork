#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include <math.h>
#include "M3508.h"
#include "PID.h"
#include "Parameter.h"
#include "Mecanum.h"
#include "Remote.h"
#include "GM6020.h"
#include "RefereeSystem.h"

#define Mecanum_PowerControlSpeedNormalizationValue			3.0f//功率控制归一化速度标准值,范围[0,3]
#define Mecanum_PowerControlGainCoefficientInitialValue		0.5f//功率控制增益系数初始值
#define Mecanum_PowerControl_T								5.0f//功率控制周期(T=Mecanum_PowerControl_T*2ms)
#define Mecanum_PowerControl_UseBuffer						30.0f//功率控制消耗的缓冲能量
#define Mecanum_PowerControl_PowerMax						4.0f//功率控制功率上限与裁判系统功率上限比值上限
#define Mecanum_PowerControl_UltraCAPPower					100.0f//使用超电增加功率

PID_Typedef Mecanum_SpeedPID[4];//底盘四个电机的转速PID
PID_Typedef Mecanum_TrackPID;//底盘跟随PID
float Mecanum_YawTheta;//底盘云台相对偏航角度
uint8_t Mecanum_GyroScopeFlag;//底盘小陀螺标志位
float Mecanum_Power;//底盘功率(软件计算值)
float Mecanum_Current,Mecanum_PowerLimit,Mecanum_CurrentLimit=16384*4.0f;//底盘电流总和,功率控制功率上限,底盘电流限幅
	
/*
 *函数简介:麦轮初始化
 *参数说明:无
 *返回类型:无
 *备注:即四个转速PID的初始化
 *备注:四个电机的ID参照上方的宏定义,最好使用M3508电机的ID1~4,否则下方电机控制的代码需要修改
 */
void Mecanum_Init(void)
{
	PID_Init(&Mecanum_SpeedPID[Chassis_RightFront - 0x201], 0, 5);//右前轮
	PID_SetParameter(&Mecanum_SpeedPID[Chassis_RightFront - 0x201], 11.8, 0, 0);
	PID_EnableOutputLimit(&Mecanum_SpeedPID[Chassis_RightFront - 0x201], 15000);      
	
	PID_Init(&Mecanum_SpeedPID[Chassis_RightRear - 0x201], 0, 5);//右后轮
	PID_SetParameter(&Mecanum_SpeedPID[Chassis_RightRear - 0x201], 11.8, 0, 0);
	PID_EnableOutputLimit(&Mecanum_SpeedPID[Chassis_RightRear - 0x201], 15000);
	
	PID_Init(&Mecanum_SpeedPID[Chassis_LeftRear - 0x201], 0, 5);//左后轮
	PID_SetParameter(&Mecanum_SpeedPID[Chassis_LeftRear - 0x201], 11.8, 0, 0);
	PID_EnableOutputLimit(&Mecanum_SpeedPID[Chassis_LeftRear - 0x201], 15000);
	
	PID_Init(&Mecanum_SpeedPID[Chassis_LeftFront - 0x201], 0, 5);//左前轮
	PID_SetParameter(&Mecanum_SpeedPID[Chassis_LeftFront - 0x201], 11.8, 0, 0);
	PID_EnableOutputLimit(&Mecanum_SpeedPID[Chassis_LeftFront - 0x201], 15000);
	
	PID_Init(&Mecanum_TrackPID, Yaw_GM6020PositionValue, 5);//底盘跟随
	PID_SetParameter(&Mecanum_TrackPID, 0.004, 0, 0);
	PID_EnableOutputLimit(&Mecanum_TrackPID, 2);
}

/*
 *函数简介:麦轮PID清理
 *参数说明:无
 *返回类型:无
 *备注:清理四个转速位置式PID
 */
void Mecanum_CleanPID(void)
{
	PID_Clean(&Mecanum_SpeedPID[Chassis_RightFront - 0x201]);//右前轮
	PID_Clean(&Mecanum_SpeedPID[Chassis_RightRear - 0x201]);//右后轮
	PID_Clean(&Mecanum_SpeedPID[Chassis_LeftRear - 0x201]);//左后轮
	PID_Clean(&Mecanum_SpeedPID[Chassis_LeftFront - 0x201]);//左前轮
//	PID_Clean(&Mecanum_TrackPID);//底盘跟随
}

/*
 *函数简介:麦轮速度控制
 *返回类型:无
 *备注:单独控制四个轮子的速度
 */
void Mecanum_ControlSpeed(int16_t RightFrontSpeed, int16_t RightRearSpeed, int16_t LeftRearSpeed, int16_t LeftFrontSpeed)
{
	//更改期望
	Mecanum_SpeedPID[Chassis_LeftFront - 0x201].Target = LeftFrontSpeed;//左前轮
	Mecanum_SpeedPID[Chassis_RightFront - 0x201].Target = RightFrontSpeed;//右前轮
	Mecanum_SpeedPID[Chassis_RightRear - 0x201].Target = RightRearSpeed;//右后轮
	Mecanum_SpeedPID[Chassis_LeftRear - 0x201].Target = LeftRearSpeed;//左后轮
	
	//PID计算
	PID_Calculate(&Mecanum_SpeedPID[Chassis_LeftFront - 0x201], M3508_MotorStatus[Chassis_LeftFront-0x201].RotorSpeed);//左前轮
	PID_Calculate(&Mecanum_SpeedPID[Chassis_RightFront - 0x201], M3508_MotorStatus[Chassis_RightFront-0x201].RotorSpeed);//右前轮
	PID_Calculate(&Mecanum_SpeedPID[Chassis_RightRear - 0x201], M3508_MotorStatus[Chassis_RightRear-0x201].RotorSpeed);//右后轮
	PID_Calculate(&Mecanum_SpeedPID[Chassis_LeftRear - 0x201], M3508_MotorStatus[Chassis_LeftRear-0x201].RotorSpeed);//左后轮

	Mecanum_Current = 0;
	for(uint8_t i = 0; i < 4; i ++)
		Mecanum_Current += fabs(Mecanum_SpeedPID[i].OUT);
	
	if(Mecanum_Current > Mecanum_CurrentLimit)
	{
		Mecanum_SpeedPID[Chassis_RightFront - 0x201].OUT *= (Mecanum_CurrentLimit / Mecanum_Current);
		Mecanum_SpeedPID[Chassis_RightRear - 0x201].OUT *= (Mecanum_CurrentLimit / Mecanum_Current);
		Mecanum_SpeedPID[Chassis_LeftRear - 0x201].OUT *= (Mecanum_CurrentLimit / Mecanum_Current);
		Mecanum_SpeedPID[Chassis_LeftFront - 0x201].OUT *= (Mecanum_CurrentLimit / Mecanum_Current);
	}
	
	M3508_CAN1SetLIDCurrent(Mecanum_SpeedPID[Chassis_LeftFront - 0x201].OUT, Mecanum_SpeedPID[Chassis_RightFront - 0x201].OUT, Mecanum_SpeedPID[Chassis_RightRear - 0x201].OUT, Mecanum_SpeedPID[Chassis_LeftRear - 0x201].OUT);//M3508控制
}

/*
 *函数简介:麦轮逆运动解算
 *参数说明:x轴速度,单位m/s(以前为正)
 *参数说明:y轴速度,单位m/s(以左为正)
 *参数说明:z轴转速,单位rad/s(以逆时针为正)
 *返回类型:无
 *备注:速度转速转换系数:
 *	   w'=v/R (rad/s)=v/(2Π×R) (r/s)=60×v/(2Π×R) (r/min)=1/19 w
 *	   ⇒ w=19×60×v/(2Π×R/100)=19×60×100×v/(2Π×R)=18143.663512×v/R,R单位cm
 *	   * 0.70710678f   // 1/sqrt(2)，45°全向轮投影系数
 *备注:麦轮半径参数在上方宏定义Mecanum_WheelRadius修改,默认7cm,转换参数2591.95
 *备注:底盘中心到轮子中心的距离由上方宏定义x轴分量Mecanum_rx和y轴分量Mecanum_ry决定
 */
void Mecanum_InverseMotionControl(float v_x, float v_y, float w)
{
	//逆运动解算
	int16_t RightFrontSpeed = (int16_t)((v_x+v_y+w * (Mecanum_rx+Mecanum_ry) / 100.0f) * 0.70710678f /Mecanum_WheelRadius * (M3508_ReductionRatio * 60.0f * 100.0f / (2.0f * PI)));//右前轮
	int16_t RightRearSpeed = (int16_t)((v_x-v_y+w * (Mecanum_rx+Mecanum_ry) / 100.0f) * 0.70710678f /Mecanum_WheelRadius * (M3508_ReductionRatio * 60.0f * 100.0f / (2.0f * PI)));//右后轮
	int16_t LeftRearSpeed = (int16_t)((-v_x-v_y+w * (Mecanum_rx+Mecanum_ry) / 100.0f) * 0.70710678f /Mecanum_WheelRadius * (M3508_ReductionRatio * 60.0f * 100.0f / (2.0f * PI)));//左后轮
	int16_t LeftFrontSpeed = (int16_t)((-v_x+v_y+w * (Mecanum_rx+Mecanum_ry) / 100.0f) * 0.70710678f /Mecanum_WheelRadius * (M3508_ReductionRatio * 60.0f * 100.0f / (2.0f * PI)));//左前轮
	
	Mecanum_ControlSpeed(RightFrontSpeed, RightRearSpeed, LeftRearSpeed, LeftFrontSpeed);//M3508速度控制
}

#define RC_JOYSTICK_DEADBAND 5.0f   // 摇杆死区，消除机械回中误差
#define RC_JOYSTICK_MAX      784.0f  // 遥控器单轴最大偏差
#define MAX_W  4.0f      // 最大角速度 rad/s
float vx_G = 0.0f, vy_G = 0.0f, w = 0.0f, vx_C = 0.0f, vy_C = 0.0f;
int16_t Raw_Theta;
uint8_t Mecanum_GyroScopeCloseFlag=0;

/*
 *函数简介:只做单独底盘简单控制
 *参数说明:无
 *返回类型:无
 *备注:麦轮半径参数在上方宏定义Mecanum_WheelRadius修改,默认7cm,转换参数2591.95
 *备注:底盘中心到轮子中心的距离由上方宏定义x轴分量Mecanum_rx和y轴分量Mecanum_ry决定
 */
void Mecanum_MoveControl(void)
{
	float vx_raw, vy_raw;
	
	/* 获取遥控器 */
	vx_raw = -(1024 - Remote_RxData.Remote_R_UD);
	vy_raw =  (1024 - Remote_RxData.Remote_R_RL);
	
	/* 摇杆死区 */
	if(fabsf(vx_raw) < RC_JOYSTICK_DEADBAND) vx_raw = 0.0f;
	if(fabsf(vy_raw) < RC_JOYSTICK_DEADBAND) vy_raw = 0.0f;
	
	/* 摇杆量线性映射到实际速度 */
	vx_G = vx_raw / RC_JOYSTICK_MAX * Mecanum_PowerControlSpeedNormalizationValue;
	vy_G = vy_raw / RC_JOYSTICK_MAX * Mecanum_PowerControlSpeedNormalizationValue;
	
	float sigma = sqrtf(vx_G * vx_G + vy_G * vy_G);
	if(sigma > Mecanum_PowerControlSpeedNormalizationValue)
	{
		float scale = Mecanum_PowerControlSpeedNormalizationValue / sigma;
		vx_G *= scale;
		vy_G *= scale;
	}
	
	//获取底盘云台相对角度原始数据(底盘相对云台逆时针0-2Π用于坐标系变换)
	Raw_Theta=GM6020_MotorStatus[Gimbal_YawMotor-0x205].Angle-Yaw_GM6020PositionValue;
	if(Raw_Theta<0)Raw_Theta+=8192;
	
	//Mecanum_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Mecanum_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对弧度制角度
	
	//获取x、y速度(底盘Chassis坐标系)
	vx_C=vx_G*cosf(Mecanum_YawTheta)-vy_G*sinf(Mecanum_YawTheta);
	vy_C=vx_G*sinf(Mecanum_YawTheta)+vy_G*cosf(Mecanum_YawTheta);//根据底盘云台相对角度修正xy轴速度
	
	/*========== 小陀螺或底盘跟随 ==========*/
	if(Yaw_GM6020PositionValue < 4096)//正向小陀螺为逆时针
	{
		if(Remote_RxData.Remote_SWB == 0x70F || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[Gimbal_YawMotor-0x205].Position < Yaw_GM6020PositionValue+500))//小陀螺模式
		{
			w = Mecanum_GyroScopeAngularVelocity;
			Mecanum_GyroScopeFlag = 1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag = 1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag == 1 && ((GM6020_MotorStatus[Gimbal_YawMotor-0x205].Angle > Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[Gimbal_YawMotor-0x205].Angle < Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[Gimbal_YawMotor-0x205].Speed > 2 || GM6020_MotorStatus[Gimbal_YawMotor-0x205].Speed < -2)))//小陀螺关闭状态
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID, 2);
			PID_Calculate(&Mecanum_TrackPID, GM6020_MotorStatus[Gimbal_YawMotor-0x205].Position);//底盘跟云台
			w = -Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag = 0;//小陀螺未处于待关闭状态
		}
//		else if(Remote_RxData.Remote_SWC == 0xF0)//反向小陀螺模式(仅检录使用)
//			w = -Mecanum_GyroScopeAngularVelocity;
		else
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID, 5);
			Mecanum_GyroScopeFlag = 0;
			PID_Calculate(&Mecanum_TrackPID, GM6020_MotorStatus[Gimbal_YawMotor-0x205].Position);//底盘跟云台
			w = -Mecanum_TrackPID.OUT;
		}
	}
	else if(Yaw_GM6020PositionValue>=4096)//正向小陀螺为顺时针
	{
		if(Remote_RxData.Remote_SWB == 0x70F || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[Gimbal_YawMotor-0x205].Position>Yaw_GM6020PositionValue-500))//小陀螺模式
		{
			w=-Mecanum_GyroScopeAngularVelocity;
			Mecanum_GyroScopeFlag=1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==1 && ((GM6020_MotorStatus[Gimbal_YawMotor-0x205].Angle>Yaw_GM6020PositionValue+5 || GM6020_MotorStatus[Gimbal_YawMotor-0x205].Angle<Yaw_GM6020PositionValue-5) || (GM6020_MotorStatus[Gimbal_YawMotor-0x205].Speed>1||GM6020_MotorStatus[Gimbal_YawMotor-0x205].Speed<-1)))//小陀螺关闭状态
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID,2);
			PID_Calculate(&Mecanum_TrackPID,GM6020_MotorStatus[Gimbal_YawMotor-0x205].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
//		else if(Remote_RxData.Remote_SWC==0xF0)//反向小陀螺模式(仅检录使用)
//			w=Mecanum_GyroScopeAngularVelocity;
		else
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID,5);
			Mecanum_GyroScopeFlag=0;
			PID_Calculate(&Mecanum_TrackPID,GM6020_MotorStatus[Gimbal_YawMotor-0x205].Position);//底盘跟云台
			w=-Mecanum_TrackPID.OUT;
		}
	}
	
	/* 四轮逆解 */
	Mecanum_InverseMotionControl(vx_C, vy_C, w);
}

float vx_G_raw,vy_G_raw,V_w;


float GainCoefficient = 1.0f;
uint8_t Count = 0;
float PowerSum = 0;
    
float Mecanum_TotalPower = 0;
float Mecanum_PowerLimit = 0;

float PowerRef,Scale,Power_avg,BufferCurrent;

void Mecanum_PowerMoveControl(void)
{
	//获取x、y速度(云台Gimbal坐标系)
	vx_G_raw=-(1024-Remote_RxData.Remote_R_UD);
	vy_G_raw=(1024-Remote_RxData.Remote_R_RL);
	if (fabs(vx_G_raw) < RC_JOYSTICK_DEADBAND) vx_G_raw = 0.0f;
    if (fabs(vy_G_raw) < RC_JOYSTICK_DEADBAND) vy_G_raw = 0.0f;
	
//	vx_G = (vx_G_raw / RC_JOYSTICK_MAX) * Mecanum_PowerControlSpeedNormalizationValue;
//	vy_G = (vy_G_raw / RC_JOYSTICK_MAX) * Mecanum_PowerControlSpeedNormalizationValue;
	
//	float sigma=sqrtf(vx_G*vx_G+vy_G*vy_G);//获取xy轴速度归一化系数
	
	float sigma=sqrtf(vx_G_raw*vx_G_raw+vy_G_raw*vy_G_raw);//获取xy轴速度归一化系数
	
	if(sigma!=0)//x,y轴速度归一化(正交合成速度不变为标准值)
	{
		vx_G=vx_G_raw/sigma*Mecanum_PowerControlSpeedNormalizationValue;
		vy_G=vy_G_raw/sigma*Mecanum_PowerControlSpeedNormalizationValue;
	}
	else {vx_G=0;vy_G=0;}
	
	//获取底盘云台相对角度原始数据(底盘相对云台逆时针0-2Π用于坐标系变换)
	Raw_Theta=GM6020_MotorStatus[GM6020_5-0x205].Angle-Yaw_GM6020PositionValue;
	if(Raw_Theta<0)Raw_Theta+=8192;
	
	//Mecanum_YawTheta=Raw_Theta/8192.0f*2.0f*3.141592653589793238462643383279f;
	Mecanum_YawTheta=Raw_Theta*0.000766990393942820614859043794746f;//获取底盘云台相对弧度制角度
	
	//获取x、y速度(底盘Chassis坐标系)
	vx_C=vx_G*cosf(Mecanum_YawTheta)-vy_G*sinf(Mecanum_YawTheta);
	vy_C=vx_G*sinf(Mecanum_YawTheta)+vy_G*cosf(Mecanum_YawTheta);//根据底盘云台相对角度修正xy轴速度
	
	/*========== 小陀螺或底盘跟随 ==========*/
	if(Yaw_GM6020PositionValue<4096 && Remote_RxData.Remote_SWA != 0xF0)//正向小陀螺为逆时针
	{
		if(Remote_RxData.Remote_SWB == 0x70F || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[GM6020_5-0x205].Position<Yaw_GM6020PositionValue+500))//小陀螺模式
		{
			V_w=Mecanum_GyroScopeAngularVelocity;
			Mecanum_GyroScopeFlag=1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==1 && ((GM6020_MotorStatus[GM6020_5-0x205].Angle>Yaw_GM6020PositionValue+100 || GM6020_MotorStatus[GM6020_5-0x205].Angle<Yaw_GM6020PositionValue-100) || (GM6020_MotorStatus[GM6020_5-0x205].Speed>2||GM6020_MotorStatus[GM6020_5-0x205].Speed<-2)))//小陀螺关闭状态
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID, 2);
			PID_Calculate(&Mecanum_TrackPID,GM6020_MotorStatus[GM6020_5-0x205].Position);//底盘跟云台
			V_w=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_SWC==0xF0)//反向小陀螺模式(仅检录使用)
			V_w=-Mecanum_GyroScopeAngularVelocity;
		else
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID,5);
			Mecanum_GyroScopeFlag=0;
			PID_Calculate(&Mecanum_TrackPID,GM6020_MotorStatus[GM6020_5-0x205].Position);//底盘跟云台
			V_w=-Mecanum_TrackPID.OUT;
		}
	}
	else if(Yaw_GM6020PositionValue>=4096 && Remote_RxData.Remote_SWA != 0xF0)//正向小陀螺为顺时针
	{
		if(Remote_RxData.Remote_SWB == 0x70F || (Mecanum_GyroScopeFlag==1 && Mecanum_GyroScopeCloseFlag==1 && GM6020_MotorStatus[GM6020_5-0x205].Position>Yaw_GM6020PositionValue-500))//小陀螺模式
		{
			V_w=-Mecanum_GyroScopeAngularVelocity;
			Mecanum_GyroScopeFlag=1;//处于小陀螺状态
			Mecanum_GyroScopeCloseFlag=1;//小陀螺处于待关闭状态
		}
		else if(Mecanum_GyroScopeFlag==1 && ((GM6020_MotorStatus[GM6020_5-0x205].Angle>Yaw_GM6020PositionValue+5 || GM6020_MotorStatus[GM6020_5-0x205].Angle<Yaw_GM6020PositionValue-5) || (GM6020_MotorStatus[GM6020_5-0x205].Speed>1||GM6020_MotorStatus[GM6020_5-0x205].Speed<-1)))//小陀螺关闭状态
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID,2);
			PID_Calculate(&Mecanum_TrackPID,GM6020_MotorStatus[GM6020_5-0x205].Position);//底盘跟云台
			V_w=-Mecanum_TrackPID.OUT;
			Mecanum_GyroScopeCloseFlag=0;//小陀螺未处于待关闭状态
		}
		else if(Remote_RxData.Remote_SWC==0xF0)//反向小陀螺模式(仅检录使用)
			V_w=Mecanum_GyroScopeAngularVelocity;
		else
		{
			PID_EnableOutputLimit(&Mecanum_TrackPID,5);
			Mecanum_GyroScopeFlag=0;
			PID_Calculate(&Mecanum_TrackPID,GM6020_MotorStatus[GM6020_5-0x205].Position);//底盘跟云台
			V_w=-Mecanum_TrackPID.OUT;
		}
	}
	else V_w = 0;
	
	/*========== 功率控制 ==========*/
//    static float GainCoefficient = 1.0f;
//    static uint8_t Count = 0;
//    static float PowerSum = 0;
//    
//    float Mecanum_TotalPower = 0;
//    float Mecanum_PowerLimit = 0;
    
    /*========== 计算当前功率 ==========*/
    Mecanum_TotalPower = 1.7320508f * (fabs(M3508_MotorStatus[Chassis_RightFront-0x201].Power) + 
                                       fabs(M3508_MotorStatus[Chassis_RightRear-0x201].Power) + 
                                       fabs(M3508_MotorStatus[Chassis_LeftRear-0x201].Power) + 
                                       fabs(M3508_MotorStatus[Chassis_LeftFront-0x201].Power));
    
    PowerSum += Mecanum_TotalPower;
    
    /*========== 功率上限==========*/
    PowerRef = 1.0f / (60.0f - Mecanum_PowerControl_UseBuffer) * RefereeSystem_Buffer;
    
    if(PowerRef > Mecanum_PowerControl_PowerMax) PowerRef = Mecanum_PowerControl_PowerMax;
    if(PowerRef < 0.1f) PowerRef = 0.1f;
    Mecanum_PowerLimit = PowerRef * RefereeSystem_Ref;
	
	Scale=sigma/784.0f;
	if(Scale<1 && Scale>0)Mecanum_PowerLimit*=Scale;//平移约束
	if(Mecanum_GyroScopeFlag==1)Mecanum_PowerLimit=PowerRef*RefereeSystem_Ref;//小陀螺约束
    if(Mecanum_PowerLimit > 150.0f) Mecanum_PowerLimit = 150.0f; // 物理死区最大保护
    
    /*========== 前馈速度增益调节 ==========*/
    if(Count >= Mecanum_PowerControl_T)
    {
        Count = 0;
        Power_avg = PowerSum / Mecanum_PowerControl_T;
        PowerSum = 0;
        
        if(Power_avg < Mecanum_PowerLimit / 2.0f)
		{
            GainCoefficient += 0.05f; 
            if(GainCoefficient > 2.0f) GainCoefficient = 2.0f;
        }
        else if(Power_avg < Mecanum_PowerLimit - 5.0f)
		{
            GainCoefficient += 0.02f;
            if(GainCoefficient > 2.0f) GainCoefficient = 2.0f;
        }
        else if(Power_avg > Mecanum_PowerLimit + 5.0f)
		{
            if(RefereeSystem_Buffer < 60.0f - Mecanum_PowerControl_UseBuffer) GainCoefficient -= 0.05f;
            else GainCoefficient -= 0.02f;
            
            if(GainCoefficient<Mecanum_PowerControlGainCoefficientInitialValue/2.0f)GainCoefficient=Mecanum_PowerControlGainCoefficientInitialValue/2.0f;
        }
    }
    else if(vx_C != 0 || vy_C != 0 || Mecanum_GyroScopeFlag==1)
	{
        Count++;
    }
    else
	{
        GainCoefficient=Mecanum_PowerControlGainCoefficientInitialValue;
        PowerSum = 0;
    }
	if(Mecanum_GyroScopeFlag==1 && (vx_C!=0 || vy_C!=0)){V_w*=0.6f;vx_C*=0.5f;vy_C*=0.5f;}//小陀螺移动直接降速,保证移动瞬间为缓冲能量增加
    
    /*========== 电流限幅 ==========*/
    BufferCurrent = Mecanum_PowerLimit * 512.0f / 15.0f; 
    
    if(RefereeSystem_Buffer > 20.0f)
	{
        Mecanum_CurrentLimit = BufferCurrent + BufferCurrent * 5.0f; 
    }
    else
	{
        if(RefereeSystem_Buffer < 5.0f) Mecanum_CurrentLimit = BufferCurrent * 0.5f;
		else Mecanum_CurrentLimit = BufferCurrent * (RefereeSystem_Buffer + 10.0f) / 30.0f; 
    }
    
    if(Mecanum_CurrentLimit > 16384.0f * 4.0f) Mecanum_CurrentLimit = 16384.0f * 4.0f;
    
    /*========== 应用前馈速度过滤 ==========*/
    vx_C *= GainCoefficient;
    vy_C *= GainCoefficient;
    if(Mecanum_GyroScopeFlag==1)V_w*= GainCoefficient;
	
	Mecanum_InverseMotionControl(vx_C, vy_C, V_w);
}
