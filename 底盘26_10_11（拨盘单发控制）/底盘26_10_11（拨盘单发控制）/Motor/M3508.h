#include <stm32f4xx.h>
#ifndef __M3508_H
#define __M3508_H

#define PI 3.14159265358979323846f

#define M3508_StandardReductionRatio    (3591.0f / 187.0f) //M3508标准减速比
#define M3508_ReductionRatio			(268.0f/17.0f) //M3508实际减速比

#define M3508_StandardTorqueConstant    0.3f //M3508标准转矩常数
#define M3508_TorqueConstant			(M3508_StandardTorqueConstant * M3508_ReductionRatio / M3508_StandardReductionRatio)

typedef enum
{
	M3508_1=0x201,//ID1
	M3508_2=0x202,//ID2
	M3508_3=0x203,//ID3
	M3508_4=0x204,//ID4
	M3508_5=0x205,//ID5
	M3508_6=0x206,//ID6
	M3508_7=0x207,//ID7
	M3508_8=0x208,//ID8
}M3508_ID;//M3508电机ID号枚举

typedef struct
{
	uint8_t First_Flag;//M3508电机首次接收标志位
	int64_t Rotor_r;//M3508电机转子转过圈数
	uint16_t RawRotorAngle;//M3508电机转子机械角度原始数据(范围0~8191,对应0~360°,注意:8192对应360°)
	float RotorAngle;//M3508电机转子机械角度(单位°)
	int64_t RawRotorPosition;//M3508电机转子角度位置原始数据
	float RotorPosition;//M3508电机转子角度位置(单位°)
	int16_t RotorSpeed;//M3508电机转子转速(单位RPM)
	
	int64_t Shaft_r;//M3508电机转轴转过圈数
	float ShaftAngle;//M3508电机转轴机械角度(单位°)
	float ShaftPosition;//M3508电机转轴角度位置(单位°)
	float ShaftSpeed;//M3508电机转轴转速(单位RPM)
	
	int16_t RawCurrent;//M3508电机转矩电流原始数据(范围-16384~16384,对应-20A~20A,注意:16384对应20A)
	float Current;//M3508电机转矩电流(单位A)
	
	float Power;//M3508电机功率(单位W)
	uint8_t Temperature;//M3508电机电机温度(单位℃)
}M3508_Motor;//M3508电机状态结构体(减速比3591:187(≈19:1),转矩系数0.3N·m/A)

extern M3508_Motor M3508_MotorStatus[];//M3508电机状态数组

uint8_t M3508_CAN1SetLIDCurrent(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4);//CAN总线设置M3508低位ID电流
uint8_t M3508_CAN1SetHIDCurrent(int16_t Current5,int16_t Current6,int16_t Current7,int16_t Current8);
void M3508_CANDataProcess(M3508_ID ID,uint8_t *Data);//M3508数据处理

#endif
