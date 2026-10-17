#ifndef __PARAMETER_H
#define __PARAMETER_H

/*=============================================结构参数=============================================*/
#define Yaw_GM6020PositionValue								3629//Yaw轴编码器值

/*=============================================麦轮参数=============================================*/
#define Mecanum_WheelRadius									8.1f//全向轮半径(单位cm)

#define Mecanum_rx											20.0f//底盘中心到轮子中心的距离的x轴分量(单位cm)
#define Mecanum_ry											20.0f//底盘中心到轮子中心的距离的y轴分量(单位cm)

#define Mecanum_LeverSpeedMapRate							(1.2f/660.0f)//拨杆速度映射比例
#define Mecanum_GyroScopeAngularVelocity					5.0f//小陀螺角速度
//#define Mecanum_GyroScopeAngularVelocity					0.5f//小陀螺角速度
#define Mecanum_NormalSpeedRate								1.5f//底盘正常速度和超功率速度的比值

#endif
