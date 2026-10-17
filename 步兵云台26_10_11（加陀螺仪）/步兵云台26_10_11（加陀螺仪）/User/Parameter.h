#ifndef __PARAMETER_H
#define __PARAMETER_H

/*=============================================结构参数=============================================*/
#define Yaw_GM6020PositionValue								3641//Yaw轴编码器值
#define Pitch_AngleLowerLinit								0.13f//Pitch轴编码器值下限
#define Pitch_AngleUpperLinit								-0.67f//Pitch轴编码器值上限
#define DM4310ZeroPitch										0.354302f
#define DM4340ZeroPitch										0.354302f

/*=============================================云台参数=============================================*/
#define Gimbal_FrictionWheelSpeed							(4200/60.0f*2.0f*3.1415926f)//摩擦轮转速,弹速限制30m/s
#define Gimbal_RammerCount									6.0f//六爪2000//7200//拨弹盘转速,射频为7时大概冷却和热量相抵,5400是射频20的最低下限
#define Gimbal_RammerSpeed									1.0f//每秒20发2000//7200//拨弹盘转速,射频为7时大概冷却和热量相抵,5400是射频20的最低下限

#define Gimbal_LeverSpeedMapRate							5.0f//0.5f//云台俯仰拨杆速度映射比例
#define Gimbal_YawPitchSpeedRate							1.5f//云台偏航俯仰速度比

/*=============================================操作手参数=============================================*/
#define PC_Go												(Remote_RxData.Remote_Key_W)//前
#define PC_Back												(Remote_RxData.Remote_Key_S)//后
#define PC_Left												(Remote_RxData.Remote_Key_A)//左
#define PC_Right											(Remote_RxData.Remote_Key_D)//右

#define PC_Spin												(Remote_RxData.Remote_Mouse_RL)//视角水平移动
#define PC_Pitch											(Remote_RxData.Remote_Mouse_DU)//视角垂直移动
#define PC_Mouse_RLSensitivity								3.0f//鼠标左右灵敏度
#define PC_Mouse_DUSensitivity								8.0f//鼠标上下灵敏度

#define PC_FrictionWheel									(Remote_RxData.Remote_KeyPush_Q)//摩擦轮
#define PC_Fire												(Remote_RxData.Remote_Mouse_KeyL)//发射
#define PC_Ejection											(Remote_RxData.Remote_Key_E)//退弹

#define PC_GyroScope										(Remote_RxData.Remote_KeyPush_Ctrl)//小陀螺
#define PC_SpeedUp											(Remote_RxData.Remote_Key_Shift)//加速

#define PC_VT												(Remote_RxData.Remote_KeyPush_E)
#define PC_Scope											(Remote_RxData.Remote_KeyPush_R)

#endif
