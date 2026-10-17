#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "M2006.h"

//M2006和M3508标识符完全相同
#define M2006_Control_ID_L		0x200//M2006低位ID发送报文标识符
#define M2006_Control_ID_H		0x1FF//M2006高位ID发送报文标识符(M2006高位标识符和GM6020低位标识符相同)

#define M2006_ReductionRatio	(36.0f/1.0f)//M2006减速比36:1
#define M2006_TorqueConstant	0.18f//M2006转矩常数0.18N·m/A

M2006_Motor M2006_MotorStatus[8];//M2006电机状态数组

/*
 *函数简介:CAN总线设置M2006低位ID电流
 *参数说明:Currrent1~4分别对应ID1~4
 *返回类型:1-发送成功,0-发送失败
 *备注:只能配置ID1~4(标识符0x200)
 *备注:默认标准格式数据帧,8字节数据段
 *备注:注意M2006的报文标识符与M3508的报文标识符完全相同
 *备注:给电机一定的电流,会促使电机产生加速度
 */
uint8_t M2006_CANSetLIDCurrent(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=M2006_Control_ID_L;//低位ID标准标识符0x200
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Current1>>8;//ID1电流高八位
	TxMessage.Data[1]=Current1;//ID1电流低八位
	TxMessage.Data[2]=Current2>>8;//ID2电流高八位
	TxMessage.Data[3]=Current2;//ID2电流低八位
	TxMessage.Data[4]=Current3>>8;//ID3电流高八位
	TxMessage.Data[5]=Current3;//ID3电流低八位
	TxMessage.Data[6]=Current4>>8;//ID4电流高八位
	TxMessage.Data[7]=Current4;//ID4电流低八位
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:CAN总线设置M2006高位ID电流
 *参数说明:Currrent5~8分别对应ID5~8
 *返回类型:1-发送成功,0-发送失败
 *备注:只能配置ID5~8(标识符0x1FF)
 *备注:默认标准格式数据帧,8字节数据段
 *备注:注意M2006的报文标识符与M3508的报文标识符完全相同
 *备注:注意M2006的ID5~8报文标识符与GM6020的ID1~4报文标识符相同
 *备注:给电机一定的电流,会促使电机产生加速度
 */
uint8_t M2006_CANSetHIDCurrent(int16_t Current5,int16_t Current6,int16_t Current7,int16_t Current8)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=M2006_Control_ID_H;//高位ID标准标识符0x1FF
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Current5>>8;//ID5电流高八位
	TxMessage.Data[1]=Current5;//ID5电流低八位
	TxMessage.Data[2]=Current6>>8;//ID6电流高八位
	TxMessage.Data[3]=Current6;//ID6电流低八位
	TxMessage.Data[4]=Current7>>8;//ID7电流高八位
	TxMessage.Data[5]=Current7;//ID7电流低八位
	TxMessage.Data[6]=Current8>>8;//ID8电流高八位
	TxMessage.Data[7]=Current8;//ID8电流低八位
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:M2006数据处理
 *参数说明:M2006电机ID号枚举,M2006_1~8对应ID号0x201~0x208
 *参数说明:反馈数据(8字节)
 *返回类型:无
 *备注:保存到M2006_MotorStatus结构体数组
 *备注:M2006减速比36:1,转矩系数0.18N·m/A
 */
void M2006_CANDataProcess(M2006_ID ID,uint8_t *Data)
{
	uint16_t M2006_RotorNowAngle=(uint16_t)((((uint16_t)Data[0])<<8)|Data[1]);//本次转子机械角度原始数据
	if(M2006_RotorNowAngle-M2006_MotorStatus[ID-0x201].RawRotorAngle>4000 && M2006_MotorStatus[ID-0x201].First_Flag==1)M2006_MotorStatus[ID-0x201].Rotor_r--;//本次转子机械角度原始数据和上次转子机械角度原始数据出现跃变
	else if(M2006_MotorStatus[ID-0x201].RawRotorAngle-M2006_RotorNowAngle>4000 && M2006_MotorStatus[ID-0x201].First_Flag==1)M2006_MotorStatus[ID-0x201].Rotor_r++;
	else if(M2006_MotorStatus[ID-0x201].First_Flag!=1)M2006_MotorStatus[ID-0x201].First_Flag=1;
	
	M2006_MotorStatus[ID-0x201].RawRotorAngle=M2006_RotorNowAngle;//转子机械角度原始数据
	M2006_MotorStatus[ID-0x201].RotorAngle=M2006_RotorNowAngle*0.0439453125f;//=M2006_RotorNowAngle/8192.0f*360.0f;//转子机械角度
	M2006_MotorStatus[ID-0x201].RawRotorPosition=8192*M2006_MotorStatus[ID-0x201].Rotor_r+M2006_RotorNowAngle;//转子角度位置原始数据
	M2006_MotorStatus[ID-0x201].RotorPosition=360.0f*M2006_MotorStatus[ID-0x201].Rotor_r+M2006_MotorStatus[ID-0x201].RotorAngle;//转子角度位置
	M2006_MotorStatus[ID-0x201].RotorSpeed=(int16_t)((((uint16_t)Data[2])<<8)|Data[3]);//转子转速原始数据
	
	M2006_MotorStatus[ID-0x201].ShaftPosition=M2006_MotorStatus[ID-0x201].RotorPosition*0.0277777777777778f;//=M2006_MotorStatus[ID-0x201].RotorPosition/M2006_ReductionRatio;//转轴角度位置
	M2006_MotorStatus[ID-0x201].Shaft_r=(int64_t)(M2006_MotorStatus[ID-0x201].ShaftPosition)/360;
	if(M2006_MotorStatus[ID-0x201].ShaftPosition<0 && M2006_MotorStatus[ID-0x201].ShaftPosition-360.0f*M2006_MotorStatus[ID-0x201].Shaft_r<0)M2006_MotorStatus[ID-0x201].Shaft_r--;//获取转轴圈数
	M2006_MotorStatus[ID-0x201].ShaftAngle=M2006_MotorStatus[ID-0x201].ShaftPosition-360.0f*M2006_MotorStatus[ID-0x201].Shaft_r;//转轴机械角度
	M2006_MotorStatus[ID-0x201].ShaftSpeed=M2006_MotorStatus[ID-0x201].RotorSpeed*0.0277777777777778f;//=M2006_MotorStatus[ID-0x201].RotorSpeed/M2006_ReductionRatio;//转轴转速
	
	M2006_MotorStatus[ID-0x201].RawCurrent=(int16_t)((((uint16_t)Data[4])<<8)|Data[5]);//转矩电流原始数据
	M2006_MotorStatus[ID-0x201].Current=M2006_MotorStatus[ID-0x201].RawCurrent*0.001f;//=M2006_MotorStatus[ID-0x201].RawCurrent/10000.0f*10.0f;//转矩电流
	
	M2006_MotorStatus[ID-0x201].Power=M2006_MotorStatus[ID-0x201].ShaftSpeed*M2006_MotorStatus[ID-0x201].Current*0.018848167539267f;//=M2006_MotorStatus[ID-0x201].ShaftSpeed*M2006_MotorStatus[ID-0x201].Current*M2006_TorqueConstant/9.55f;//电机功率(功率P(kW)=转轴转速v(RPM)*转矩T(N·m)/9550,转矩T=转矩电流*转矩常数)
	if(M2006_MotorStatus[ID-0x201].Power<0)M2006_MotorStatus[ID-0x201].Power*=-1;//功率去负数化
}
