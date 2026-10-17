#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "GM6020.h"

#define GM6020_Control_ID_L		0x1FE//GM6020低位ID发送报文标识符
#define GM6020_Control_ID_H		0x2FE//GM6020低位ID发送报文标识符

GM6020_Motor GM6020_MotorStatus[7];//GM6020电机状态数组

/*
 *函数简介:CAN1总线设置GM6020低位ID电流
 *参数说明:Current1~4分别对应ID1~4
 *返回类型:1-发送成功,0-发送失败
 *备注:只能配置ID1~4(标识符0x1FE)
 *备注:默认标准格式数据帧,4字节数据段
 *备注:给电机一定的电流,会促使电机产生加速度
 */
uint8_t GM6020_CAN2SetLIDCurrent(int16_t Current1,int16_t Current2,int16_t Current3,int16_t Current4)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=GM6020_Control_ID_L;//低位ID标准标识符0x1FE
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
	
	uint8_t mbox=CAN_Transmit(CAN2,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN2,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:CAN2总线设置GM6020高位ID电流
 *参数说明:Current5~7分别对应ID5~7
 *返回类型:1-发送成功,0-发送失败
 *备注:只能配置ID5~7(标识符0x2FE)
 *备注:默认标准格式数据帧,4字节数据段
 *备注:给电机一定的电流,会促使电机产生加速度
 */
uint8_t GM6020_CAN2SetHIDCurrent(int16_t Current5,int16_t Current6,int16_t Current7)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=GM6020_Control_ID_H;//低位ID标准标识符0x2FE
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Current5>>8;//ID5电流高八位
	TxMessage.Data[1]=Current5;//ID5电流低八位
	TxMessage.Data[2]=Current6>>8;//ID6电流高八位
	TxMessage.Data[3]=Current6;//ID6电流低八位
	TxMessage.Data[4]=Current7>>8;//ID7电流高八位
	TxMessage.Data[5]=Current7;//ID7电流低八位
	TxMessage.Data[6]=0;//ID4电流高八位
	TxMessage.Data[7]=0;//ID4电流低八位
	
	uint8_t mbox=CAN_Transmit(CAN2,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN2,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

/*
 *函数简介:GM6020数据处理
 *参数说明:GM6020电机ID号枚举,GM6020_1~7对应ID号0x205~0x20B
 *参数说明:反馈数据(8字节)
 *返回类型:无
 *备注:保存到GM6020_MotorStatus结构体数组
 */
void GM6020_CANDataProcess(GM6020_ID ID,uint8_t *Data)
{
	uint16_t GM6020_NowAngle=(uint16_t)((((uint16_t)Data[0])<<8)|Data[1]);//本次机械角度原始数据
	if(GM6020_NowAngle-GM6020_MotorStatus[ID-0x205].Angle>4000 && GM6020_MotorStatus[ID-0x205].First_Flag==1 && GM6020_MotorStatus[ID-0x205].r>0)GM6020_MotorStatus[ID-0x205].r--;//本次机械角度原始数据和上次机械角度原始数据出现跃变
	else if(GM6020_MotorStatus[ID-0x205].Angle-GM6020_NowAngle>4000 && GM6020_MotorStatus[ID-0x205].First_Flag==1 && GM6020_MotorStatus[ID-0x205].r<1)GM6020_MotorStatus[ID-0x205].r++;
	else if(GM6020_MotorStatus[ID-0x205].First_Flag!=1)GM6020_MotorStatus[ID-0x205].First_Flag++;

	GM6020_MotorStatus[ID-0x205].Angle=GM6020_NowAngle;//机械角度
	GM6020_MotorStatus[ID-0x205].Position=8192*GM6020_MotorStatus[ID-0x205].r+GM6020_NowAngle;//角度位置
	GM6020_MotorStatus[ID-0x205].Speed=(int16_t)((((uint16_t)Data[2])<<8)|Data[3]);//转速
	GM6020_MotorStatus[ID-0x205].Current=(int16_t)((((uint16_t)Data[4])<<8)|Data[5]);//实际转矩电流
	GM6020_MotorStatus[ID-0x205].Temperature=Data[6];//电机温度
}
