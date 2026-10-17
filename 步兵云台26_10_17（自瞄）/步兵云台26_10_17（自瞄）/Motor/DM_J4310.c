#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "DM_J4310.h"
#include "Delay.h"
#include "Parameter.h"

DM_J4310_Motor DM_J4310_MotorStatus[2];

uint8_t DM_J4310_CANSend(DM_J4310_ID ID,uint8_t *Data)
{
	CanTxMsg TxMessage;
	TxMessage.StdId=ID;
	TxMessage.RTR=CAN_RTR_Data;//数据帧
	TxMessage.IDE=CAN_Id_Standard;//标准格式
	TxMessage.DLC=0x08;//8字节数据段
	TxMessage.Data[0]=Data[0];
	TxMessage.Data[1]=Data[1];
	TxMessage.Data[2]=Data[2];
	TxMessage.Data[3]=Data[3];
	TxMessage.Data[4]=Data[4];
	TxMessage.Data[5]=Data[5];
	TxMessage.Data[6]=Data[6];
	TxMessage.Data[7]=Data[7];
	
	uint8_t mbox=CAN_Transmit(CAN1,&TxMessage);//发送数据并获取邮箱号
	uint16_t i=0;
	while((CAN_TransmitStatus(CAN1,mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;//等待发送结束
	if(i>=0xFFF)return 0;//发送失败
	return 1;//发送成功
}

void DM_J4310_Init(void)
{
	uint8_t Start[8]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	DM_J4310_CANSend(DM_J4310_2,Start);
	DM_J4310_CANSend(DM_J4310_2,Start);
	Delay_us(200);
	DM_J4310_CANSend(DM_J4310_2,Start);
	DM_J4310_CANSend(DM_J4310_2,Start);
}

uint16_t float_to_uint(float x,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset=x_min;
	return (uint16_t)((x-offset)*((float)((1<<bits)-1))/span);
}

void DM_J4310_Set(DM_J4310_ID ID,float tau)
{
	uint8_t Data[8];
	uint16_t Tau=float_to_uint(tau,-DM_J4310_TMAX,DM_J4310_TMAX,12);
	Data[0]=0;
    Data[1]=0;
    Data[2]=0;
    Data[3]=0;
    Data[4]=0;
    Data[5]=0;
    Data[6]=Tau>>8;
    Data[7]=Tau & 0x00FF;
	DM_J4310_CANSend(ID,Data);
}	

float uint_to_float(int x_int,float x_min,float x_max,int bits)
{
	float span=x_max-x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1))+offset;
}

void DM_J4310_DataProcess(DM_J4310_ID ID,uint8_t *Data)
{
	DM_J4310_MotorStatus[ID-0x11].Status=(Data[0]>>4) & 0x0F;

	float NowAngle=0;
	uint16_t Position=(int16_t)((uint16_t)Data[1]<<8 | Data[2]);
	uint16_t Speed=(int16_t)((uint16_t)Data[3]<<4 | Data[4]>>4);
	uint16_t T=(int16_t)((uint16_t)(Data[4]&0x0F)<<8 | Data[5]);
	
	NowAngle=uint_to_float(Position,-DM_J4310_PMAX,DM_J4310_PMAX,16)+DM4310ZeroPitch;//本次机械角度原始数据
	if(NowAngle-DM_J4310_MotorStatus[ID-0x11].Angle>DM_J4310_PMAX && DM_J4310_MotorStatus[ID-0x11].First_Flag==1)DM_J4310_MotorStatus[ID-0x11].r--;//本次机械角度原始数据和上次机械角度原始数据出现跃变
	else if(DM_J4310_MotorStatus[ID-0x11].Angle-NowAngle>DM_J4310_PMAX && DM_J4310_MotorStatus[ID-0x11].First_Flag==1)DM_J4310_MotorStatus[ID-0x11].r++;
	else if(DM_J4310_MotorStatus[ID-0x11].First_Flag!=1)DM_J4310_MotorStatus[ID-0x11].First_Flag++;
	
	DM_J4310_MotorStatus[ID-0x11].Position=NowAngle+2*DM_J4310_PMAX*DM_J4310_MotorStatus[ID-0x11].r;
	DM_J4310_MotorStatus[ID-0x11].Angle=NowAngle;
	
	DM_J4310_MotorStatus[ID-0x11].Speed=uint_to_float(Speed,-DM_J4310_VMAX,DM_J4310_VMAX,12);

	DM_J4310_MotorStatus[ID-0x11].T=uint_to_float(T,-DM_J4310_TMAX,DM_J4310_TMAX,12);
	DM_J4310_MotorStatus[ID-0x11].Power=DM_J4310_MotorStatus[ID-0x11].Speed*DM_J4310_MotorStatus[ID-0x11].T;
	if(DM_J4310_MotorStatus[ID-0x11].Power<0)DM_J4310_MotorStatus[ID-0x11].Power*=-1;
}	
