#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "RefereeSystem_CRCTable.h"
#include "Keyboard.h"
#include "Warming.h"
#include "Remote.h"
#include "Lock.h"
#include "UART.h"

/****************************************************************************************************

	此处裁判系统只接收0x0201命令,获取机器人性能体系数据,主要获取发射机构是否上电
	帧格式:
		0xA5	0x0D	0x00	包序号	帧头CRC8校验	0x01	0x02	13ByteData	整包CRC16校验
		 |___________________________________|			 |_______|			 |
						 帧头							  命令码			数据
	
****************************************************************************************************/

uint8_t RefereeSystem_ShooterStatus=1;//发射机构状态,0-发射机构未上电,1-发射机构上电
uint8_t RefereeSystem_ShooterOpenFlag=0,RefereeSystem_ShooterCloseFlag=0;//发射机构上电标志位,1-发射机构正在上电,0-其他
uint16_t RefereeSystem_ShooterOpenCounter,RefereeSystem_ShooterCloseCounter;//发射机构上电读秒等待设备启动
uint16_t RefereeSystem_Ref=55,RefereeSystem_Buffer=30.0f;//底盘功率上限,底盘功率缓冲能量(若无裁判系统,默认功率上限80W,缓冲能量30J)
uint8_t RefereeSystem_Status=0;//图传链路连接状态,0-图传链路未连接,1-图传链路连接
uint8_t RefereeSystem_RobotID=0;//机器人ID(无裁判系统时一直为0,4号步兵ID为4/104)
uint32_t RefereeSystem_RFID;
uint8_t RefereeSystem_GameStatus;

/*
 *函数简介:裁判系统CRC8查表计算
 *参数说明:校验数据
 *参数说明:数据长度
 *参数说明:CRC初始值(默认给参数0xFF)
 *返回类型:无
 *备注:表格位于Referee System_CRCTable.h文件中CRC8_Table数组
 */
uint8_t RefereeSystem_GetCRC8CheckSum(uint8_t *Data,uint16_t Length,uint8_t Initial) 
{ 
	uint8_t Minuend;
	while(Length--)
	{
		Minuend=Initial^(*Data);
		Data++;
		Initial=CRC8_Table[Minuend];
	}
	return Initial;
}

/*
 *函数简介:裁判系统CRC8校验
 *参数说明:校验数据(含尾端CRC校验码)
 *参数说明:数据长度
 *返回类型:校验正确返回1,否则返回0
 *备注:无
 */
uint8_t RefereeSystem_VerifyCRC8CheckSum(uint8_t *Data,uint16_t Length) 
{ 
	uint8_t CRC8CheckSum=0;
	if((Data==0) || (Length<=2))return 0;//特殊情况处理
	CRC8CheckSum=RefereeSystem_GetCRC8CheckSum(Data,Length-1,CRC8_Initial);//获取CRC8计算值
	return CRC8CheckSum==Data[Length-1];//测量值与计算值相比较
} 

/*
 *函数简介:裁判系统CRC16查表计算
 *参数说明:校验数据
 *参数说明:数据长度
 *参数说明:CRC初始值(默认给参数0xFFFF)
 *返回类型:无
 *备注:表格位于Referee System_CRCTable.h文件中CRC16_Table数组
 */
uint16_t RefereeSystem_GetCRC16CheckSum(uint8_t *Data,uint32_t Length,uint16_t Initial)
{ 
	uint8_t Minuend;
	while(Length--)
	{
		Minuend=*Data;
		Data++;
		Initial=((uint16_t)(Initial)>>8)^CRC16_Table[((uint16_t)(Initial)^(uint16_t)(Minuend))&0x00FF];
	}
	return Initial;
}

/*
 *函数简介:裁判系统CRC16校验
 *参数说明:校验数据(含尾端CRC校验码)
 *参数说明:数据长度
 *返回类型:校验正确返回1,否则返回0
 *备注:无
 */
uint32_t RefereeSystem_VerifyCRC16CheckSum(uint8_t *Data, uint32_t Length) 
{ 
	uint16_t CRC16CheckSum=0; 
	if((Data==0)||(Length<=2))return 0;//特殊情况处理 
	CRC16CheckSum=RefereeSystem_GetCRC16CheckSum(Data,Length-2,CRC16_Initial);//获取CRC16计算值
	return ((CRC16CheckSum&0xFF)==Data[Length-2]&&((CRC16CheckSum>>8)&0xff)==Data[Length-1]);//测量值与计算值相比较
}

/*
 *函数简介:裁判系统接收初始化
 *参数说明:无
 *返回类型:无
 *备注:默认使用UART1(USART6),默认Rx引脚PG9
 */
void RefereeSystem_Init(void)
{
	UART1_Init();
	
	if(Lock_Check()==1)//未解锁
	{
		uint32_t i=0;
		while(RefereeSystem_RobotID==0 && i<0xFFFFFFF)i++;//等待裁判系统通讯建立
		
		if(RefereeSystem_RobotID==0)//裁判系统通讯未建立
		{
			Warming_EraseFLASH();//告警FLASH即将清除
			Lock_EraseFLASH();//清除FLASH
		}
	}
}

uint8_t RefereeSystem_ReceiveNumber=1;//CAN1总线上设备数量
uint32_t RefereeSystem_ReceiveID[10][2]={{0x0304,12},0};

/*接收数据缓冲区数组元素数=命令码对应数据段长度+9*/
uint8_t RefereeSystem_RxPacket[25]={0xA5,0};//裁判系统0x0304命令码接收数据缓冲区

/*
 *函数简介:UART1串口中断接收裁判系统数据
 *参数说明:无
 *返回类型:无
 *备注:数据帧格式在最上方注释
 */
void USART6_IRQHandler(void)
{
	static int RxState=0;//定义静态变量用于接收模式的选择
	static int IDState=0;
	static int pRxState=0;//定义静态变量用于充当计数器
	
	uint8_t RefereeSystem_RxData;//裁判系统接收数据
	
	TIM_SetCounter(TIM7,0);
	TIM_Cmd(TIM7,DISABLE);//关闭定时器并重置计数值
		
	if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)//查询接收中断标志位
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);//清除接收中断标志位
		
		RefereeSystem_RxData=USART_ReceiveData(USART6);//将数据存入缓存区
		
		/*=====检查帧头=====*/
		if(RxState==0){if(RefereeSystem_RxData==0xA5)RxState=1;}
		
		/*=====检查数据段长度,区分命令码=====*/
		else if(RxState==1)
		{
			uint8_t i=0;
			for(i=0;i<RefereeSystem_ReceiveNumber;i++)
			{
				if(RefereeSystem_RxData==RefereeSystem_ReceiveID[i][1])
					break;
			}
			
			if(i==RefereeSystem_ReceiveNumber){IDState=0;RxState=0;}
			else
			{
				IDState=i+1;RxState=2;pRxState=0;
				RefereeSystem_RxPacket[1]=RefereeSystem_RxData;
			}
		}
		
		/*=====检查帧头其他部分=====*/
		else if(RxState==2)
		{
			RefereeSystem_RxPacket[pRxState+2]=RefereeSystem_RxData;
			pRxState++;
			
			if(pRxState>=3)
			{
				if(RefereeSystem_VerifyCRC8CheckSum(RefereeSystem_RxPacket,5)==1){RxState=3;pRxState=0;}
				else RxState=0;
			}
		}
		
		/*=====检查命令码=====*/
		else if(RxState==3)
		{
			RefereeSystem_RxPacket[pRxState+5]=RefereeSystem_RxData;
			pRxState++;
			
			if(pRxState>=2)
			{
				if(RefereeSystem_RxPacket[5]==(RefereeSystem_ReceiveID[IDState-1][0] & 0x00FF) && \
				   RefereeSystem_RxPacket[6]==((RefereeSystem_ReceiveID[IDState-1][0]>>8) & 0x00FF))
				{RxState=4;pRxState=0;}
				else RxState=0;
			}
		}
		
		/*=====接收有效数据=====*/
		else if(RxState==4)
		{
			RefereeSystem_RxPacket[pRxState+7]=RefereeSystem_RxData;//接收数据
			pRxState++;
			
			if(pRxState>=RefereeSystem_ReceiveID[IDState-1][1]+2)
			{
				if(RefereeSystem_VerifyCRC16CheckSum(RefereeSystem_RxPacket,RefereeSystem_ReceiveID[IDState-1][1]+9)==1)//CRC校验
				{
					if(RefereeSystem_ReceiveID[IDState-1][0]==0x0304)
					{
						Keyboard_DataProcess(&(RefereeSystem_RxPacket[7]));//获取键盘数据
						RefereeSystem_Status=1;
						
						if(Remote_StartFlag==1)//第一次接收数据
						{
							Remote_StartFlag=2;
							Warming_LEDClean();
						}
						Remote_Status=1;//遥控器已连接
					}
				}
				RxState=0;
			}
		}
				
	}
	TIM_Cmd(TIM7,ENABLE);//开启定时
}
