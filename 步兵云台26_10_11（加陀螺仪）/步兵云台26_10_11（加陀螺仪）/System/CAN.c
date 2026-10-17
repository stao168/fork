#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "GM6020.h"
#include "M3508.h"
#include "Warming.h"
#include "LinkCheck.h"
#include "DM_J4310.h"
#include "Dm_J4340.h"
#include "CAN.h"

uint16_t ID1;
uint16_t ID2;

/*
 *函数简介:CAN总线初始化
 *参数说明:无
 *返回类型:无
 *备注:默认使用CAN1(CAN1-Tx为PD1,CAN1-Rx为PD0),CAN2(CAN2-Tx为PB6,CAN2-Rx为PB5)
 *备注:CAN波特率1M,默认1Tq=1/14us≈0.07us
 *备注:使用CAN2需要在打开CAN2时钟之前打开CAN1时钟,且CAN2筛选器编号为15~27
 */
void MyCAN_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//开启时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);//配置PD0(CAN1-Rx)和PD1(CAN1-Tx)
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5 | GPIO_Pin_6;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//配置PB5(CAN2-Rx)和PB6(CAN2-Tx)

	GPIO_PinAFConfig(GPIOD,GPIO_PinSource0,GPIO_AF_CAN1);//开启PD0的CAN1复用模式
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource1,GPIO_AF_CAN1);//开启PD1的CAN1复用模式
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2);//开启PB5的CAN2复用模式
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2);//开启PB6的CAN2复用模式
	
	CAN_InitTypeDef CAN_InitStructure;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;//正常模式
	CAN_InitStructure.CAN_Prescaler=3;//时钟分频,1Tq=Prescaler/T_PCLK=Prescaler/42M
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;//SJW极限值
	CAN_InitStructure.CAN_BS1=CAN_BS1_10tq;//PBS1段长度
	CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;//PBS2段长度,1/Baudrate=T_1bit=(1+BS1+BS2)Tq
	CAN_InitStructure.CAN_TTCM=DISABLE;//关闭时间触发功能
	CAN_InitStructure.CAN_ABOM=ENABLE;//开启自动离线管理功能
	CAN_InitStructure.CAN_AWUM=ENABLE;//开启自动唤醒功能
	CAN_InitStructure.CAN_NART=ENABLE;//禁止自动重传功能
	CAN_InitStructure.CAN_RFLM=DISABLE;//关闭锁定FIFO功能
	CAN_InitStructure.CAN_TXFP=DISABLE;//配置报文优先级判定为标识符决定
	CAN_Init(CAN1,&CAN_InitStructure);
	CAN_Init(CAN2,&CAN_InitStructure);
	
	CAN_FilterInitTypeDef CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber=0;//筛选器编号0(编号0~14属于CAN1,编号15~27属于CAN2)
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//存入FIFO0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;//掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;//筛选器尺度为16bits
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x000<<5;//标识符=STID[15:5]+RTR[4:4](0)+IDE[3:3](0)+EXID[2:0](000)
	CAN_FilterInitStructure.CAN_FilterIdLow=0;//筛选器尺度为16bits,故没有高低位之分
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0;//掩码(1111 1111 0000 0011),1对应位必须匹配,0对应位无需匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0;//筛选器尺度为16bits,故没有高低位之分
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	CAN_FilterInitStructure.CAN_FilterNumber=15;//筛选器编号15(编号0~14属于CAN1,编号15~27属于CAN2)
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;//存入FIFO1
	CAN_FilterInitStructure.CAN_FilterIdHigh=(0x000<<5);//标识符=STID[15:5](189/0001 1000 1001)+RTR[4:4](0)+IDE[3:3](0)+EXID[2:0](000)
	CAN_FilterInitStructure.CAN_FilterIdLow=(0x000<<5);//CAN2先给0,默认从CAN1开始串行接收
	CAN_FilterInit(&CAN_FilterInitStructure);
	
	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//打通CAN1_FIFO0到NVIC的通道
	CAN_ITConfig(CAN2,CAN_IT_FMP1,ENABLE);//打通CAN2_FIFO1到NVIC的通道
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级，2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=CAN1_RX0_IRQn;//选择CAN1_FIFO0接收中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化CAN1_FIFO0的NVIC
	NVIC_InitStructure.NVIC_IRQChannel=CAN2_RX1_IRQn;//选择CAN2_FIFO1接收中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化CAN2_FIFO1的NVIC
	
}

/*
 *函数简介:CAN1总线接收报文
 *参数说明:报文存储数组
 *返回类型:报文ID
 *备注:默认8字节标准数据帧
 *备注:没有接收到数据,直接退出,返回0
 */
uint32_t CAN_CAN1Receive(uint8_t *Data)
{
	CanRxMsg RxMessage;
	
	if(CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;//没有接收到数据,直接退出
	
	CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);//读取数据
	
	for(uint32_t i=0;i<8;i++)
		Data[i]=RxMessage.Data[i];//储存数据
	
	return RxMessage.StdId;//返回ID
}

/*
 *函数简介:CAN2总线接收报文
 *参数说明:报文存储数组
 *返回类型:报文ID
 *备注:默认8字节数据
 *备注:没有接收到数据,直接退出,返回0
 */
uint32_t CAN_CAN2Receive(uint8_t *Data)
{
	CanRxMsg RxMessage;
	
	if(CAN_MessagePending(CAN2,CAN_FIFO1)==0)return 0;//没有接收到数据,直接退出
	
	CAN_Receive(CAN2,CAN_FIFO1,&RxMessage);//读取数据
	
	for(uint32_t i=0;i<8;i++)
		Data[i]=RxMessage.Data[i];//储存数据
	
	return RxMessage.StdId;//返回ID
}

uint16_t Flag;
/*
 *函数简介:CAN1_FIFO0接收中断函数
 *参数说明:无
 *返回类型:无
 *备注:只能接收ID1
 */
void CAN1_RX0_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0)==SET)
	{
		uint8_t Data[8];
		
		if (CAN_MessagePending(CAN1, CAN_FIFO0) > 0)
		{
			Flag ++;
			ID1 = CAN_CAN1Receive(Data);
			switch(ID1)
			{
				case DM_J4310_IDReceiveOffset + DM_J4310_2:
					DM_J4310_DataProcess(ID1, Data);
					Can_Test[CAN1_DM4310] = 0;
					break;
				case M3508_1:
					M3508_CANDataProcess(ID1, Data);
					Can_Test[CAN1_M3508_1] = 0;
					break;
				case M3508_2:
					M3508_CANDataProcess(ID1, Data);
					Can_Test[CAN1_M3508_2] = 0;
					break;
			}
		}

		CAN_ClearITPendingBit(CAN1,CAN_IT_FMP0);
	}
}

#include "UART.h"

/*
 *函数简介:CAN2_FIFO1接收中断函数
 *参数说明:无
 *返回类型:无
 *备注:只能接收ID1
 */
void CAN2_RX1_IRQHandler(void)
{
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP1)==SET)
	{
		uint8_t Data[8];
		
		if (CAN_MessagePending(CAN2, CAN_FIFO1) > 0)
		{
			ID2 = CAN_CAN2Receive(Data);
			if(ID2 == GM6020_1)
			{
				GM6020_CANDataProcess(ID2, Data);
				Can_Test[CAN2_GM6020] = 0;
			}
		}
		
		CAN_ClearITPendingBit(CAN2,CAN_IT_FMP1);
	}
}

