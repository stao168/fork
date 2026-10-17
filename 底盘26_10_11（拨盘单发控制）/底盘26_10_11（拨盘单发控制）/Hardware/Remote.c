#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Remote.h"
#include "LED.h"
#include <stdlib.h>
#include "Warming.h"
#include "Dial.h"
#include "CToC.h"

uint8_t Remote_RxData0[25];//遥控器DMA数据存储器0
uint8_t Remote_RxData1[25];//遥控器DMA数据存储器1

Remote_Data Remote_LastRxData, Remote_RxData;//遥控器上一次接收数据, 遥控器接收数据
uint8_t Remote_Status;//遥控器连接状态,默认未连接/掉电(0),连接(1),有电未连接(2)
uint8_t Remote_StartFlag=1;//遥控器启动标志位,0-未在启动阶段,1-准备启动,2-第一次接收到数据

/*
 *函数简介:遥控器初始化
 *参数说明:无
 *返回类型:无
 *备注:默认接收引脚为PC11(USART3-Rx)
 *备注:采用串口DMA双缓冲接收
 *备注:配置定时中断为TIM7 25ms,用来检测遥控器连接情况
 *备注:加入独立看门狗,用来在数据接收错误时复位(只在上电的一刻可能数据错误)
 *备注:独立看门狗时钟为LSI(32kHz),预分频数默认为8,故IWDG时钟4kHz,重装载值为0x0FFF(4095)
 *备注:喂狗时间(溢出时间)T_OUT=Reload/(LSI/Prescaler)=4095/(32k/8)=1.02375s
 */
void Remote_Init(void)
{
	/*===============配置时钟===============*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);//开启时钟
	
	/*===============配置GPIO===============*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//复用推挽
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);//初始化USART3-Rx(PC11)
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);//开启PC11的USART3复用模式
	
	/*===============配置USART和串口接收DMA===============*/
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate=100000;//配置波特率100k
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;//配置无硬件流控制
	USART_InitStructure.USART_Mode=USART_Mode_Rx;//配置为接收模式
	USART_InitStructure.USART_Parity=USART_Parity_Even;//配置为偶校验
	USART_InitStructure.USART_StopBits=USART_StopBits_1;//配置停止位为1
	USART_InitStructure.USART_WordLength=USART_WordLength_9b;//配置字长9bit
	USART_Init(USART3,&USART_InitStructure);//初始化USART3
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel=DMA_Channel_4;//选择DMA通道4
	DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;//普通模式(非自动重装)
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;//转运方向为外设到存储器
	DMA_InitStructure.DMA_BufferSize=25;//数据传输量为25字节
	DMA_InitStructure.DMA_Priority=DMA_Priority_VeryHigh;//最高优先级
	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(USART3->DR);//外设地址(USART的DR数据接收寄存器)
	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;//外设数据长度为1字节(8bits)
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//外设地址不自增
	DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)Remote_RxData0;//存储器地址(遥控器DMA数据存储器0)
	DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;//存储器数据长度为1字节(8bits)
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;//存储器地址自增
	DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Disable;//不使用FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold=DMA_FIFOStatus_1QuarterFull;//设置FIFO阈值为1/4(不使用FIFO模式时,此位无意义)
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);//初始化数据流1
	
	DMA_DoubleBufferModeConfig(DMA1_Stream1,(uint32_t)Remote_RxData1,DMA_Memory_0);//设置双缓冲搬运从遥控器DMA数据存储器0开始
	DMA_DoubleBufferModeCmd(DMA1_Stream1,ENABLE);//使能DMA双缓冲功能
	
	/*===============配置定时器===============*/
	TIM_InternalClockConfig(TIM7);//选择时基单元的时钟(TIM7)
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;//配置时基单元（配置参数）
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//配置时钟分频为1分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//配置计数器模式为向上计数
	TIM_TimeBaseInitStructure.TIM_Period=2100-1;//配置自动重装值ARR
	TIM_TimeBaseInitStructure.TIM_Prescaler=1000-1;//配置分频值PSC,默认定时25ms
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//配置重复计数单元的置为0
	TIM_TimeBaseInit(TIM7,&TIM_TimeBaseInitStructure);//初始化TIM7
	
	TIM_ClearFlag(TIM7,TIM_FLAG_Update);//清除配置时基单元产生的中断标志位
	
	/*===============配置接收中断和定时器中断===============*/
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);//打通USART3到NVIC的串口接收中断通道
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);//使能TIM7更新中断
		
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组2(2位抢占优先级,2位响应优先级)
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=USART3_IRQn;//选择USART3中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化USART3的NVIC
	NVIC_InitStructure.NVIC_IRQChannel=TIM7_IRQn;//选择中断通道为TIM2
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//TIM2的抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2;//TIM2的响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	/*===============使能===============*/
	DMA_Cmd(DMA1_Stream1,ENABLE);//使能DMA1的数据流1
	USART_DMACmd(USART3,USART_DMAReq_Rx,ENABLE);//使能串口USART3的DMA搬运
	USART_Cmd(USART3,ENABLE);//启动USART3
	TIM_Cmd(TIM7,ENABLE);//启动定时器
	
	/*===============配置IWDG===============*/
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_8);//8分频
	IWDG_SetReload(0x0FFF);//设置重装载值
	IWDG_Enable();
}

/*
 *函数简介:遥控器开启
 *参数说明:无
 *返回类型:无
 *备注:默认开启串口USART3
 */
void Remote_ON(void)
{
	USART_Cmd(USART3,ENABLE);//启动USART3
}

/*
 *函数简介:遥控器关闭
 *参数说明:无
 *返回类型:无
 *备注:默认关闭串口USART3
 */
void Remote_OFF(void)
{
	USART_Cmd(USART3,DISABLE);//失能USART3
	Remote_Status=0;//遥控器连接状态变为未连接
}

/*
 *函数简介:遥控器DMA转运复位
 *参数说明:无
 *返回类型:无
 *备注:无
 */
void Remote_TransferReset(void)
{
	while(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1)==RESET);//判断接收完成
	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);//清除接收完成标志位
	DMA_Cmd(DMA1_Stream1,DISABLE);//失能DMA1的数据流1
	while(DMA_GetCmdStatus(DMA1_Stream1)!=DISABLE);//检测DMA1的数据流1为可配置状态
	DMA_SetCurrDataCounter(DMA1_Stream1,25);//恢复传输计数器的值
	DMA_Cmd(DMA1_Stream1,ENABLE);//使能DMA1的数据流1
}

/*
 *函数简介:遥控器数据处理
 *参数说明:无
 *返回类型:无
 *备注:遥控器接收数据共25Bytes
 */
void Remote_DataProcess(void)
{
	uint8_t *Data;//选择存储器
	if(DMA_GetCurrentMemoryTarget(DMA1_Stream1)==0)Data=Remote_RxData1;//若当前转运位于存储器0,则存储器1数据完整,采用存储器1进行数据处理
	else Data=Remote_RxData0;//若当前转运位于存储器1,则存储器0数据完整,采用存储器0进行数据处理
	
	if(Data[0] != 0x0F) {
		Remote_RxData.Remote_Online = 0xFF;
        return;  // 不是有效帧
    }
	
	//覆盖旧数据
	Remote_LastRxData = Remote_RxData;
	
	//接收新数据
	Remote_RxData.Remote_R_RL=(((uint16_t)Data[2]<<8) | Data[1]) & 0x07FF;//B[0:10],11bits
	Remote_RxData.Remote_R_UD=(((uint16_t)Data[3]<<5) | (Data[2]>>3)) & 0x07FF;//B[11:21],11bits
	Remote_RxData.Remote_L_UD=(((uint16_t)Data[5]<<10) | (((uint16_t)Data[4]<<2) | (Data[3]>>6))) & 0x07FF;//B[22:32],11bits
	Remote_RxData.Remote_L_RL=(((uint16_t)Data[6]<<7) | (Data[5]>>1)) & 0x07FF;//B[33:43],11bits
	
	Remote_RxData.Remote_SWA=((int16_t)Data[6] >> 4 | ((int16_t)Data[7] << 4 )) & 0x07FF;//B[44:45],2bits
	Remote_RxData.Remote_SWB=((int16_t)Data[7] >> 7 | ((int16_t)Data[8] << 1 ) | (int16_t)Data[9] << 9 ) & 0x07FF;//B[46:47],2bits
	
	Remote_RxData.Remote_SWC=((int16_t)Data[9] >> 2 | ((int16_t)Data[10] << 6 )) & 0x07FF;//B[48:63],16bits
	Remote_RxData.Remote_SWD=((int16_t)Data[10] >> 5 | ((int16_t)Data[11] << 3 )) & 0x07FF;//B[64:79],16bits
	Remote_RxData.Remote_VRA=((int16_t)Data[12] >> 0 | ((int16_t)Data[13] << 8 )) & 0x07FF;//B[80:95],16bits
	Remote_RxData.Remote_VRB=((int16_t)Data[13] >> 3 | ((int16_t)Data[14] << 5 )) & 0x07FF;//B[96:103],8bits

	Remote_RxData.Remote_Online = Data[23];//遥控器在线标志位
	
	//按需要处理旧数据
	if(Remote_LastRxData.Remote_SWD == 0xF0 && Remote_RxData.Remote_SWD == 0x400 && Shoot_OpenFlag == 1) Dial_SingleShootFlag = 1;
}

/*
 *函数简介:遥控器接收中断
 *参数说明:无
 *返回类型:无
 *备注:USART的接收中断
 */
void USART3_IRQHandler(void)
{
	TIM_SetCounter(TIM7,0);
	TIM_Cmd(TIM7,DISABLE);//关闭定时器并重置计数值
	if(DMA_GetCurrDataCounter(DMA1_Stream1)==25)//转运一次完成,并交换了存储器
	{
		Remote_TransferReset();//复位DMA1的数据流1
		Remote_DataProcess();//数据处理
		
		if(Remote_RxData.Remote_Online==0)//正常连接状态
		{
			if(Remote_StartFlag==1)//第一次接收数据
			{
				while(abs(Remote_RxData.Remote_R_RL - 1024) > 10 || abs(Remote_RxData.Remote_R_UD - 1024) > 10 || abs(Remote_RxData.Remote_L_RL - 1024) > 10 || abs(Remote_RxData.Remote_L_UD - 1024) > 10)//数据错误
//				while(Remote_RxData.Remote_R_RL != 1024 || Remote_RxData.Remote_R_UD != 1024 || Remote_RxData.Remote_L_RL != 1024 || Remote_RxData.Remote_L_UD != 1024)//数据错误
					Warming_RemoteDataERROR();//数据错误报错,等待看门狗复位
				Remote_StartFlag=2;
				Warming_LEDClean();
			}
			Remote_Status=1;//遥控器已连接
		}
		else
		{
			Remote_Status=2;//遥控器掉线
			while(Remote_RxData.Remote_Online == 0xFF)
			{
				Warming_MotorControl();
				Warming_RemoteDataERROR();//数据错误报错,等待看门狗复位
			}
		}
	}
	TIM_Cmd(TIM7,ENABLE);//开启定时
	
	USART_ClearITPendingBit(USART3,USART_IT_RXNE);//清除接收中断标志位
}

/*
 *函数简介:TIM7定时器更新中断函数
 *参数说明:无
 *返回类型:无
 *备注:进入中断即接收机掉电
 */
void TIM7_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM7,TIM_IT_Update)==SET)//检测TIM2更新
	{
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);//清除标志位
		Warming_RemoteNoCheck();//遥控器未连接报警
		Remote_Status=0;//遥控器连接状态变为未连接
		Remote_StartFlag=1;//遥控器处于准备启动阶段
	}
}

