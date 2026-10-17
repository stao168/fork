#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "MyI2C.h"
#include "Warming.h"
#include "Delay.h"

#define IST8310_Address		0x0E//IST8310的地址

#define IST8310_Who_Am_I	0x00//Who Am I寄存器
#define IST8310_CNTL1		0x0A//控制设置1寄存器
#define IST8310_CNTL2		0x0B//控制设置2寄存器
#define IST8310_AVGCNTL		0x41//采样平均寄存器
#define IST8310_PDCNTL		0x42//脉冲持续时间寄存器
#define IST8310_DataStart	0x03//数据寄存器首地址
#define IST8310_TEMPL		0x1C//温度寄存器-温度低八位
#define IST8310_TEMPH		0x1D//温度寄存器-温度高八位

MyI2C_InitTypedef MyI2C_InitStruction;//I2C配置结构体
uint8_t IST8310_DataStartFlag;//IST8310开始接收数据标志位
int16_t IST8310_RawData[4];//IST8310原始数据,1~3为xyz轴磁场原始数据,4为温度原始数据
float IST8310_MagneticField[3];//IST8310磁场数据,分别为xyz轴磁场数据(单位uT)
float IST8310_Temperature;//IST8310温度数据(单位℃)

/*
 *函数简介:IST8310的RSTN延时
 *参数说明:无
 *返回类型:无
 *备注:默认配置RSTN延时500us,重启一次延时1ms
 */
void IST8310_Delay_us(void)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=500*21; 						//时间加载，我们要延时500倍的us, 1us是一个fac_ua周期，所以总共要延时的周期值为二者相乘最后送到Load中。		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开启使能位 开始倒数
	do temp=SysTick->CTRL;
	while((temp&0x01) && !(temp&(1<<16)));		//用来判断 systick 定时器是否还处于开启状态，然后在等待时间到达，也就是数到0的时候,此时第十六位设置为1
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器使能位
	SysTick->VAL=0x00;       					//清空计数器
}

/*
 *函数简介:IST8310配置RSTN电平
 *参数说明:高低电平,0-低电平 1-高电平
 *返回类型:无
 *备注:规定RSTN为PG6
 *备注:RSTN为低电平时重启IST8310
 *备注:每次修改RSTN,需要等待一段时间产生作用(测试得到0.5ms足矣)
 */
void IST8310_RSTN(uint8_t x)
{
	GPIO_WriteBit(GPIOG,GPIO_Pin_6,(BitAction)(x));
	IST8310_Delay_us();//延时0.5ms
}

/*
 *函数简介:IST8310重启
 *参数说明:无
 *返回类型:无
 *备注:重启大概耗时1ms
 */
void IST8310_Reset(void)
{
	IST8310_RSTN(0);
	IST8310_RSTN(1);
}

/*
 *函数简介:IST8310初始化
 *参数说明:无
 *返回类型:无
 *备注:规定SCL为PA8,SDA为PC9,DRDY为PG3(下降沿)
 *备注:初始化中配置了寄存器并进行了重启,大概耗时1ms
 *备注:在初始化中对寄存器进行了配置
 *备注:初始化时会检查IST8310的ID,ID错误会进行报警,并且程序会卡死不断的进行检测,具体报警现象见Warming.h
 */
void IST8310_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);//开启时钟
	
	MyI2C_InitStruction.MyI2C_SCL_RCC=RCC_AHB1Periph_GPIOA;//配置SCL为PA8
	MyI2C_InitStruction.MyI2C_SCL_GPIOx=GPIOA;
	MyI2C_InitStruction.MyI2C_SCL_Pin=GPIO_Pin_8;
	MyI2C_InitStruction.MyI2C_SDA_RCC=RCC_AHB1Periph_GPIOC;//配置SDA为PC9
	MyI2C_InitStruction.MyI2C_SDA_GPIOx=GPIOC;
	MyI2C_InitStruction.MyI2C_SDA_Pin=GPIO_Pin_9;
	MyI2C_Init(&MyI2C_InitStruction);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//配置DRDY(PG3)
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOG,EXTI_PinSource3);//配置PG3与中断线的映射关系
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line=EXTI_Line3;//配置中断线3
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;//使能中断线
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//配置为外部中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//选择下降沿触发
	EXTI_Init(&EXTI_InitStructure);//初始化EXTI
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组为2
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=EXTI3_IRQn;//配置NVIC通道3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能NVIC通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	while(1)
	{
		if(MyI2C_CheckDevice(&MyI2C_InitStruction,IST8310_Address)==1 && MyI2C_CheckWhoAmI(&MyI2C_InitStruction,IST8310_Address,IST8310_Who_Am_I)==0x10)//设备连接正常,Who Am I正确
		{
			NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//开启NVIC通道
			NVIC_Init(&NVIC_InitStructure);//初始化NVIC
			
			MyI2C_WriteRegister(&MyI2C_InitStruction,IST8310_Address,IST8310_CNTL1,0x03);//配置为连续输出模式,输出频率200Hz
			MyI2C_WriteRegister(&MyI2C_InitStruction,IST8310_Address,IST8310_CNTL2,0x08);//配置为DRDY下降沿中断,不重启
			MyI2C_WriteRegister(&MyI2C_InitStruction,IST8310_Address,IST8310_AVGCNTL,0x12);//配置为四次采样平均
			MyI2C_WriteRegister(&MyI2C_InitStruction,IST8310_Address,IST8310_PDCNTL,0xC0);//配置为正常脉冲持续时间
			IST8310_Reset();//IST8310重启
			break;
		}
		else//设备异常
		{
			NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;//关闭NVIC通道
			NVIC_Init(&NVIC_InitStructure);//初始化NVIC
			Warming_IST8310LinkError();//IST8310连接异常报警
			Delay_ms(25);
		}
	}
	
	while(IST8310_DataStartFlag==0);//等待数据开始接收
}

/*
 *函数简介:IST8310数据读取
 *参数说明:无
 *返回类型:无
 *备注:读取三轴磁场(单位uT)和温度(单位℃)
 *备注:温度转换比例没找到,暂且以0.001为比例转换
 */
void IST8310_GetData(void)
{
	uint8_t IST8310_MagneticFieldReceive[6],TempL,TempH;//三轴磁场返回数据,温度返回数据(低八位),温度返回数据(高八位)
	
	MyI2C_ContinuousReadRegister(&MyI2C_InitStruction,IST8310_Address,IST8310_DataStart,IST8310_MagneticFieldReceive,6);//读取三轴磁场返回数据
	TempL=MyI2C_ReadRegister(&MyI2C_InitStruction,IST8310_Address,IST8310_TEMPL);//读取温度返回数据(低八位)
	TempH=MyI2C_ReadRegister(&MyI2C_InitStruction,IST8310_Address,IST8310_TEMPH);//读取温度返回数据(高八位)
	
	IST8310_RawData[0]=(int16_t)(((uint16_t)(IST8310_MagneticFieldReceive[1])<<8)|IST8310_MagneticFieldReceive[0]);
	IST8310_RawData[1]=(int16_t)(((uint16_t)(IST8310_MagneticFieldReceive[3])<<8)|IST8310_MagneticFieldReceive[2]);
	IST8310_RawData[2]=(int16_t)(((uint16_t)(IST8310_MagneticFieldReceive[5])<<8)|IST8310_MagneticFieldReceive[4]);//获取三轴磁场原始数据
	IST8310_RawData[3]=(int16_t)(((uint16_t)(TempH)<<8)|TempL);//获取温度原始数据

	IST8310_MagneticField[0]=IST8310_RawData[0]*0.3f;
	IST8310_MagneticField[1]=IST8310_RawData[1]*0.3f;
	IST8310_MagneticField[2]=IST8310_RawData[2]*0.3f;//获取三轴磁场数据
	IST8310_Temperature=IST8310_RawData[3]*0.001f;//获取温度
}

/*
 *函数简介:IST8310外部中断中断函数
 *参数说明:无
 *返回类型:无
 *备注:在中断函数中读取数据
 */
void EXTI3_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line3)==SET)//检测IST8310外部中断触发(即检测EXTI通道3中断触发)
	{
		EXTI_ClearITPendingBit(EXTI_Line3);//清除标志位
		IST8310_GetData();//读取数据
		IST8310_DataStartFlag=1;//数据开始接收
	}
}

/*硬件I2C的遗骸*/
////void IST8310_I2CInit(void)
////{
////	
////	
////	GPIO_InitTypeDef GPIO_InitStructure;
////	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
////	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;
////	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
////	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;
////	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
////	GPIO_Init(GPIOA,&GPIO_InitStructure);
////	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
////	GPIO_Init(GPIOC,&GPIO_InitStructure);
////	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
////	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
////	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
////	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;
////	GPIO_Init(GPIOG,&GPIO_InitStructure);
////	
////	GPIO_PinAFConfig(GPIOA,GPIO_PinSource8,GPIO_AF_I2C3);
////	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_I2C3);
////	
////	I2C_InitTypeDef I2C_InitStructure;
////	I2C_InitStructure.I2C_Mode=I2C_Mode_I2C;
////	I2C_InitStructure.I2C_ClockSpeed=10000;
////	I2C_InitStructure.I2C_DutyCycle=I2C_DutyCycle_2;
////	I2C_InitStructure.I2C_Ack=I2C_Ack_Disable;
////	I2C_InitStructure.I2C_AcknowledgedAddress=I2C_AcknowledgedAddress_7bit;
////	I2C_InitStructure.I2C_OwnAddress1=0x01;
////	I2C_Init(I2C3,&I2C_InitStructure);
////	
////	I2C3->CR1 |= I2C_CR1_SWRST;
////	while(I2C_GetFlagStatus(I2C3,I2C_FLAG_BUSY)==SET);
////	I2C3->CR1 &= ~I2C_CR1_SWRST;
////	
////	I2C_Cmd(I2C3,ENABLE);
////	
////	GPIO_ResetBits(GPIOG,GPIO_Pin_6);
////	Delay_ms(50);
////	GPIO_SetBits(GPIOG,GPIO_Pin_6);
////	Delay_ms(50);
////	LED_BInit();
////}

////void IST8310_I2CWriteData(uint8_t Address,uint8_t Data)
////{
////	uint8_t B=0;
////	while(I2C_GetFlagStatus(I2C3,I2C_FLAG_BUSY)==SET);
////	
////	I2C_GenerateSTART(I2C3,ENABLE);
////	while(I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);
////	
////	while(I2C_GetFlagStatus(I2C3,I2C_FLAG_BUSY)!=SET);
////	while(I2C_GetFlagStatus(I2C3,I2C_FLAG_MSL)!=SET);
////	//I2C_SendData(I2C3,0x0E<<1);
////	int i=0;
////	do
////	{
////		I2C_Send7bitAddress(I2C3,i<<1,I2C_Direction_Receiver);
////		i++;
////		if(i==0x8F)break;
////	}
////	while(I2C_GetFlagStatus(I2C3,I2C_FLAG_ADDR)!=SET);
////	if(i<0x8F)LED_BON();
////	//while(I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS);

////	I2C_SendData(I2C3,0x00);
////	//while(I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_BYTE_TRANSMITTING)!=SUCCESS);
////	
//////	I2C_SendData(I2C3,Data);
////	//while(I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_BYTE_TRANSMITTED)!=SUCCESS);
////	I2C_GenerateSTART(I2C3,ENABLE);
////	//while(I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_MODE_SELECT)!=SUCCESS);
////	
////	//I2C_SendData(I2C3,(0x0E<<1)|0x01);
////	I2C_Send7bitAddress(I2C3,0x0E,I2C_Direction_Receiver);
////	//while(I2C_GetFlagStatus(I2C3,I2C_FLAG_AF)==SET);
////	//while(I2C_GetFlagStatus(I2C3,I2C_FLAG_ADDR)!=SET);
////	//while(I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)!=SUCCESS);

////	B=I2C_ReceiveData(I2C3);
////	//while(I2C_CheckEvent(I2C3,I2C_EVENT_MASTER_BYTE_TRANSMITTING)!=SUCCESS);
////	
////	I2C_GenerateSTOP(I2C3,ENABLE);
////}
