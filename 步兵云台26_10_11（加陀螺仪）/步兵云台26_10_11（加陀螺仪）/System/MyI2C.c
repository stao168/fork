#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "MyI2C.h"

/*
 *函数简介:软件I2C专用us延时
 *参数说明:延时时长,单位1/21us
 *返回类型:无
 *备注:参数范围:0~4294967295
 */
void MyI2C_Delay_us(uint32_t us)
{
	uint32_t temp;	    	 
	SysTick->LOAD=us; 							//时间加载，我们要延时n倍的us, 1us是一个fac_ua周期，所以总共要延时的周期值为二者相乘最后送到Load中。		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开启使能位 开始倒数
	do temp=SysTick->CTRL;
	while((temp&0x01) && !(temp&(1<<16)));		//用来判断 systick 定时器是否还处于开启状态，然后在等待时间到达，也就是数到0的时候,此时第十六位设置为1
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器使能位
	SysTick->VAL=0x00;       					//清空计数器
}

/*
 *函数简介:软件I2C写SCL电平
 *参数说明:软件I2C配置结构体
 *参数说明:高低电平,0-低电平 1-高电平
 *返回类型:无
 *备注:使用前需要先定义结构体并初始化
 *备注:由于I2C速度较慢,在更改电平之后需要加入延时
 */
void MyI2C_SCL(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t x)
{
	GPIO_WriteBit(MyI2C_InitStructure->MyI2C_SCL_GPIOx,MyI2C_InitStructure->MyI2C_SCL_Pin,(BitAction)(x));
	MyI2C_Delay_us(1);//延时
}

/*
 *函数简介:软件I2C写SDA电平
 *参数说明:软件I2C配置结构体
 *参数说明:高低电平,0-低电平 1-高电平
 *返回类型:
 *备注:使用前需要先定义结构体并初始化
 *备注:由于I2C速度较慢,在更改电平之后需要加入延时
 */
void MyI2C_SDA(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t x)
{
	GPIO_WriteBit(MyI2C_InitStructure->MyI2C_SDA_GPIOx,MyI2C_InitStructure->MyI2C_SDA_Pin,(BitAction)(x));
	MyI2C_Delay_us(1);//延时
}

/*
 *函数简介:软件I2C读SDA电平
 *参数说明:软件I2C配置结构体
 *返回类型:高低电平,0-低电平 1-高电平
 *备注:使用前需要先定义结构体并初始化
 *备注:由于I2C速度较慢,在电平更改之后需要加入延时,读取时也可以不加
 */
uint8_t MyI2C_Read_SDA(MyI2C_InitTypedef *MyI2C_InitStructure)
{
	MyI2C_Delay_us(1);//延时
	uint8_t SDA_Status=GPIO_ReadInputDataBit(MyI2C_InitStructure->MyI2C_SDA_GPIOx,MyI2C_InitStructure->MyI2C_SDA_Pin);
	return SDA_Status;
}

/*
 *函数简介:软件I2C初始化
 *参数说明:软件I2C配置结构体
 *返回类型:无
 *备注:使用前需要先定义结构体并初始化
 */
void MyI2C_Init(MyI2C_InitTypedef *MyI2C_InitStructure)
{
	RCC_AHB1PeriphClockCmd(MyI2C_InitStructure->MyI2C_SCL_RCC,ENABLE);//开启SCL时钟
	RCC_AHB1PeriphClockCmd(MyI2C_InitStructure->MyI2C_SDA_RCC,ENABLE);//开启SDA时钟
	
	GPIO_InitTypeDef GPIO_InitStructure;
 	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_OD;//开漏输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=MyI2C_InitStructure->MyI2C_SCL_Pin;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
 	GPIO_Init(MyI2C_InitStructure->MyI2C_SCL_GPIOx,&GPIO_InitStructure);//配置SCL
	GPIO_InitStructure.GPIO_Pin=MyI2C_InitStructure->MyI2C_SDA_Pin;
 	GPIO_Init(MyI2C_InitStructure->MyI2C_SDA_GPIOx,&GPIO_InitStructure);//配置SDA
	
	MyI2C_SCL(MyI2C_InitStructure,1);//初始化SCL
	MyI2C_SDA(MyI2C_InitStructure,1);//初始化SDA
}

/*
 *函数简介:软件I2C起始
 *参数说明:软件I2C配置结构体
 *返回类型:无
 *I2C协议:
 *	SCL	~~~~/---\___
 *	SDA	~~/---\_____
 */
void MyI2C_Start(MyI2C_InitTypedef *MyI2C_InitStructure)
{
	MyI2C_SDA(MyI2C_InitStructure,1);
	MyI2C_SCL(MyI2C_InitStructure,1);//复位SCL和SDA
	MyI2C_SDA(MyI2C_InitStructure,0);//起始
	MyI2C_SCL(MyI2C_InitStructure,0);//准备接收数据
}

/*
 *函数简介:软件I2C终止
 *参数说明:软件I2C配置结构体
 *返回类型:无
 *I2C协议:
 *	SCL	____/-----
 *	SDA	~~\___/---
 */
void MyI2C_Stop(MyI2C_InitTypedef *MyI2C_InitStructure)
{
	MyI2C_SDA(MyI2C_InitStructure,0);
	MyI2C_SCL(MyI2C_InitStructure,1);//准备接收停止信号
	MyI2C_SDA(MyI2C_InitStructure,1);//终止
}

/*
 *函数简介:软件I2C产生应答
 *参数说明:软件I2C配置结构体
 *参数说明:是否应答,0-NAck非应答 1-Ack应答
 *返回类型:无
 *I2C协议:
 *	Ack:
 *		SCL____/-\___
 *		SDA~~\___/---
 *	NAck:
 *		SCL____/-\___
 *		SDA~~/-------
 */
void MyI2C_SendAck(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t AckBit)
{
	MyI2C_SDA(MyI2C_InitStructure,!AckBit);//发送应答情况
	MyI2C_SCL(MyI2C_InitStructure,1);//产生时钟
	MyI2C_SCL(MyI2C_InitStructure,0);
	MyI2C_SDA(MyI2C_InitStructure,1);//释放总线
}

/*
 *函数简介:软件I2C接收应答
 *参数说明:软件I2C配置结构体
 *返回类型:是否应答,0-NAck非应答 1-Ack应答
 *I2C协议:
 *	SCL	____/-\___
 *	SDA	~~/-|==|~~
 *			(  )从机
 */
uint8_t MyI2C_ReceiveAck(MyI2C_InitTypedef *MyI2C_InitStructure)
{
	uint8_t AckBit;//应答标志位
	
	MyI2C_SDA(MyI2C_InitStructure,1);//释放SDA总线
	MyI2C_SCL(MyI2C_InitStructure,1);//提示从机发送Ack
	
	AckBit=!MyI2C_Read_SDA(MyI2C_InitStructure);//接收应答
	
	MyI2C_SCL(MyI2C_InitStructure,0);//置SCL为0,准备下一轮发送
	return AckBit;//返回应答情况
}

/*
 *函数简介:软件I2C发送一个8bits数据
 *参数说明:软件I2C配置结构体
 *参数说明:8bits发送数据
 *返回类型:无
 *I2C协议:
 *	SCL	____/-\___/-\___/-\___/-\___/-\___/-\___/-\___/-\___
 *	SDA	~~x=====x=====x=====x=====x=====x=====x=====x=====~~
 */
void MyI2C_Send8bits(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Byte)
{
	for(uint8_t i=0;i<8;i++)
	{
		MyI2C_SDA(MyI2C_InitStructure,Byte&(0x80>>i));//发送数据每一位,高位先行
		MyI2C_SCL(MyI2C_InitStructure,1);//产生时钟
		MyI2C_SCL(MyI2C_InitStructure,0);
	}
	
	MyI2C_SDA(MyI2C_InitStructure,1);//释放SDA总线
}

/*
 *函数简介:软件I2C读取一个8bits数据
 *参数说明:软件I2C配置结构体
 *返回类型:8bits接收数据
 *I2C协议:
 *	SCL	____/-\_/-\_/-\_/-\_/-\_/-\_/-\_/-\___
 *	SDA	~~/-|==============================|~~
 *			(							   )从机
 */
uint8_t MyI2C_Receive8bits(MyI2C_InitTypedef *MyI2C_InitStructure)
{
	uint8_t Byte=0x00;
	MyI2C_SDA(MyI2C_InitStructure,1);//释放SDA总线
	
	for(uint8_t i=0;i<8;i++)//读取数据每一位,高位先行
	{
		MyI2C_SCL(MyI2C_InitStructure,1);//产生时钟
		if(MyI2C_Read_SDA(MyI2C_InitStructure)){Byte|=(0x80>>i);}//读取SDA
		MyI2C_SCL(MyI2C_InitStructure,0);
	}
	
	return Byte;
}

/*
 *函数简介:软件I2C发送一个字节数据
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *参数说明:8bits发送数据
 *返回类型:无
 *备注:无
 */
void MyI2C_SendByte(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address,uint8_t Byte)
{
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,Address<<1);//发送设备地址(写)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	MyI2C_Send8bits(MyI2C_InitStructure,Byte);//发送数据
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
}

/*
 *函数简介:软件I2C发送数据
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *参数说明:发送数据数组
 *参数说明:发送数据数组长度
 *返回类型:无
 *备注:无
 */
void MyI2C_SendData(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address,uint8_t *Data,uint16_t Length)
{
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,Address<<1);//发送设备地址(写)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	for(uint16_t i=0;i<Length;i++)
	{
		MyI2C_Send8bits(MyI2C_InitStructure,Data[i]);//发送数据
		while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	}
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
}

/*
 *函数简介:软件I2C写寄存器
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *参数说明:8bits寄存器地址
 *参数说明:8bits数据
 *返回类型:无
 *备注:无
 */
void MyI2C_WriteRegister(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t RegAddress,uint8_t Data)
{
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,SlaveAddress<<1);//发送从机地址(写)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	MyI2C_Send8bits(MyI2C_InitStructure,RegAddress);//发送寄存器地址
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	MyI2C_Send8bits(MyI2C_InitStructure,Data);//发送数据
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
}

/*
 *函数简介:软件I2C接收一个字节数据
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *返回类型:8bits接收数据
 *备注:无
 */
uint8_t MyI2C_ReceiveByte(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address)
{
	uint8_t Byte=0;
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,(Address<<1) | 0x01);//发送设备地址(读)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	Byte=MyI2C_Receive8bits(MyI2C_InitStructure);//接收数据
	MyI2C_SendAck(MyI2C_InitStructure,0);//发送NAck
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
	return Byte;
}

/*
 *函数简介:软件I2C接收数据
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *参数说明:接收数据数组
 *参数说明:接收数据数组长度
 *返回类型:8bits接收数据
 *备注:无
 */
void MyI2C_ReceiveData(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address,uint8_t *Data,uint16_t Length)
{
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,(Address<<1) | 0x01);//发送设备地址(读)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	for(uint16_t i=0;i<Length-1;i++)
	{
		Data[i]=MyI2C_Receive8bits(MyI2C_InitStructure);//接收数据
		MyI2C_SendAck(MyI2C_InitStructure,1);//发送Ack
	}
	Data[Length-1]=MyI2C_Receive8bits(MyI2C_InitStructure);//接收最后一个数据
	MyI2C_SendAck(MyI2C_InitStructure,0);//发送NAck
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
}

/*
 *函数简介:软件I2C读寄存器
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *参数说明:8bits寄存器地址
 *返回类型:8bits数据
 *备注:无
 */
uint8_t MyI2C_ReadRegister(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t RegAddress)
{
	uint8_t Byte=0;
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,SlaveAddress<<1);//发送从机地址(写)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	MyI2C_Send8bits(MyI2C_InitStructure,RegAddress);//发送寄存器地址
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	MyI2C_Start(MyI2C_InitStructure);//重复起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,(SlaveAddress<<1) | 0x01);//发送设备地址(读)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	Byte=MyI2C_Receive8bits(MyI2C_InitStructure);//接收数据
	MyI2C_SendAck(MyI2C_InitStructure,0);//发送NAck
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
	return Byte;
}

/*
 *函数简介:软件I2C连续读寄存器
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *参数说明:8bits寄存器首地址
 *参数说明:接收数据数组
 *参数说明:接收数据数组长度
 *返回类型:无
 *备注:无
 */
void MyI2C_ContinuousReadRegister(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t RegStartAddress,uint8_t *ReceiveData,uint16_t Length)
{
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,SlaveAddress<<1);//发送从机地址(写)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	MyI2C_Send8bits(MyI2C_InitStructure,RegStartAddress);//发送寄存器地址
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	MyI2C_Start(MyI2C_InitStructure);//重复起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,(SlaveAddress<<1) | 0x01);//发送设备地址(读)
	while(MyI2C_ReceiveAck(MyI2C_InitStructure)!=1);//等待Ack应答
	
	for(uint16_t i=0;i<Length-1;i++)
	{
		ReceiveData[i]=MyI2C_Receive8bits(MyI2C_InitStructure);//接收数据
		MyI2C_SendAck(MyI2C_InitStructure,1);//发送Ack
	}
	ReceiveData[Length-1]=MyI2C_Receive8bits(MyI2C_InitStructure);//接收最后一个数据
	MyI2C_SendAck(MyI2C_InitStructure,0);//发送NAck
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
}

/*
 *函数简介:软件I2C检测设备状态
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *返回类型:工作状态,0-设备不存在或异常工作 1-设备正常工作
 *备注:无
 */
uint8_t MyI2C_CheckDevice(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address)
{
	uint8_t WorkStatus;//工作标志位
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,Address<<1);//发送设备地址(写)
	WorkStatus=MyI2C_ReceiveAck(MyI2C_InitStructure);//检测Ack应答
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
	return WorkStatus;
}

/*
 *函数简介:软件I2C查询Who Am I寄存器
 *参数说明:软件I2C配置结构体
 *参数说明:7bits从机地址
 *参数说明:8bits Who Am I寄存器地址
 *返回类型:Who Am I寄存器的值,0-设备不存在或异常工作 其他-返回寄存器值
 *备注:无
 */
uint8_t MyI2C_CheckWhoAmI(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t WhoAmIAddress)
{
	uint8_t WhoAmI;
	MyI2C_Start(MyI2C_InitStructure);//起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,SlaveAddress<<1);//发送设备地址(写)
	if(MyI2C_ReceiveAck(MyI2C_InitStructure)==0)return 0;//检测Ack应答
	MyI2C_Send8bits(MyI2C_InitStructure,WhoAmIAddress);//发送寄存器地址
	if(MyI2C_ReceiveAck(MyI2C_InitStructure)==0)return 0;//检测Ack应答
	
	MyI2C_Start(MyI2C_InitStructure);//重复起始
	
	MyI2C_Send8bits(MyI2C_InitStructure,(SlaveAddress<<1) | 0x01);//发送设备地址(读)
	if(MyI2C_ReceiveAck(MyI2C_InitStructure)==0)return 0;//检测Ack应答
	
	WhoAmI=MyI2C_Receive8bits(MyI2C_InitStructure);//接收数据
	MyI2C_SendAck(MyI2C_InitStructure,0);//发送NAck
	
	MyI2C_Stop(MyI2C_InitStructure);//终止
	return WhoAmI;
}
