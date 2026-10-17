#ifndef __MYI2C_H
#define __MYI2C_H

typedef struct
{
	uint32_t MyI2C_SCL_RCC;//SCL引脚时钟
	GPIO_TypeDef *MyI2C_SCL_GPIOx;//SCL引脚的GPIO
	uint32_t MyI2C_SCL_Pin;//SCL引脚的Pin号
	
	uint32_t MyI2C_SDA_RCC;//SDA引脚时钟
	GPIO_TypeDef *MyI2C_SDA_GPIOx;//SDA引脚的GPIO
	uint32_t MyI2C_SDA_Pin;//SDA引脚的Pin号
}MyI2C_InitTypedef;//软件I2C配置结构体


void MyI2C_Init(MyI2C_InitTypedef *MyI2C_InitStructure);//软件I2C初始化
void MyI2C_Start(MyI2C_InitTypedef *MyI2C_InitStructure);//软件I2C起始
void MyI2C_Stop(MyI2C_InitTypedef *MyI2C_InitStructure);//软件I2C终止
void MyI2C_SendAck(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t AckBit);//软件I2C产生应答
uint8_t MyI2C_ReceiveAck(MyI2C_InitTypedef *MyI2C_InitStructure);//软件I2C接收应答
void MyI2C_Send8bits(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Byte);//软件I2C发送一个8bits数据
uint8_t MyI2C_Receive8bits(MyI2C_InitTypedef *MyI2C_InitStructure);//软件I2C读取一个8bits数据

void MyI2C_SendByte(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address,uint8_t Byte);//软件I2C发送一个字节数据
void MyI2C_SendData(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address,uint8_t *Data,uint16_t Length);//软件I2C发送数据
void MyI2C_WriteRegister(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t RegAddress,uint8_t Data);//软件I2C写寄存器
uint8_t MyI2C_ReceiveByte(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address);//软件I2C接收一个字节数据
void MyI2C_ReceiveData(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address,uint8_t *Data,uint16_t Length);//软件I2C接收数据
uint8_t MyI2C_ReadRegister(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t RegAddress);//软件I2C读寄存器
void MyI2C_ContinuousReadRegister(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t RegStartAddress,uint8_t *ReceiveData,uint16_t Length);//软件I2C连续读寄存器

uint8_t MyI2C_CheckDevice(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t Address);//软件I2C检测设备状态
uint8_t MyI2C_CheckWhoAmI(MyI2C_InitTypedef *MyI2C_InitStructure,uint8_t SlaveAddress,uint8_t WhoAmIAddress);//软件I2C查询Who Am I寄存器

#endif
