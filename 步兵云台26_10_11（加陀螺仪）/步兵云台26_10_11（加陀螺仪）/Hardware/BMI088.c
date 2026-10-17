#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "Warming.h"

/*
 *SPI模式选择:
 *
 *CPOL(时钟极性):
 *		0/Low-空闲状态时,SCK=0
 *		1/High-空闲状态时,SCK=1
 *CPHA(时钟相位):
 *		0/1Edge-SCK第一个边沿移入数据,第二个边沿移出数据
 *		1/2Edge-SCK第一个边沿移出数据,第二个边沿移入数据
 *
 *模式0:CPOL=0,CPHA=0
 *模式1:CPOL=0,CPHA=1
 *模式2:CPOL=1,CPHA=0
 *模式3:CPOL=1,CPHA=1
 */

//加速度计寄存器
#define BMI088_ACC_CHIP_ID				0x00//加速度计Who Am I寄存器
#define BMI088_ACC_PWR_CTRL				0x7D//加速度计电源控制寄存器
#define BMI088_ACC_PWR_CONF				0x7C//加速度计电源配置寄存器
#define BMI088_ACC_CONF					0x40//加速度计配置寄存器
#define BMI088_ACC_RANGE				0x41//加速度计范围设置寄存器
#define BMI088_INT1_IO_CONF				0x53//加速度计配置INT1输入输出引脚寄存器
#define BMI088_INT1_INT2_MAP_DATA		0x58//加速度计映射数据就绪中断到INT1/INT2寄存器
#define BMI088_ACC_SOFTRESET			0x7E//加速度计软件重启寄存器
#define BMI088_ACC_AccelDataStart		0x12//加速度计数据寄存器首地址
#define BMI088_ACC_TEMP_MSB				0x22//加速度计温度寄存器高八位
#define BMI088_ACC_TEMP_LSB				0x23//加速度计温度寄存器低三位

//陀螺仪寄存器
#define BMI088_GYRO_CHIP_ID				0x00//陀螺仪Who Am I寄存器
#define BMI088_GYRO_RANGE				0x0F//陀螺仪范围设置寄存器
#define BMI088_GYRO_BANDWIDTH			0x10//陀螺仪带宽寄存器
#define BMI088_GYRO_LPM1				0x11//陀螺仪主电源模式选择寄存器
#define BMI088_GYRO_INT_CTRL			0x15//陀螺仪中断控制寄存器
#define BMI088_INT3_INT4_IO_CONF		0x16//陀螺仪配置中断输入输出引脚寄存器
#define BMI088_INT3_INT4_IO_MAP			0x18//陀螺仪映射数据就绪中断到INT3/INT4寄存器
#define BMI088_GYRO_SOFTRESET			0x14//陀螺仪软件重启寄存器
#define BMI088_GYRO_GyroDataStart		0x02//陀螺仪数据寄存器首地址

#define BMI088_ACC_Address				0x1E//加速度计地址
#define BMI088_GYRO_Address				0x0F//陀螺仪地址


#define BMI088_CS_Accel(x)				GPIO_WriteBit(GPIOA,GPIO_Pin_4,(BitAction)(x))//写加速度计的片选引脚
#define BMI088_CS_Gyro(x)				GPIO_WriteBit(GPIOB,GPIO_Pin_0,(BitAction)(x))//写陀螺仪的片选引脚

#define BMI088_Accel_Start()			BMI088_CS_Accel(0)//BMI088加速度计硬件SPI数据交换开始(拉低BMI088_CS_Accel完成片选)
#define BMI088_Accel_Stop()				BMI088_CS_Accel(1)//BMI088加速度计硬件SPI数据交换结束(拉高BMI088_CS_Accel结束片选)
#define BMI088_Gyro_Start()				BMI088_CS_Gyro(0)//BMI088陀螺仪硬件SPI数据交换开始(拉低BMI088_CS_Gyro完成片选)
#define BMI088_Gyro_Stop()				BMI088_CS_Gyro(1)//BMI088陀螺仪硬件SPI数据交换结束(拉高BMI088_CS_Gyro结束片选)


uint8_t BMI088_SPI_DMASend[8],BMI088_SPI_DMAReceive[8];//BMI088的SPI发送DMA存储器数组,BMI088的SPI接收DMA存储器数组
uint8_t BMI088_FirstDMA_Flag=2,BMI088_DMAAccGyroSelect;//BMI088的DMA首次启动标志位(0-已经首次启动,1-准备首次启动,2-等待首次启动),BMI088的DMA处于加速度计数据接收还是陀螺仪数据接收的状态选择(1-加速度计数据接收,2-陀螺仪数据接收)
uint8_t BMI088_DataStartFlag;//BMI088开始接收数据标志位

int16_t BMI088_RawAccelData[3],BMI088_RawGyroData[3],BMI088_RawTemperatureData;//BMI088的三轴加速度原始数据,三轴角速度原始数据和温度原始数据
float BMI088_Accel[3],BMI088_Gyro[3],BMI088_Temperature;//BMI088的三轴加速度数据,三轴角速度数据和温度

/*
 *函数简介:BMI088专用微秒级延时
 *参数说明:延时时长,单位us
 *返回类型:无
 *备注:参数范围:0~4294967295
 */
void BMI088_Delay_us(uint32_t us)
{		
	uint32_t temp;	    	 
	SysTick->LOAD=us*21; 						//时间加载，我们要延时n倍的us, 1us是一个fac_ua周期，所以总共要延时的周期值为二者相乘最后送到Load中。		 
	SysTick->VAL=0x00;        					//清空计数器
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;	//开启使能位 开始倒数
	do temp=SysTick->CTRL;
	while((temp&0x01) && !(temp&(1<<16)));		//用来判断 systick 定时器是否还处于开启状态，然后在等待时间到达，也就是数到0的时候,此时第十六位设置为1
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk;	//关闭计数器使能位
	SysTick->VAL=0x00;       					//清空计数器
}

/*
 *函数简介:BMI088加速度计写寄存器
 *参数说明:8bits寄存器地址
 *参数说明:8bits寄存器写入数据
 *返回类型:无
 *备注:在第一个数据发送时,第二个数据已经在发送缓冲区等待以实现连续数据流
 *备注:BMI088的寄存器地址的最高位bit需要为0(即&0x7F),以实现写寄存器
 */
void BMI088_SPI_AccelWriteRegister(uint8_t RegAddress,uint8_t Data)
{
	BMI088_Accel_Start();//开始
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,RegAddress & 0x7F);//将地址(写数据)放入发送缓冲区
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,Data);//SPI发送数据(放入发送缓冲区,实现连续数据流)
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第一个数据(地址数据)移位完成
	SPI_I2S_ReceiveData(SPI1);//此数据来自发送地址,故读取当前接收缓冲区以清除接收数据和标志位
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第二个数据(接收数据)移位完成
	SPI_I2S_ReceiveData(SPI1);//SPI接收数据,清除接收数据和标志位
	
	BMI088_Accel_Stop();//结束
}

/*
 *函数简介:BMI088加速度计读寄存器
 *参数说明:8bits寄存器地址
 *返回类型:8bits寄存器读出数据
 *备注:在第一个数据发送时,第二个数据已经在发送缓冲区等待以实现连续数据流
 *备注:BMI088的寄存器地址的最高位bit需要为1(即|0x80),以实现读寄存器
 *备注:加速度计读取寄存器时(包括连续读取和非连续读取),第一个读出数据是乱码,从第二个开始才是有效数据
 */
uint8_t BMI088_SPI_AccelReadRegister(uint8_t RegAddress)
{
	BMI088_Accel_Start();//开始
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,RegAddress | 0x80);//将地址(读数据)放入发送缓冲区
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,0x00);//SPI发送数据(放入发送缓冲区,实现连续数据流)
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第一个数据(地址数据)移位完成
	SPI_I2S_ReceiveData(SPI1);//此数据来自发送地址,故读取当前接收缓冲区以清除接收数据和标志位
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,0x00);//SPI发送数据(放入发送缓冲区,实现连续数据流)
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第二个数据(地址数据)移位完成
	SPI_I2S_ReceiveData(SPI1);//此回传数据为乱码,故读取当前接收缓冲区以清除接收数据和标志位
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第三个数据(接收数据)移位完成
	BMI088_Accel_Stop();//结束
	
	return SPI_I2S_ReceiveData(SPI1);//SPI接收数据
}

/*
 *函数简介:BMI088陀螺仪写寄存器
 *参数说明:8bits寄存器地址
 *参数说明:8bits寄存器写入数据
 *返回类型:无
 *备注:在第一个数据发送时,第二个数据已经在发送缓冲区等待以实现连续数据流
 *备注:BMI088的寄存器地址的最高位bit需要为0(即&0x7F),以实现写寄存器
 */
void BMI088_SPI_GyroWriteRegister(uint8_t RegAddress,uint8_t Data)
{
	BMI088_Gyro_Start();//开始
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,RegAddress & 0x7F);//将地址(写数据)放入发送缓冲区
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,Data);//SPI发送数据(放入发送缓冲区,实现连续数据流)
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第一个数据(地址数据)移位完成
	SPI_I2S_ReceiveData(SPI1);//此数据来自发送地址,故读取当前接收缓冲区以清除接收数据和标志位
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第二个数据(接收数据)移位完成
	SPI_I2S_ReceiveData(SPI1);//SPI接收数据,清除接收数据和标志位
	
	BMI088_Gyro_Stop();//结束
}

/*
 *函数简介:BMI088陀螺仪读寄存器
 *参数说明:8bits寄存器地址
 *返回类型:8bits寄存器读出数据
 *备注:在第一个数据发送时,第二个数据已经在发送缓冲区等待以实现连续数据流
 *备注:BMI088的寄存器地址的最高位bit需要为1(即|0x80),以实现读寄存器
 */
uint8_t BMI088_SPI_GyroReadRegister(uint8_t RegAddress)
{
	BMI088_Gyro_Start();//开始
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,RegAddress | 0x80);//将地址(读数据)放入发送缓冲区
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE)!=SET);//发送缓冲区为空
	SPI_I2S_SendData(SPI1,0x00);//SPI发送数据(放入发送缓冲区,实现连续数据流)
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第一个数据(地址数据)移位完成
	SPI_I2S_ReceiveData(SPI1);//此数据来自发送地址,故读取当前接收缓冲区以清除接收数据和标志位
	
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)!=SET);//接收缓冲区非空,即第二个数据(接收数据)移位完成
	BMI088_Gyro_Stop();//结束
	
	return SPI_I2S_ReceiveData(SPI1);//SPI接收数据
}

/*
 *函数简介:BMI088初始化
 *参数说明:无
 *返回类型:无
 *备注:使用SPI1,SPI模式3(CPOL=1,CPHA=1),波特率5.25MHz
 *备注:规定SPI1_CLK为PB3,SPI1_MISO为PB4,SPI1_MOSI为PA7,CS1_Accel为PA4(加速度计片选引脚),CS1_Gyro为PB0(陀螺仪片选引脚)
 *备注:规定INT1_Accel为PC4,INT1_Gyro为PC5,分别连接芯片的INT1和INT3,配置为数据就绪中断(下降沿)
 *备注:采用SPI收发DMA节约CPU资源(SPI1发送DMA为DMA2数据流3通道3,SPI1接收DMA为DMA2数据流2通道3)
 *备注:在初始化中对寄存器进行了配置,最关键配置为加速度量程±3g,角速度量程±2000°/s
 *备注:配置寄存器时会对加速度计和陀螺仪进行软重启,加速度计耗时1ms,陀螺仪耗时30ms,同时软重启之后加速度计回到I2C模式,需要虚拟SPI读取切换到SPI模式(此次读取的数据是错误的)
 *备注:加速度计软重启之后需要重新上电,需要配置BMI088_ACC_PWR_CTRL和BMI088_ACC_PWR_CONF两个寄存器,每次配置需要等待一段时间,测试得到250us足矣,共耗时500us,保险起见采用300us,共耗时600us
 *备注:保险起见在每一次配置寄存器之后加入300us的延时,共计3.6ms
 *备注:初始化时会检查加速度计和陀螺仪的ID,ID错误会进行报警,并且程序会卡死不断的进行检测,具体报警现象见Warming.h
 */
void BMI088_Init(void)
{
	/*===============配置时钟===============*/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);//开启时钟
	
	/*===============配置GPIO===============*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//默认上拉
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//配置CS1_Accel(加速度计片选引脚)
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//配置CS1_Gyro(陀螺仪片选引脚)
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//复用推挽
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_Init(GPIOB,&GPIO_InitStructure);//配置SPI1_CLK和SPI1_MISO
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_7;
	GPIO_Init(GPIOA,&GPIO_InitStructure);//配置SPI1_MOSI
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IN;//上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_Init(GPIOC,&GPIO_InitStructure);//配置INT1_Accel和INT1_Gyro(加速度计和陀螺仪的中断线)
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1);//开启PA7的SPI1复用模式
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_SPI1);//开启PB3的SPI1复用模式
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_SPI1);//开启PB4的SPI1复用模式
	
	/*===============配置SPI和SPI收发DMA===============*/
	SPI_InitTypeDef SPI_InitStructure;
	SPI_InitStructure.SPI_Mode=SPI_Mode_Master;//主机模式
	SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;//双线全双工
	SPI_InitStructure.SPI_DataSize=SPI_DataSize_8b;//8位数据帧
	SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;//高位先行
	SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_8;//波特率预分频系数8,SCK频率为PCLK/分频系数,即Freq_SCK=42MHz/8=5.25MHz
	SPI_InitStructure.SPI_CPOL=SPI_CPOL_High;//CPOL=1
	SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge;//第二个边沿开始采样,即CPHA=1
	SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;//软件NSS模式
	SPI_InitStructure.SPI_CRCPolynomial=10;//CRC检测,不懂
	SPI_Init(SPI1,&SPI_InitStructure);
	
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel=DMA_Channel_3;//选择DMA通道3
	DMA_InitStructure.DMA_Mode=DMA_Mode_Normal;//普通模式(非自动重装)
	DMA_InitStructure.DMA_DIR=DMA_DIR_MemoryToPeripheral;//转运方向为存储器到外设
	DMA_InitStructure.DMA_BufferSize=8;//数据传输量为8字节
	DMA_InitStructure.DMA_Priority=DMA_Priority_VeryHigh;//最高优先级
	DMA_InitStructure.DMA_PeripheralBaseAddr=(uint32_t)&(SPI1->DR);//外设地址(SPI1的DR数据接收寄存器)
	DMA_InitStructure.DMA_PeripheralBurst=DMA_PeripheralBurst_Single;//外设突发单次传输
	DMA_InitStructure.DMA_PeripheralDataSize=DMA_PeripheralDataSize_Byte;//外设数据长度为1字节(8bits)
	DMA_InitStructure.DMA_PeripheralInc=DMA_PeripheralInc_Disable;//外设地址不自增
	DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)BMI088_SPI_DMASend;//存储器地址(BMI088的SPI发送DMA存储器数组)
	DMA_InitStructure.DMA_MemoryBurst=DMA_MemoryBurst_Single;//存储器突发单次传输
	DMA_InitStructure.DMA_MemoryDataSize=DMA_MemoryDataSize_Byte;//存储器数据长度为1字节(8bits)
	DMA_InitStructure.DMA_MemoryInc=DMA_MemoryInc_Enable;//存储器地址自增
	DMA_InitStructure.DMA_FIFOMode=DMA_FIFOMode_Disable;//不使用FIFO模式
	DMA_InitStructure.DMA_FIFOThreshold=DMA_FIFOStatus_1QuarterFull;//设置FIFO阈值为1/4(不使用FIFO模式时,此位无意义)
	DMA_Init(DMA2_Stream3,&DMA_InitStructure);//初始化数据流3(SPI1发送DMA)
	
	DMA_InitStructure.DMA_DIR=DMA_DIR_PeripheralToMemory;//转运方向为外设到存储器
	DMA_InitStructure.DMA_Memory0BaseAddr=(uint32_t)BMI088_SPI_DMAReceive;//存储器地址(BMI088的SPI接收DMA存储器数组)
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);//初始化数据流2(SPI1接收DMA)
	
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Tx,ENABLE);//使能SPI1的发送DMA搬运
	SPI_I2S_DMACmd(SPI1,SPI_I2S_DMAReq_Rx,ENABLE);//使能SPI1的接收DMA搬运
	
	BMI088_CS_Accel(1);//复位CS1_Accel(加速度计片选引脚)
	BMI088_CS_Gyro(1);//复位CS1_Gyro(陀螺仪片选引脚)
		
	/*===============配置外部中断和SPI1接收DMA传输完成中断===============*/
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource4);//配置PC4与中断线的映射关系
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC,EXTI_PinSource5);//配置PC5与中断线的映射关系
	
	EXTI_InitTypeDef EXTI_InitStructure;
	EXTI_InitStructure.EXTI_Line=EXTI_Line4;//配置中断线4
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;//使能中断线
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;//配置为外部中断模式
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;//选择下降沿触发
	EXTI_Init(&EXTI_InitStructure);//初始化EXTI
	EXTI_InitStructure.EXTI_Line=EXTI_Line5;//配置中断线5
	EXTI_Init(&EXTI_InitStructure);//初始化EXTI
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选择NVIC分组为2
	
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel=EXTI4_IRQn;//配置NVIC通道4
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能NVIC通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	NVIC_InitStructure.NVIC_IRQChannel=EXTI9_5_IRQn;//配置NVIC通道9_5
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级
	NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	
	DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);//打通SPI1到NVIC的接收传输完成中断通道
		
	NVIC_InitStructure.NVIC_IRQChannel=DMA2_Stream2_IRQn;//选择SPI1接收DMA传输完成中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//使能中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;//抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;//响应优先级为1
	NVIC_Init(&NVIC_InitStructure);//初始化SPI1接收DMA传输完成中断的NVIC
	
	/*===============使能===============*/
	DMA_Cmd(DMA2_Stream3,DISABLE);//失能DMA2的数据流3(SPI1发送DMA)
	DMA_Cmd(DMA2_Stream2,DISABLE);//失能DMA2的数据流2(SPI1接收DMA)
	SPI_Cmd(SPI1,ENABLE);//使能SPI1

	/*===============检测加速度计和陀螺仪ID并配置寄存器===============*/
	while(1)
	{
		if(BMI088_SPI_AccelReadRegister(BMI088_ACC_CHIP_ID)==BMI088_ACC_Address && BMI088_SPI_GyroReadRegister(BMI088_GYRO_CHIP_ID)==BMI088_GYRO_Address)//设备连接正常,加速度计和陀螺仪Who Am I正确
		{
			NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;//关闭NVIC通道
			NVIC_Init(&NVIC_InitStructure);//初始化NVIC
			
			BMI088_SPI_AccelWriteRegister(BMI088_ACC_SOFTRESET,0xB6);BMI088_Delay_us(1000);//加速度计软重启,耗时1ms
			BMI088_SPI_GyroWriteRegister(BMI088_GYRO_SOFTRESET,0xB6);BMI088_Delay_us(30000);//陀螺仪软重启,耗时30ms
			
			BMI088_SPI_AccelReadRegister(BMI088_ACC_CHIP_ID);//虚拟SPI读取,促使从机BMI088的加速度计进入SPI模式(软重启之后加速度计回到I2C模式,需要虚拟SPI读取切换到SPI模式)
			
			BMI088_SPI_AccelWriteRegister(BMI088_ACC_PWR_CTRL,0x04);BMI088_Delay_us(300);//打开加速度计
			BMI088_SPI_AccelWriteRegister(BMI088_ACC_PWR_CONF,0x00);BMI088_Delay_us(300);//加速度计正常工作(不挂起)
			BMI088_SPI_AccelWriteRegister(BMI088_ACC_CONF,0xAB);BMI088_Delay_us(300);//配置加速度计正常低通滤波带宽,800Hz输出
			BMI088_SPI_AccelWriteRegister(BMI088_ACC_RANGE,0x00);BMI088_Delay_us(300);//配置加速度计量程±3g
			BMI088_SPI_AccelWriteRegister(BMI088_INT1_IO_CONF,0x08);BMI088_Delay_us(300);//配置加速度计INT1推挽输出,低电平有效(下降沿)
			BMI088_SPI_AccelWriteRegister(BMI088_INT1_INT2_MAP_DATA,0x04);BMI088_Delay_us(300);//配置INT1中断映射为数据就绪中断
			
			BMI088_SPI_GyroWriteRegister(BMI088_GYRO_RANGE,0x00);BMI088_Delay_us(300);//配置陀螺仪量程±2000°/s
			BMI088_SPI_GyroWriteRegister(BMI088_GYRO_BANDWIDTH,0x82);BMI088_Delay_us(300);//配置陀螺仪滤波器带宽116Hz,1000Hz输出(写此寄存器,写入数据最高位需要为1)
			BMI088_SPI_GyroWriteRegister(BMI088_GYRO_LPM1,0x00);BMI088_Delay_us(300);//配置陀螺仪主电源模式为正常模式
			BMI088_SPI_GyroWriteRegister(BMI088_GYRO_INT_CTRL,0x80);BMI088_Delay_us(300);//使能陀螺仪数据就绪中断
			BMI088_SPI_GyroWriteRegister(BMI088_INT3_INT4_IO_CONF,0x00);BMI088_Delay_us(300);//配置陀螺仪INT3推挽输出,低电平有效(下降沿)
			BMI088_SPI_GyroWriteRegister(BMI088_INT3_INT4_IO_MAP,0x01);BMI088_Delay_us(300);//配置INT3中断映射为数据就绪中断
			
			BMI088_FirstDMA_Flag=1;//准备首次启动DMA
			break;
		}
		else//设备异常
		{
			NVIC_InitStructure.NVIC_IRQChannelCmd=DISABLE;//关闭NVIC通道
			NVIC_Init(&NVIC_InitStructure);//初始化NVIC
			Warming_BMI088LinkError();//BMI088连接异常报警
			BMI088_Delay_us(25000);
		}
	}
	
	while(BMI088_DataStartFlag==0);//等待数据开始接收
}

/*
 *函数简介:BMI088处理加速度计数据
 *参数说明:无
 *返回类型:无
 *备注:本函数用于SPI接收DMA传输完成之后,数据来源为BMI088的SPI接收DMA存储器数组BMI088_SPI_DMAReceive
 *备注:加速度a(m2/s)=a_raw(g)*9.8=(a_raw/32768*Range)*9.8
 *					 =(a_raw/32768*3)*9.8
 *					 =a_raw*0.0008974358974
 */
void BMI088_ProcessAccelData(void)
{
	BMI088_RawAccelData[0]=(int16_t)((uint16_t)BMI088_SPI_DMAReceive[3]<<8|BMI088_SPI_DMAReceive[2]);//拼接为原始数据
	BMI088_RawAccelData[1]=(int16_t)((uint16_t)BMI088_SPI_DMAReceive[5]<<8|BMI088_SPI_DMAReceive[4]);
	BMI088_RawAccelData[2]=(int16_t)((uint16_t)BMI088_SPI_DMAReceive[7]<<8|BMI088_SPI_DMAReceive[6]);
	
	BMI088_Accel[0]=BMI088_RawAccelData[0]*0.0008974358974f*1.02194846f;
	BMI088_Accel[1]=BMI088_RawAccelData[1]*0.0008974358974f*1.02194846f;
	BMI088_Accel[2]=BMI088_RawAccelData[2]*0.0008974358974f*1.02194846f;
//	BMI088_Accel[0]=BMI088_RawAccelData[0]/32768.0f*9.8f*3.0f;
//	BMI088_Accel[1]=BMI088_RawAccelData[1]/32768.0f*9.8f*3.0f;
//	BMI088_Accel[2]=BMI088_RawAccelData[2]/32768.0f*9.8f*3.0f;
}

/*
 *函数简介:BMI088处理温度数据
 *参数说明:无
 *返回类型:无
 *备注:本函数用于SPI接收DMA传输完成之后,数据来源为BMI088的SPI接收DMA存储器数组BMI088_SPI_DMAReceive
 *备注:温度t(℃)=t_raw*0.125+23
 *备注:温度数据是11bits数据
 */
void BMI088_ProcessTemperatureData(void)
{
	uint16_t unsignedTemperatureData=((uint16_t)BMI088_SPI_DMAReceive[2]<<3) | (BMI088_SPI_DMAReceive[3]>>5);//拼接为原始数据
	
	if(unsignedTemperatureData>1023)BMI088_RawTemperatureData=unsignedTemperatureData-2048;//对11bits数据进行正负数处理
	else BMI088_RawTemperatureData=unsignedTemperatureData;
	
	BMI088_Temperature=BMI088_RawTemperatureData*0.125f+23.0f;
}

/*
 *函数简介:BMI088处理陀螺仪数据
 *参数说明:无
 *返回类型:无
 *备注:本函数用于SPI接收DMA传输完成之后,数据来源为BMI088的SPI接收DMA存储器数组BMI088_SPI_DMAReceive
 *备注:角速度w(rad/s)=w_raw(°/s)/180*Π=(w_raw/32768*Range)/180*Π
 *					  =(w_raw/32768*2000)/180*Π
 *					  =w_raw*0.00106526443603169529841533860381
 */
void BMI088_ProcessGyroData(void)
{
	BMI088_RawGyroData[0]=(int16_t)((uint16_t)BMI088_SPI_DMAReceive[2]<<8|BMI088_SPI_DMAReceive[1]);//拼接为原始数据
	BMI088_RawGyroData[1]=(int16_t)((uint16_t)BMI088_SPI_DMAReceive[4]<<8|BMI088_SPI_DMAReceive[3]);
	BMI088_RawGyroData[2]=(int16_t)((uint16_t)BMI088_SPI_DMAReceive[6]<<8|BMI088_SPI_DMAReceive[5]);
	
	BMI088_Gyro[0]=BMI088_RawGyroData[0]*0.00106526443603169529841533860381f-0.00584748201f;
	BMI088_Gyro[1]=BMI088_RawGyroData[1]*0.00106526443603169529841533860381f-0.00296888314f;
	BMI088_Gyro[2]=BMI088_RawGyroData[2]*0.00106526443603169529841533860381f+0.000562281464f;
//	BMI088_Gyro[0]=BMI088_RawGyroData[0]/32768.0f*2000.0f/180.0f*3.141592653589793238462643383279f;
//	BMI088_Gyro[1]=BMI088_RawGyroData[1]/32768.0f*2000.0f/180.0f*3.141592653589793238462643383279f;
//	BMI088_Gyro[2]=BMI088_RawGyroData[2]/32768.0f*2000.0f/180.0f*3.141592653589793238462643383279f;
}

/*
 *函数简介:BMI088检测并关闭DMA
 *参数说明:无
 *返回类型:无
 *备注:用在外部中断中用来等待SPI传输结束后配合BMI088_OpenDMA函数重新启动DMA
 */
void BMI088_CheckAndCloseDMA(void)
{
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY)==SET);//等待总线空闲
	while(DMA_GetFlagStatus(DMA2_Stream3,DMA_FLAG_TCIF3)==RESET);//判断发送完成
	DMA_ClearFlag(DMA2_Stream3,DMA_FLAG_TCIF3);//清除发送完成标志位
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_BSY)==SET);//等待总线空闲
	DMA_Cmd(DMA2_Stream3,DISABLE);//失能DMA2的数据流2
	DMA_Cmd(DMA2_Stream2,DISABLE);//失能DMA2的数据流3
}

/*
 *函数简介:BMI088开启DMA
 *参数说明:DMA传输数据个数
 *返回类型:无
 *备注:SPI收发DMA开启后会立刻开始发送接收,请在使用函数前配置BMI088的SPI发送DMA存储器数组BMI088_SPI_DMASend
 */
void BMI088_OpenDMA(uint8_t DMA_BufferSize)
{
	while(DMA_GetCmdStatus(DMA2_Stream2)!=DISABLE);//检测DMA2的数据流2为可配置状态
	DMA_SetCurrDataCounter(DMA2_Stream2,DMA_BufferSize);//恢复传输计数器的值
	DMA_Cmd(DMA2_Stream2,ENABLE);//使能DMA2的数据流2
	while(DMA_GetCmdStatus(DMA2_Stream3)!=DISABLE);//检测DMA2的数据流3为可配置状态
	DMA_SetCurrDataCounter(DMA2_Stream3,DMA_BufferSize);//恢复传输计数器的值
	DMA_Cmd(DMA2_Stream3,ENABLE);//使能DMA2的数据流3
}

/*
 *函数简介:BMI088加速度计外部中断中断函数
 *参数说明:无
 *返回类型:无
 *备注:在中断函数中启动DMA收发以实现减少CPU负担
 */
void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)==SET)//检测BMI088加速度计外部中断触发(即检测EXTI通道4中断触发)
	{
		EXTI_ClearITPendingBit(EXTI_Line4);//清除标志位
		
		if(BMI088_FirstDMA_Flag==0)//非第一次启动DMA
		{
			BMI088_CheckAndCloseDMA();//关闭DMA准备配置
			
			BMI088_Accel_Start();//片选加速度计
			BMI088_DMAAccGyroSelect=1;//DMA为加速度计数据接收
			BMI088_SPI_DMASend[0]=BMI088_ACC_AccelDataStart | 0x80;//配置SPI的发送数据的地址(读数据)
			
			BMI088_OpenDMA(8);//开启DMA转运8个数据
		}
		else if(BMI088_FirstDMA_Flag==1)//第一次启动DMA
		{
			BMI088_FirstDMA_Flag=0;
			
			BMI088_Accel_Start();//片选加速度计
			BMI088_DMAAccGyroSelect=1;//DMA为加速度计数据接收
			BMI088_SPI_DMASend[0]=BMI088_ACC_AccelDataStart | 0x80;//配置SPI的发送数据的地址(读数据)
			
			DMA_Cmd(DMA2_Stream2,ENABLE);//使能DMA2的数据流2
			DMA_Cmd(DMA2_Stream3,ENABLE);//使能DMA2的数据流3
		}
	}
}

/*
 *函数简介:BMI088陀螺仪外部中断中断函数
 *参数说明:无
 *返回类型:无
 *备注:在中断函数中启动DMA收发以实现减少CPU负担
 *备注:第一次DMA转运8个数据,从第二次DMA开始转运7个数据
 */
void EXTI9_5_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line5)==SET)//检测BMI088陀螺仪外部中断触发(即检测EXTI通道5中断触发)
	{
		EXTI_ClearITPendingBit(EXTI_Line5);//清除标志位
		
		if(BMI088_FirstDMA_Flag==0)//非第一次启动DMA
		{
			BMI088_CheckAndCloseDMA();//关闭DMA准备配置
			
			BMI088_Gyro_Start();//片选陀螺仪
			BMI088_DMAAccGyroSelect=2;//DMA为陀螺仪数据接收
			BMI088_SPI_DMASend[0]=BMI088_GYRO_GyroDataStart | 0x80;//配置SPI的发送数据的地址(读数据)
			
			BMI088_OpenDMA(7);//开启DMA转运7个数据
		}
		else if(BMI088_FirstDMA_Flag==1)//第一次启动DMA
		{
			BMI088_FirstDMA_Flag=0;	
			
			BMI088_Gyro_Start();//片选陀螺仪
			BMI088_DMAAccGyroSelect=2;//DMA为陀螺仪数据接收
			BMI088_SPI_DMASend[0]=BMI088_GYRO_GyroDataStart | 0x80;//配置SPI的发送数据的地址(读数据)
			
			DMA_Cmd(DMA2_Stream2,ENABLE);//使能DMA2的数据流2
			DMA_Cmd(DMA2_Stream3,ENABLE);//使能DMA2的数据流3
		}
	}
}

/*
 *函数简介:SPI1接收DMA传输完成中断中断函数
 *参数说明:无
 *返回类型:无
 *备注:在此函数中会处理接收数据获取BMI088的三轴加速度数据,三轴角速度数据和温度
 *备注:在此函数中如果接收的是加速度计数据会再开启一个DMA去接收温度数据
 */
void DMA2_Stream2_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2)==SET)//检测SPI1接收DMA传输完成中断触发
	{
		static uint8_t BMI088_ReceiveTEMPFlag=0;//BMI088的DMA接收温度数据标志位(0-未在接收温度数据,1-正在接收温度数据)
		BMI088_DataStartFlag=1;//数据开始接收
		DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);//清除标志位
		
		if(BMI088_ReceiveTEMPFlag==1)//正在接收温度数据
		{
			BMI088_Accel_Stop();//结束加速度计片选
			BMI088_ProcessTemperatureData();//处理温度数据
			BMI088_ReceiveTEMPFlag=0;
		}
		else//接收加速度或角速度数据
		{
			if(BMI088_DMAAccGyroSelect==1)//接收加速度计数据
			{
				BMI088_Accel_Stop();//结束加速度计片选
				BMI088_ProcessAccelData();//处理加速度数据
				
				BMI088_ReceiveTEMPFlag=1;//开始接收温度数据
				BMI088_CheckAndCloseDMA();//关闭DMA准备配置
				
				BMI088_Accel_Start();//片选加速度计
				BMI088_SPI_DMASend[0]=BMI088_ACC_TEMP_MSB | 0x80;//配置SPI的发送数据的地址(读数据)
				
				BMI088_OpenDMA(4);//开启DMA转运4个数据
			}
			else if(BMI088_DMAAccGyroSelect==2)//接收陀螺仪数据
			{
				BMI088_Gyro_Stop();//结束陀螺仪片选
				BMI088_ProcessGyroData();//处理陀螺仪数据
			}
		}
	}
}
