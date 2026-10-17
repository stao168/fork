#include "HI04.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"

#define HI04_BAUDRATE       921600U
#define HI04_RX_BUFFER_SIZE 2048U

/*
 * USART1_RX: PB7
 * USART1_TX: PA9
 * RX DMA: DMA2 Stream5 Channel4
 *
 * 缓冲区放普通 SRAM，不要放 CCM。
 */
volatile uint8_t hi04_rx_buffer[HI04_RX_BUFFER_SIZE];

/* 后续读取 DMA 数据时需要使用的计数 */
volatile uint32_t hi04_dma_wraps;
uint32_t hi04_rx_consumed;

void HI04_Init(void)
{
	/* 开启时钟 */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
	/* GPIO：PA9 TX、PB7 RX，均使用 AF7 */
	GPIO_InitTypeDef gpio;
	GPIO_StructInit(&gpio);
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	gpio.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &gpio);
	gpio.GPIO_Pin = GPIO_Pin_7;
	GPIO_Init(GPIOB, &gpio);
	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
	/* USART1：921600，8N1，无硬件流控 */
	USART_InitTypeDef usart;
	USART_StructInit(&usart);
	usart.USART_BaudRate = HI04_BAUDRATE;
	usart.USART_WordLength = USART_WordLength_8b;
	usart.USART_StopBits = USART_StopBits_1;
	usart.USART_Parity = USART_Parity_No;
	usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &usart);
	
	/* DMA 接收，不开启 RXNE 逐字节中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
	
	/* DMA2 Stream5 Channel4：循环接收 */
	DMA_InitTypeDef dma;
	DMA_StructInit(&dma);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;
	dma.DMA_Memory0BaseAddr = (uint32_t)hi04_rx_buffer;
	dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize = HI04_RX_BUFFER_SIZE;
	
	dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	
	dma.DMA_Mode = DMA_Mode_Circular;
	dma.DMA_Priority = DMA_Priority_High;
	dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA2_Stream5, &dma);
	
	/* 6. 清除标志，准备软件计数 */
	DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_FEIF5 | DMA_FLAG_DMEIF5 | DMA_FLAG_TEIF5 | DMA_FLAG_HTIF5 | DMA_FLAG_TCIF5);
	
	volatile uint32_t dummy;
	
	hi04_dma_wraps = 0U;
	hi04_rx_consumed = 0U;
	
	NVIC_InitTypeDef nvic;
	NVIC_ClearPendingIRQ(USART1_IRQn);
	NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);
	
	/*
	* DMA 传输完成中断
	* 每收满一个缓冲区记录一次回绕。
	* 沿用工程已有的 NVIC 分组，不在这里重新分组。
	*/
	DMA_ITConfig(DMA2_Stream5, DMA_IT_TC, ENABLE);
	
	nvic.NVIC_IRQChannel = DMA2_Stream5_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 1;
	nvic.NVIC_IRQChannelSubPriority = 2;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
	
	/* 启动前清除旧的 UART 接收状态 */
	dummy = USART1->SR;
	dummy = USART1->DR;
	(void)dummy;
	
	/* 8. 先准备 DMA，再启动 USART 接收 */
	DMA_Cmd(DMA2_Stream5, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	USART_Cmd(USART1, ENABLE);
}

/*
 * 放在 HI04.c 即可，中断函数不要求必须放 stm32f4xx_it.c。
 * 整个工程只能有一个同名实现。
 */
void DMA2_Stream5_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5) != RESET)
	{
		DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
		hi04_dma_wraps++;
	}
}
