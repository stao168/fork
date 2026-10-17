#include "HI04.h"
#include "stm32f4xx.h"
#include "stm32f4xx_conf.h"
#include <string.h>
#include "hipnuc_dec.h"
#include <math.h>
#include <float.h>

volatile HI04_Data_t HI04_Data;
volatile HI04_Stats_t HI04_Stats;

/* 解析器需要一直保留，不能定义成 Poll 内的普通局部变量 */
static hipnuc_raw_t hi04_decoder;

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
	
	memset(&hi04_decoder, 0, sizeof(hi04_decoder));
	memset((void *)&HI04_Data, 0, sizeof(HI04_Data));
	memset((void *)&HI04_Stats, 0, sizeof(HI04_Stats));	
	
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
 * 接收发生缺口后，丢弃未完成帧。
 * 保留解析器中的累计统计值。
 */
static void HI04_ResetPartialFrame(void)
{
    hi04_decoder.nbyte = 0;
    hi04_decoder.len = 0;
    hi04_decoder.buf[0] = 0U;
    hi04_decoder.buf[1] = 0U;

    hi04_decoder.hi91.tag = 0U;
    hi04_decoder.hi81.tag = 0U;
    hi04_decoder.hi83.tag = 0U;
}

/*
 * 获取 DMA 累计写入的字节数。
 *
 * 短暂屏蔽中断，防止与 DMA 回绕中断同时修改 wraps。
 * DMA 本身不会因此停止。
 */
static uint32_t HI04_DMAProduced(void)
{
    uint32_t irq_mask;
    uint32_t produced;
    uint16_t remaining;

    irq_mask = __get_PRIMASK();
    __disable_irq();

    do
    {
        /*
         * DMA 已回绕，但中断可能还没执行。
         * 在这里补记一次，并清除标志，避免 ISR 重复计数。
         */
        if (DMA_GetFlagStatus(DMA2_Stream5,
                              DMA_FLAG_TCIF5) != RESET)
        {
            DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5);
            hi04_dma_wraps++;
        }

        remaining = DMA_GetCurrDataCounter(DMA2_Stream5);

        /*
         * 如果读计数时恰好回绕，则重新取快照。
         */
    } while (remaining == 0U ||
             DMA_GetFlagStatus(DMA2_Stream5,
                               DMA_FLAG_TCIF5) != RESET);

    produced = hi04_dma_wraps * HI04_RX_BUFFER_SIZE
             + HI04_RX_BUFFER_SIZE - remaining;

    __DMB();
    __set_PRIMASK(irq_mask);

    return produced;
}

/*
 * 返回 1：检测到硬件错误，已重新启动接收。
 * 返回 0：未发现错误。
 *
 * 从主循环调用。
 */
static int HI04_CheckHardwareError(void)
{
    uint32_t status;
    volatile uint32_t dummy;
    int dma_error;

    status = USART1->SR;

    dma_error =
        (DMA_GetFlagStatus(DMA2_Stream5,
                           DMA_FLAG_TEIF5) != RESET) ||
        (DMA_GetFlagStatus(DMA2_Stream5,
                           DMA_FLAG_DMEIF5) != RESET) ||
        (DMA_GetFlagStatus(DMA2_Stream5,
                           DMA_FLAG_FEIF5) != RESET);

    if ((status & (USART_SR_ORE |
                   USART_SR_FE |
                   USART_SR_NE |
                   USART_SR_PE)) == 0U &&
        !dma_error)
    {
        return 0;
    }

    HI04_Stats.hardware_errors++;

    NVIC_DisableIRQ(DMA2_Stream5_IRQn);

    /* 先停止 DMA 接收请求，再停止数据流 */
    USART_DMACmd(USART1, USART_DMAReq_Rx, DISABLE);
    DMA_Cmd(DMA2_Stream5, DISABLE);

    while (DMA_GetCmdStatus(DMA2_Stream5) != DISABLE)
    {
    }

    /*
     * DMA 已停止，CPU 此时读取 DR 不会与 DMA 争抢。
     * 先读 SR、再读 DR，清除 UART 接收错误。
     */
    dummy = USART1->SR;
    dummy = USART1->DR;
    (void)dummy;

    DMA_ClearFlag(DMA2_Stream5,
                  DMA_FLAG_FEIF5 |
                  DMA_FLAG_DMEIF5 |
                  DMA_FLAG_TEIF5 |
                  DMA_FLAG_HTIF5 |
                  DMA_FLAG_TCIF5);

    DMA_SetCurrDataCounter(DMA2_Stream5,
                           HI04_RX_BUFFER_SIZE);

    hi04_dma_wraps = 0U;
    hi04_rx_consumed = 0U;

    HI04_ResetPartialFrame();
    HI04_Data.received = 0U;

    NVIC_ClearPendingIRQ(DMA2_Stream5_IRQn);

    DMA_Cmd(DMA2_Stream5, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    NVIC_EnableIRQ(DMA2_Stream5_IRQn);

    return 1;
}

/*
 * 返回 1：读到一个字节。
 * 返回 0：暂无数据。
 * 返回 -1：检测到缓冲覆盖，已丢弃受影响的数据。
 */
static int HI04_ReadByte(uint8_t *byte)
{
    uint32_t produced;
    uint32_t available;
    uint8_t value;

    produced = HI04_DMAProduced();
    available = produced - hi04_rx_consumed;

    if (available == 0U)
    {
        return 0;
    }

    /*
     * 达到缓冲容量时保守丢弃，避免读取正在被覆盖的位置。
     */
    if (available >= HI04_RX_BUFFER_SIZE)
    {
        hi04_rx_consumed = produced;
        HI04_Stats.buffer_overruns++;
        return -1;
    }

    value = hi04_rx_buffer[
        hi04_rx_consumed % HI04_RX_BUFFER_SIZE];

    __DMB();

    /*
     * 复制字节期间，可能被其他中断打断，
     * 再检查一次 DMA 是否已经追上当前读取位置。
     */
    produced = HI04_DMAProduced();

    if (produced - hi04_rx_consumed >= HI04_RX_BUFFER_SIZE)
    {
        hi04_rx_consumed = produced;
        HI04_Stats.buffer_overruns++;
        return -1;
    }

    hi04_rx_consumed++;
    *byte = value;

    return 1;
}

void HI04_Poll(void)
{
    uint8_t byte;
    int read_result;
    int parse_result;
    unsigned int i;
    unsigned int budget;

    if (HI04_CheckHardwareError())
    {
        return;
    }

    /*
     * 限制单次调用工作量，避免长期占用主循环。
     * 正常每轮只有几十到几百字节。
     */
    budget = HI04_RX_BUFFER_SIZE;

    while (budget > 0U)
    {
        budget--;

        read_result = HI04_ReadByte(&byte);

        if (read_result == 0)
        {
            break;
        }

        if (read_result < 0)
        {
            HI04_ResetPartialFrame();
            HI04_Data.received = 0U;
            continue;
        }

        parse_result = hipnuc_input(&hi04_decoder, byte);

        HI04_Stats.crc_errors =
            hi04_decoder.crc_error_count;

        HI04_Stats.invalid_frames =
            hi04_decoder.invalid_count;

        /*
         * 只有返回值 > 0，且是 HI91，
         * 才能读取这一帧的测量数据。
         */
        if (parse_result > 0 &&
            hi04_decoder.hi91.tag == HIPNUC_ID_HI91)
        {
            HI04_Data.roll_deg =
                hi04_decoder.hi91.roll;

            HI04_Data.pitch_deg =
                hi04_decoder.hi91.pitch;

            HI04_Data.yaw_deg =
                hi04_decoder.hi91.yaw;

            for (i = 0U; i < 3U; i++)
            {
                HI04_Data.acc_g[i] =
                    hi04_decoder.hi91.acc[i];

                /* HI91 原始角速度为 deg/s */
                HI04_Data.gyro_rad_s[i] =
                    hi04_decoder.hi91.gyr[i] * HIPNUC_DEG2RAD;
            }

            uint32_t primask = __get_PRIMASK();
            __disable_irq();

            for (i = 0U; i < 4U; i++)
            {
                HI04_Data.quat[i] =
                    hi04_decoder.hi91.quat[i];
            }

            __DMB();
            __set_PRIMASK(primask);

            HI04_Data.device_time_ms =
                hi04_decoder.hi91.system_time;

            HI04_Data.frame_count++;
            HI04_Data.received = 1U;
        }

        /*
         * CRC 或格式错误时，解析器负责重新寻找帧头。
         * 继续喂字节，不要在这里反复 memset 解析器。
         */
    }
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

#define HI04_VIS_PI      3.14159265358979323846f
#define HI04_VIS_HALF_PI (HI04_VIS_PI * 0.5f)
#define HI04_VIS_TWO_PI  (HI04_VIS_PI * 2.0f)

/* 不依赖编译器是否提供 isfinite 宏 */
static int HI04_VisualFloatFinite(float value)
{
	return (value >= -FLT_MAX) && (value <= FLT_MAX);
}

/* 与视觉 limit_rad() 一致：(-PI, PI] */
static float HI04_VisualLimitRad(float angle)
{
	while (angle > HI04_VIS_PI) angle -= HI04_VIS_TWO_PI;
	while (angle <= -HI04_VIS_PI) angle += HI04_VIS_TWO_PI;
	return angle;
}

int HI04_QuaternionToVisual(const float quat[4], HI04_VisualAttitude_t *result)
{
	float w, x, y, z;
	float scale, norm;
	float a, b, c, d;
	float n2, cos_middle, middle;
	float half_sum, half_diff;
	float angle0, angle2;
	int safe1, safe2;
	
	if (quat == 0 || result == 0) return 0;
	
	w = quat[0];
	x = quat[1];
	y = quat[2];
	z = quat[3];
	
	if (!HI04_VisualFloatFinite(w) || !HI04_VisualFloatFinite(x) || !HI04_VisualFloatFinite(y) || !HI04_VisualFloatFinite(z)) return 0;
	
	/*
	* 先缩放再归一化，避免异常大数在平方时溢出。
	* 正常 HI04 四元数的模长应接近 1。
	*/
	scale = fabsf(w);
	if (fabsf(x) > scale) scale = fabsf(x);
	if (fabsf(y) > scale) scale = fabsf(y);
	if (fabsf(z) > scale) scale = fabsf(z);
	
	if (scale == 0.0f) return 0;
	
	w /= scale;
	x /= scale;
	y /= scale;
	z /= scale;
	
	norm = sqrtf(w*w + x*x + y*y + z*z);
	
	/* 与视觉一样，拒绝原始模长 <= 1e-6 的四元数 */
	if (scale <= 1.0e-6f / norm) return 0;
	
	w /= norm;
	x /= norm;
	y /= norm;
	z /= norm;
	
	/*
	* 对应视觉：
	* eulers(q, 2, 1, 0, false)
	*
	* 内部交换轴后 i=0, j=1, k=2，sign=+1。
	*/
	a = w - y;
	b = x + z;
	c = y + w;
	d = z - x;
	
	n2 = a*a + b*b + c*c + d*d;
	
	cos_middle = 2.0f * (a*a + b*b) / n2 - 1.0f;
	
	/* 避免浮点舍入使 acosf 输入超出定义域 */
	if (cos_middle >  1.0f) cos_middle =  1.0f;
	if (cos_middle < -1.0f) cos_middle = -1.0f;
	
	middle = acosf(cos_middle);
	
	half_sum  = atan2f(b, a);
	half_diff = atan2f(-d, c);
	
	safe1 = fabsf(middle) >= 1.0e-7f;
	safe2 = fabsf(middle - HI04_VIS_PI) >= 1.0e-7f;
	
	if (safe1 && safe2)
	{
		angle0 = half_sum + half_diff;
		angle2 = half_sum - half_diff;
	}
	else
	{
		/* 与视觉内禀旋转的奇异姿态分支一致 */
		angle0 = 0.0f;
	
		if (!safe1) angle2 = 2.0f * half_sum;
		else angle2 = -2.0f * half_diff;
	}
	
	/* 对应视觉最后交换第 0、2 项 */
	result->yaw_rad   = HI04_VisualLimitRad(angle2);
	result->pitch_rad = middle - HI04_VIS_HALF_PI;
	result->roll_rad  = HI04_VisualLimitRad(angle0);
	
	return 1;
}
