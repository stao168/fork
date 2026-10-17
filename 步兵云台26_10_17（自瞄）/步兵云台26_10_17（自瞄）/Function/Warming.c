#include "stm32f4xx.h"                  // Device header
#include "stm32f4xx_conf.h"
#include "LED.h"
#include "Buzzer.h"
#include "CAN.h"
#include "Delay.h"
#include "LinkCheck.h"
#include "GM6020.h"
#include "M3508.h"
#include "Laser.h"
#include "DM_J4310.h"
#include "DM_J4340.h"
#include "Warming.h"

/*====================	 报警列表	====================
	遥控器未连接	················1s里红灯连闪两下
	遥控器数据错误报警	············红灯常亮
	CAN总线设备连接异常·············2s内蜂鸣器以高音6响n下,n为连接异常的设备在ID列表的索引(LinkCheck.h文件中LinkCheck_ErrorID变量)
	IST8310连接错误	················绿灯以1s为周期闪烁
	BMI088连接错误	················1s里绿灯连闪两下
	陀螺仪温度过高报警	············绿灯常亮
	电机报警状态	················电机静止
	FLASH清除警告	················白灯常亮
  ======================================================*/

uint8_t Can_ErrorDevice;
uint8_t Can_Error_Flag = 0, flag_Warning_Stop = 0;
uint16_t Can_Test[DEVICES] = {0};

/*
 *函数简介:报警初始化
 *参数说明:无
 *返回类型:无
 *备注:报警功能用于各种错误的提示
 *备注:报警有两种方式-LED和蜂鸣器
 */
void Warming_Init(void)
{
	LED_Init();
	Buzzer_Init();
}

/*
 *函数简介:报警LED清理
 *参数说明:无
 *返回类型:无
 *备注:关闭所有灯
 */
void Warming_LEDClean(void)
{
	LED_BOFF();LED_GOFF();LED_ROFF();
}

/*
 *函数简介:报警蜂鸣器清理
 *参数说明:无
 *返回类型:无
 *备注:关闭所有蜂鸣器
 */
void Warming_BuzzerClean(void)
{
	Buzzer_ON(P);
}

/*
 *函数简介:遥控器未连接报警
 *参数说明:无
 *返回类型:无
 *备注:遥控器连接检测TIM7定时更新中断调用,定时25ms
 *报警现象:1s里红灯连闪两下
 */
void Warming_RemoteNoCheck(void)
{
	static uint8_t Counter=0;
	Counter++;
	
	if(Counter==1)LED_RON();
	else if(Counter==5)LED_ROFF();
	else if(Counter==13)LED_RON();
	else if(Counter==17)LED_ROFF();
	else if(Counter==40)Counter=0;
}

/*
 *函数简介:遥控器数据错误报警
 *参数说明:无
 *返回类型:无
 *报警现象:红灯常量
 */
void Warming_RemoteDataERROR(void)
{
	LED_RON();
}

/*
 *函数简介:CAN总线设备连接异常报警
 *参数说明:无
 *返回类型:无
 *备注:闭环控制TIM6定时更新中断调用,定时2ms
 *报警现象:2s内蜂鸣器以高音6响n下,n为连接异常的设备在ID列表的索引(LinkCheck.h文件中LinkCheck_ErrorID变量)
 */
void Warming_LinkError(void)
{
	static uint8_t i = 0;
	static uint16_t Counter = 0;
	Counter ++;
	
	if(i < Can_ErrorDevice + 1)//高音6响 Can_ErrorDevice + 1 下
	{
		if(Counter == 100 * i + 1) Buzzer_ON(H6);
		else if(Counter == 100 * i + 51) Buzzer_ON(P);
		else if(Counter == 100 * i + 100) i ++;
	}
	if(Counter == 1000){Counter = 0; i = 0;}
}

void CAN_Check(void)
{
    uint8_t offline_count = 0;        // 局部变量：记录当前掉线总数
    uint8_t highest_offline_id = 0;   // 局部变量：记录最大掉线设备号

    for(uint8_t i = 0; i < DEVICES; i++)
    {
        Can_Test[i]++; // 时间切片累加
        
        if(Can_Test[i] >= 10)
        {
            Can_Test[i] = 10; 
            
            offline_count++;
            highest_offline_id = i; 
        }
    }

    if(offline_count > 0)
    {
        Can_Error_Flag = 1;
        Can_ErrorDevice = highest_offline_id; 
    }
    else
    {
        Can_Error_Flag = 0;
        Warming_BuzzerClean();
        
    }
}

/*
 *函数简介:IST8310连接错误
 *参数说明:无
 *返回类型:无
 *备注:IST8310初始化调用,周期25ms
 *报警现象:绿灯以1s为周期闪烁
 */
void Warming_IST8310LinkError(void)
{
	static uint8_t Counter=0;
	Counter++;
	
	if(Counter==1)LED_GON();
	else if(Counter==5)LED_GOFF();
	else if(Counter==40)Counter=0;
}

/*
 *函数简介:BMI088连接错误
 *参数说明:无
 *返回类型:无
 *备注:BMI088初始化调用,周期25ms
 *报警现象:1s里绿灯连闪两下
 */
void Warming_BMI088LinkError(void)
{
	static uint8_t Counter=0;
	Counter++;
	
	if(Counter==1)LED_GON();
	else if(Counter==5)LED_GOFF();
	else if(Counter==13)LED_GON();
	else if(Counter==17)LED_GOFF();
	else if(Counter==40)Counter=0;
}

/*
 *函数简介:陀螺仪温度过高报警并停止加热
 *参数说明:无
 *返回类型:无
 *备注:IMU恒温控制调用,温度警戒50℃
 *报警现象:绿灯常亮
 */
void Warming_IMUTemperatureTooHigh(void)
{
	LED_GON();
	TIM_SetCompare1(TIM10,0);
}

/*
 *函数简介:电机报警状态
 *参数说明:无
 *返回类型:无
 *报警现象:电机静止
 */
void Warming_MotorControl(void)
{
	DM_J4310_Set(DM_J4310_2,0);
	GM6020_CAN2SetLIDCurrent(0, 0, 0, 0);
	M3508_CAN1SetLIDCurrent(0, 0, 0, 0);
}

/*
 *函数简介:FLASH清除警告
 *参数说明:无
 *返回类型:无
 *报警现象:白灯常亮
 */
void Warming_EraseFLASH(void)
{
	LED_RON();
	LED_GON();
	LED_BON();
}
