/**
  ******************************************************************************
  * @file		PID.c
  * @brief		This is the PID lib of MAS Robot.
  * @author		zpclyn
  * @version	V1.1
  * @date		2025-1-28
  * @copyright	Copyright © 2025 zpclyn <br>
  *				All rights reserved.
  *
  * @par		修改日志
  * <table>
  * <tr><th>Data		<th>Version		<th>Author		<th>Description
  * <tr><td>2025/1/27	<td>1.0			<td>zpclyn		<td>创建初始版本
  * <tr><td>2025/1/28	<td>1.1			<td>zpclyn		<td>加入一些说明
  * </table>
  *
  * @attention	最终解释权归属山海机甲
  *
  */
  
/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include "PID.h"

/** @addtogroup MAS_Control
  * @{
  */

/** @addtogroup PID
  * @brief		This is the PID lib of MAS Robot.
  * @author		zpclyn
  * @{
  */

/* Private define ------------------------------------------------------------*/ 
#define ABS(x)	((x>0)?x:-x)///<绝对值函数

/* Function ------------------------------------------------------------------*/

/** @defgroup	PID_Improvement
  * @brief		This is the PID Improvement lib of MAS Robot.
  * @{
  */

/* PID Improvement Function ***************************************************/

/** @defgroup	PID_Proportion_Limit
  * 比例限幅
  * @warning	建议MaxOUT与输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
  * @{
  */
  
/**
  * @brief  		使能比例限幅
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		MaxOUT: 比例限幅值
  * @retval			None
  * @warning		建议MaxOUT与输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
  */
void PID_EnableProportionLimit(PID_Typedef *PID,float MaxOUT)
{
	(PID->Improvement).ProportionLimit.Status=1;
	(PID->Improvement).ProportionLimit.MaxOUT=MaxOUT;
}

/**
  * @brief  		失能比例限幅
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableProportionLimit(PID_Typedef *PID)
{
	(PID->Improvement).ProportionLimit.Status=0;
	(PID->Improvement).ProportionLimit.MaxOUT=0;
}

/**
  * @brief  		比例限幅设置参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		MaxOUT: 比例限幅值
  * @retval			None
  * @warning		建议MaxOUT与输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
  */
void PID_SetProportionLimit(PID_Typedef *PID,float MaxOUT)
{
	(PID->Improvement).ProportionLimit.MaxOUT=MaxOUT;
}

/**
  * @brief  		比例限幅
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_ProportionLimit(PID_Typedef *PID)
{
    if(PID->P_OUT>(PID->Improvement).ProportionLimit.MaxOUT)PID->P_OUT=(PID->Improvement).ProportionLimit.MaxOUT;
    if(PID->P_OUT<-((PID->Improvement).ProportionLimit.MaxOUT))PID->P_OUT=-((PID->Improvement).ProportionLimit.MaxOUT);
}

/**
  * @}
  */

/** @defgroup	PID_Trapezoid_Intergral
  * 梯形积分
  * @{
  */

/**
  * @brief  		使能梯形积分
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_EnableTrapezoidIntergral(PID_Typedef *PID)
{
	(PID->Improvement).TrapezoidIntergral.Status=1;
}	

/**
  * @brief  		失能梯形积分
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableTrapezoidIntergral(PID_Typedef *PID)
{
	(PID->Improvement).TrapezoidIntergral.Status=0;
}

/**
  * @brief  		梯形积分
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_TrapezoidIntergral(PID_Typedef *PID)
{
    PID->I_Term=PID->Ki*((PID->Ek+PID->Ek_1)/2.0f);
}

/**
  * @}
  */

/** @defgroup	PID_Integral_Separation
  * 积分分离
  * @attention	变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Changing_Integral_Rate
  * @{
  */

/**
  * @brief  		使能积分分离
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		MaxErr: 积分分离误差阈值
  * @retval			None
  * @attention		变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Changing_Integral_Rate
  */
void PID_EnableIntegralSeparation(PID_Typedef *PID,float MaxErr)
{
	(PID->Improvement).IntegralSeparation.Status=1;
	(PID->Improvement).IntegralSeparation.MaxErr=MaxErr;
}

/**
  * @brief  		失能积分分离
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableIntegralSeparation(PID_Typedef *PID)
{
	(PID->Improvement).IntegralSeparation.Status=0;
	(PID->Improvement).IntegralSeparation.MaxErr=0;
}

/**
  * @brief  		积分分离设置参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		MaxErr: 积分分离误差阈值
  * @retval			None
  * @attention		变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Changing_Integral_Rate
  */
void PID_SetIntegralSeparation(PID_Typedef *PID,float MaxErr)
{
	(PID->Improvement).IntegralSeparation.MaxErr=MaxErr;
}

/**
  * @brief  		积分分离
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_IntegralSeparation(PID_Typedef *PID)
{
	if(PID->Ek*PID->I_OUT>0)//增长趋势
	{
		if(ABS(PID->Ek)<=(PID->Improvement).IntegralSeparation.MaxErr)return;
		else PID->I_Term=0;
	}
}

/**
  * @}
  */

/** @defgroup	PID_Changing_Integral_Rate
  * 变速积分
  * @note		ITerm=Ki*Err*((A-abs(Err)+B)/A)	when B<|err|\<A+B
  * @attention	变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Integral_Separation
  * @{
  */

/**
  * @brief  		使能变速积分
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		ScalarA: 系数A
  * @param[in]		ScalarB: 系数B
  * @retval			None
  * @attention		变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Integral_Separation
  */
void PID_EnableChangingIntegralRate(PID_Typedef *PID,float ScalarA,float ScalarB)
{
	(PID->Improvement).ChangingIntegralRate.Status=1;
	(PID->Improvement).ChangingIntegralRate.ScalarA=ScalarA;
	(PID->Improvement).ChangingIntegralRate.ScalarB=ScalarB;
}

/**
  * @brief  		失能变速积分
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableChangingIntegralRate(PID_Typedef *PID)
{
	(PID->Improvement).ChangingIntegralRate.Status=0;
	(PID->Improvement).ChangingIntegralRate.ScalarA=0;
	(PID->Improvement).ChangingIntegralRate.ScalarB=0;
}

/**
  * @brief  		变速积分设置参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		ScalarA: 系数A
  * @param[in]		ScalarB: 系数B
  * @retval			None
  * @attention		变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Integral_Separation
  */
void PID_SetChangingIntegralRate(PID_Typedef *PID,float ScalarA,float ScalarB)
{
	(PID->Improvement).ChangingIntegralRate.ScalarA=ScalarA;
	(PID->Improvement).ChangingIntegralRate.ScalarB=ScalarB;
}

/**
  * @brief  		变速积分
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  * @note			ITerm=Ki*Err					when |err|<A <br>
  * 				ITerm=Ki*Err*((A-abs(Err)+B)/A)	when B<|err|<A+B <br>
  * 				ITerm=0							when A+B<|err|
  */
static void PID_ChangingIntegralRate(PID_Typedef *PID)
{
    if(PID->Ek*PID->I_OUT>0)//增长趋势
    {
		float A=(PID->Improvement).ChangingIntegralRate.ScalarA;
		float B=(PID->Improvement).ChangingIntegralRate.ScalarB;
        
        if(ABS(PID->Ek)<=B)return;
        else if(ABS(PID->Ek)<=(A+B))PID->I_Term*=(A-ABS(PID->Ek)+B)/A;
        else PID->I_Term=0;
    }
}

/**
  * @}
  */

/** @defgroup	PID_Integral_Limit
  * 积分抗饱和
  * @attention	MaxOUT必须和输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
  * @{
  */

/**
  * @brief  		使能积分抗饱和
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		IntegralLimit: 积分限幅值
  * @param[in]		MaxOUT: 输出限幅值
  * @retval			None
  * @attention		MaxOUT必须和输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
  */
void PID_EnableIntegralLimit(PID_Typedef *PID,float IntegralLimit,float MaxOUT)
{
	//assert((PID->Improvement).OutputLimit.Status==0 || (PID->Improvement).OutputLimit.MaxOUT==(PID->Improvement).IntegralLimit.MaxOUT);
	
	(PID->Improvement).IntegralLimit.Status=1;
	(PID->Improvement).IntegralLimit.IntegralLimit=IntegralLimit;
	(PID->Improvement).IntegralLimit.MaxOUT=MaxOUT;
}

/**
  * @brief  		失能积分抗饱和
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableIntegralLimit(PID_Typedef *PID)
{
	(PID->Improvement).IntegralLimit.Status=0;
	(PID->Improvement).IntegralLimit.IntegralLimit=0;
	(PID->Improvement).IntegralLimit.MaxOUT=0;
}

/**
  * @brief  		积分抗饱和设置参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		IntegralLimit: 积分限幅值
  * @retval			None
  * @attention		MaxOUT只能在输出限幅中修改，参见 @ref PID_SetOutputLimit
  */
void PID_SetIntegralLimit(PID_Typedef *PID,float IntegralLimit)
{
	(PID->Improvement).IntegralLimit.IntegralLimit=IntegralLimit;
}

/**
  * @brief  		积分抗饱和
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  * @note			方法1-超过输出限幅并且呈增长趋势时，停止积分
  * @note			方法2-超过积分限幅，钳制在积分限幅值
  */
static void PID_IntegralLimit(PID_Typedef *PID)
{
    float temp_OUT,temp_IOUT;
    temp_IOUT=PID->I_OUT+PID->I_Term;
    temp_OUT=PID->P_OUT+PID->I_OUT+PID->D_OUT;
	
    if(ABS(temp_OUT)>(PID->Improvement).IntegralLimit.MaxOUT)
    {
        if(PID->Ek*PID->I_OUT>0)//增长趋势
			PID->I_Term=0;
    }

    if(temp_IOUT>(PID->Improvement).IntegralLimit.IntegralLimit)
    {
        PID->I_Term=0;
        PID->I_OUT=(PID->Improvement).IntegralLimit.IntegralLimit;
    }
    if(temp_IOUT<-(PID->Improvement).IntegralLimit.IntegralLimit)
    {
        PID->I_Term=0;
        PID->I_OUT=-(PID->Improvement).IntegralLimit.IntegralLimit;
    }
}

/**
  * @}
  */

/** @defgroup	PID_Derivative_On_Measurement
  * 微分先行
  * @{
  */

/**
  * @brief  		使能微分先行
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_EnableDerivativeOnMeasurement(PID_Typedef *PID)
{
	(PID->Improvement).DerivativeOnMeasurement.Status=1;
}

/**
  * @brief  		失能微分先行
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableDerivativeOnMeasurement(PID_Typedef *PID)
{
	(PID->Improvement).DerivativeOnMeasurement.Status=0;
}

/**
  * @brief  		微分先行
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_DerivativeOnMeasurement(PID_Typedef *PID)
{
    PID->D_OUT=PID->Kd*(PID->Last_Measure-PID->Measure);
}

/**
  * @}
  */

/** @defgroup	PID_Derivative_Filter
  * 微分滤波
  * @{
  */

/**
  * @brief  		使能微分滤波
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		DerivativeFilteringCoefficient: 滤波器系数
  * @retval			None
  */
void PID_EnableDerivativeFilter(PID_Typedef *PID,float DerivativeFilteringCoefficient)
{
	(PID->Improvement).DerivativeFilter.Status=1;
	(PID->Improvement).DerivativeFilter.DerivativeFilteringCoefficient=DerivativeFilteringCoefficient;
}

/**
  * @brief  		失能微分滤波
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableDerivativeFilter(PID_Typedef *PID)
{
	(PID->Improvement).DerivativeFilter.Status=0;
	(PID->Improvement).DerivativeFilter.DerivativeFilteringCoefficient=0;
}

/**
  * @brief  		微分滤波设置参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		DerivativeFilteringCoefficient: 滤波器系数
  * @retval			None
  */
void PID_SetDerivativeFilter(PID_Typedef *PID,float DerivativeFilteringCoefficient)
{
	(PID->Improvement).DerivativeFilter.DerivativeFilteringCoefficient=DerivativeFilteringCoefficient;
}

/**
  * @brief  		微分滤波
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_DerivativeFilter(PID_Typedef *PID)
{
	float K=(PID->Improvement).DerivativeFilter.DerivativeFilteringCoefficient;
    PID->D_OUT=PID->D_OUT*K+PID->Last_DOUT*(1.0f-K);
}

/**
  * @}
  */

/** @defgroup	PID_Output_Limit
  * 输出限幅
  * @warning	建议比例限幅MaxOUT与MaxOUT相同，参见 @ref PID_Proportion_Limit
  * @attention	积分抗饱和MaxOUT必须和MaxOUT相同，参见 @ref PID_Integral_Limit
  * @attention	故障检测必须打开输出限幅，参见 @ref PID_Output_Limit
  * @{
  */

/**
  * @brief  		使能输出限幅
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		MaxOUT: 输出限幅值
  * @retval			None
  * @warning		建议比例限幅MaxOUT与MaxOUT相同，参见 @ref PID_Proportion_Limit
  * @attention		积分抗饱和MaxOUT必须和MaxOUT相同，参见 @ref PID_Integral_Limit
  * @attention		故障检测必须打开输出限幅，参见 @ref PID_Output_Limit
  */
void PID_EnableOutputLimit(PID_Typedef *PID,float MaxOUT)
{
	//assert((PID->Improvement).IntegralLimit.Status==0 || (PID->Improvement).IntegralLimit.MaxOUT==(PID->Improvement).OutputLimit.MaxOUT);
	
	(PID->Improvement).OutputLimit.Status=1;
	(PID->Improvement).OutputLimit.MaxOUT=MaxOUT;
}

/**
  * @brief  		失能输出限幅
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableOutputLimit(PID_Typedef *PID)
{
	(PID->Improvement).OutputLimit.Status=0;
	(PID->Improvement).OutputLimit.MaxOUT=0;
}

/**
  * @brief  		输出限幅设置参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		MaxOUT: 输出限幅值
  * @retval			None
  * @attention		如果使能了积分抗饱和，积分抗饱和的MaxOUT会在输出限幅中修改。
  */
void PID_SetOutputLimit(PID_Typedef *PID,float MaxOUT)
{
	if((PID->Improvement).IntegralLimit.Status==1)
		(PID->Improvement).IntegralLimit.MaxOUT=(PID->Improvement).OutputLimit.MaxOUT=MaxOUT;
	else
		(PID->Improvement).OutputLimit.MaxOUT=MaxOUT;
}

/**
  * @brief  		输出限幅
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_OutputLimit(PID_Typedef *PID)
{
    if(PID->OUT>(PID->Improvement).OutputLimit.MaxOUT)PID->OUT=(PID->Improvement).OutputLimit.MaxOUT;
    if(PID->OUT<-(PID->Improvement).OutputLimit.MaxOUT)PID->OUT=-(PID->Improvement).OutputLimit.MaxOUT;
}

/**
  * @}
  */

/** @defgroup	PID_Output_Filter
  * 输出滤波
  * @{
  */

/**
  * @brief  		使能输出滤波
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		OutputFilteringCoefficient: 滤波器系数
  * @retval			None
  */
void PID_EnableOutputFilter(PID_Typedef *PID,float OutputFilteringCoefficient)
{
	(PID->Improvement).OutputFilter.Status=1;
	(PID->Improvement).OutputFilter.OutputFilteringCoefficient=OutputFilteringCoefficient;
}

/**
  * @brief  		失能输出滤波
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableOutputFilter(PID_Typedef *PID)
{
	(PID->Improvement).OutputFilter.Status=0;
	(PID->Improvement).OutputFilter.OutputFilteringCoefficient=0;
}

/**
  * @brief  		输出滤波设置参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		OutputFilteringCoefficient: 滤波器系数
  * @retval			None
  */
void PID_SetOutputFilter(PID_Typedef *PID,float OutputFilteringCoefficient)
{
	(PID->Improvement).OutputFilter.OutputFilteringCoefficient=OutputFilteringCoefficient;
}

/**
  * @brief  		输出滤波
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_OutputFilter(PID_Typedef *PID)
{
	float K=(PID->Improvement).OutputFilter.OutputFilteringCoefficient;
    PID->OUT=PID->OUT*K+PID->Last_OUT*(1.0f-K);
}

/**
  * @}
  */

/** @defgroup	PID_Error_Handle
  * 故障检测
  * @attention	故障检测必须打开输出限幅，参见 @ref PID_Output_Limit
  * @{
  */

/**
  * @brief  		使能故障检测
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  * @attention		故障检测必须打开输出限幅，参见 @ref PID_Output_Limit
  */
void PID_EnableErrorHandle(PID_Typedef *PID)
{
	//assert((PID->Improvement).OutputLimit.Status==1);
	
	(PID->Improvement).ErrorHandle.Status=1;
	(PID->Improvement).ErrorHandle.ERRORCount=0;
	(PID->Improvement).ErrorHandle.Motor_BlockedFlag=0;
}

/**
  * @brief  		失能故障检测
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_DisableErrorHandle(PID_Typedef *PID)
{
	(PID->Improvement).ErrorHandle.Status=0;
	(PID->Improvement).ErrorHandle.ERRORCount=0;
	(PID->Improvement).ErrorHandle.Motor_BlockedFlag=0;
}

/**
  * @brief  		清理故障检测
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_ClearErrorHandle(PID_Typedef *PID)
{
	(PID->Improvement).ErrorHandle.ERRORCount=0;
	(PID->Improvement).ErrorHandle.Motor_BlockedFlag=0;
}

/**
  * @brief  		故障检测
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
static void PID_ErrorHandle(PID_Typedef *PID)
{
    if(PID->OUT<(PID->Improvement).OutputLimit.MaxOUT*0.01f)return;//排除排除PID输出本身很小的情况
	
    if((ABS(PID->Target-PID->Measure)/PID->Target)>0.9f)//故障计数
        (PID->Improvement).ErrorHandle.ERRORCount++;
    else
		(PID->Improvement).ErrorHandle.ERRORCount=0;

    if((PID->Improvement).ErrorHandle.ERRORCount>1000)//故障发生
        (PID->Improvement).ErrorHandle.Motor_BlockedFlag=1;
}

/**
  * @}
  */

/**
  * @brief  		复位优化配置
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_ImprovementReset(PID_Typedef *PID)
{
	PID_DisableProportionLimit(PID);
	PID_DisableTrapezoidIntergral(PID);
	PID_DisableIntegralSeparation(PID);
	PID_DisableChangingIntegralRate(PID);
	PID_DisableIntegralLimit(PID);
	PID_DisableDerivativeOnMeasurement(PID);
	PID_DisableDerivativeFilter(PID);
	PID_DisableOutputLimit(PID);
	PID_DisableOutputFilter(PID);
	PID_DisableErrorHandle(PID);
}

/**
  * @}
  */

/* PID Clean and Reset ********************************************************/

/**
  * @brief  		PID清理
  * @details		清理PID的缓存。
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  * @note   		并不会清理Target和优化配置，这与 @ref PID_Reset 不同。
  */
void PID_Clean(PID_Typedef *PID)
{
	if((PID->Improvement).ErrorHandle.Status==1)PID_ClearErrorHandle(PID);//清理电机堵转标志位
	
	PID->Measure=PID->Last_Measure=0;
	PID->Ek=PID->Ek_1=0;
	PID->P_OUT=PID->I_OUT=PID->D_OUT=0;
	PID->I_Term=0;
	PID->Last_DOUT=0;
	PID->OUT=PID->Last_OUT=0;
}

/**
  * @brief  		PID复位
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @retval			None
  */
void PID_Reset(PID_Typedef *PID)
{
	PID->Target=0;
	PID->Measure=PID->Last_Measure=0;
	
	PID->Ek=PID->Ek_1=0;
	PID->DeadBand=0;
	
	PID->Kp=PID->Ki=PID->Kd=0;
	
	PID->P_OUT=PID->I_OUT=PID->D_OUT=0;
	PID->I_Term=0;
	PID->Last_DOUT=0;
	
	PID->OUT=PID->Last_OUT=0;
	
	PID_ImprovementReset(PID);//复位优化配置
}

/* PID Structure Initialize ***************************************************/

/**
  * @brief  		PID初始化
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		Target: 目标值
  * @param[in]		DeadBand: 误差死区
  * @retval			None
  * @attention		PID初始化并不会初始化Kp、Ki和Kd，需要用 @ref PID_SetParameter 函数来初始化
  */
void PID_Init(PID_Typedef *PID,float Target,float DeadBand)
{
	PID->Target=Target;
	PID->Measure=PID->Last_Measure=0;
	
	PID->Ek=PID->Ek_1=0;
	PID->DeadBand=DeadBand;
	
	PID->Kp=PID->Ki=PID->Kd=0;
	
	PID->P_OUT=PID->I_OUT=PID->D_OUT=0;
	PID->I_Term=0;
	PID->Last_DOUT=0;
	
	PID->OUT=PID->Last_OUT=0;
	
	PID_ImprovementReset(PID);
}

/* PID Set Param and DeadBand *************************************************/

/**
  * @brief  		设置PID参数
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		kp: 比例系数Kp
  * @param[in]		ki: 积分系数Ki
  * @param[in]		kd: 微分系数Kd
  * @retval			None
  */
void PID_SetParameter(PID_Typedef *PID,float kp,float ki,float kd)
{
	PID->Kp=kp;
	PID->Ki=ki;
	PID->Kd=kd;
}

/**
  * @brief  		设置PID误差死区
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		DeadBand: 误差死区
  * @retval			None
  */
void PID_SetEkDeadBand(PID_Typedef *PID,float DeadBand)
{
	PID->DeadBand=DeadBand;
}

/* PID Calculate **************************************************************/

/**
  * @brief  		PID计算
  * @param[in,out]	*PID: PID_Typedef结构体指针，参见 @ref PID_Typedef
  * @param[in]		measure: 测量值
  * @retval			None
  * @note   		计算结果将保存在PID结构体中。
  * @note   		优化配置：
  *					@arg 比例限幅：		建议比例限幅的MaxOUT和输出限幅的MaxOUT相同
  *					@arg 梯形积分：		None
  *					@arg 积分分离：		变速积分和积分分离不能同时使用，积分分离优先级低
  *					@arg 变速积分：		变速积分和积分分离不能同时使用，变速积分优先级高
  *					@arg 积分抗饱和：	积分抗饱和的MaxOUT必须和输出限幅的MaxOUT相同
  *					@arg 微分先行：		None
  *					@arg 微分滤波：		None
  *					@arg 输出限幅：		None
  *					@arg 输出滤波：		None
  *					@arg 故障检测：		故障检测必须打开输出限幅
  */
void PID_Calculate(PID_Typedef *PID,float measure)
{
    if((PID->Improvement).ErrorHandle.Status==1)//检测堵转
    {
        PID_ErrorHandle(PID);
        if((PID->Improvement).ErrorHandle.Motor_BlockedFlag==1)//堵转情况下，关断输出
        {
            PID->OUT=0;
            return;
        }
    }

    PID->Last_Measure=PID->Measure;
    PID->Measure=measure;
    PID->Ek_1=PID->Ek;
    PID->Ek=PID->Target-PID->Measure;
	
    PID->Last_DOUT=PID->D_OUT;
    PID->Last_OUT=PID->OUT;

    if(ABS(PID->Ek)>PID->DeadBand)
    {
		/*================================ 比例	================================*/
        PID->P_OUT=PID->Kp*PID->Ek;
		
        if((PID->Improvement).ProportionLimit.Status==1)PID_ProportionLimit(PID);//比例限幅
		
		/*================================ 微分	================================*/
        if((PID->Improvement).DerivativeOnMeasurement.Status==1)PID_DerivativeOnMeasurement(PID);//微分先行
		else PID->D_OUT=PID->Kd*(PID->Ek-PID->Ek_1);
        
        if((PID->Improvement).DerivativeFilter.Status==1)PID_DerivativeFilter(PID);//微分滤波

		/*================================ 积分	================================*/
        if((PID->Improvement).TrapezoidIntergral.Status==1)PID_TrapezoidIntergral(PID);//梯形积分
		else PID->I_Term=PID->Ki*PID->Ek;
		
        if((PID->Improvement).ChangingIntegralRate.Status==1)PID_ChangingIntegralRate(PID);//变速积分
		else if((PID->Improvement).IntegralSeparation.Status==1)PID_IntegralSeparation(PID);//积分分离

        if((PID->Improvement).IntegralLimit.Status==1)PID_IntegralLimit(PID);//抗积分饱和
        
		PID->I_OUT+=PID->I_Term;
		
		/*================================ 输出	================================*/
        PID->OUT=PID->P_OUT+PID->I_OUT+PID->D_OUT;
        
        if((PID->Improvement).OutputFilter.Status==1)PID_OutputFilter(PID);//输出滤波
        
		if((PID->Improvement).OutputLimit.Status==1)PID_OutputLimit(PID);//输出限幅
    }
	/*使用者自己加的else分支，编写者没写*/
	else
	{
		PID->P_OUT = 0;
		PID->OUT = 0;
	}
}

/**
  * @}
  */

/**
  * @}
  */
