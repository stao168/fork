/**
  ******************************************************************************
  * @file		PID.h
  * @brief		This is the PID lib of MAS Robot.
  * @details	我们使用位置式PID，并通过一定算法对PID进行了优化，代码中存在一些bug，对此表示由衷的歉意。
  * @details	部分代码参考于玺佬开源(https://zhuanlan.zhihu.com/p/128064498)。
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
  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H__
#define __PID_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"

/** @addtogroup MAS_Control
  * @{
  */

/** @addtogroup PID
  * @{
  */
   
/* Exported types ------------------------------------------------------------*/

/** @defgroup 	PID_Improvement
  * @{
  */

/** @defgroup 	PID_Proportion_Limit
  * @{
  */
  
/** 
  * @brief   	This is the struct of Proportion Limit.
  * @warning	建议MaxOUT与输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
  */ 
typedef struct
{
	uint8_t Status;	///< 使能状态
	float MaxOUT;	///< 比例限幅值 @warning 建议MaxOUT与输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
}Proportion_Limit;

/**
  * @}
  */

/** @defgroup 	PID_Trapezoid_Intergral
  * @{
  */

/** 
  * @brief   	This is the struct of Trapezoid Intergral.
  */ 
typedef struct
{
	uint8_t Status;	///< 使能状态
}Trapezoid_Intergral;

/**
  * @}
  */

/** @defgroup 	PID_Integral_Separation
  * @{
  */

/** 
  * @brief   	This is the struct of Integral Separation.
  * @attention	变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Changing_Integral_Rate
  */ 
typedef struct
{
	uint8_t Status;	///< 使能状态
	float MaxErr;	///< 积分分离误差阈值
}Integral_Separation;

/**
  * @}
  */

/** @defgroup 	PID_Changing_Integral_Rate
  * @{
  */

/** 
  * @brief   	This is the struct of Changing Integral Rate.
  * @note		ITerm=Ki*Err*((A-abs(Err)+B)/A)	when B<|err|\<A+B
  * @attention	变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Integral_Separation
  */ 
typedef struct
{
	uint8_t Status;	///< 使能状态
    float ScalarA;	///< 系数A
    float ScalarB;	///< 系数B
}Changing_Integral_Rate;

/**
  * @}
  */

/** @defgroup 	PID_Integral_Limit
  * @{
  */

/** 
  * @brief   	This is the struct of Integral Limit.
  * @attention	MaxOUT必须和输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
  */ 
typedef struct
{
	uint8_t Status;			///< 使能状态
	float IntegralLimit;	///< 积分限幅值
    float MaxOUT;			///< 输出限幅值 @attention	MaxOUT必须和输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
}Integral_Limit;

/**
  * @}
  */

/** @defgroup 	PID_Derivative_On_Measurement
  * @{
  */

/** 
  * @brief   	This is the struct of Derivative On Measurement.
  */ 
typedef struct
{
	uint8_t Status;	///< 使能状态
}Derivative_On_Measurement;

/**
  * @}
  */

/** @defgroup 	PID_Derivative_Filter
  * @{
  */

/** 
  * @brief   	This is the struct of Derivative Filter.
  */ 
typedef struct
{
	uint8_t Status;							///< 使能状态
    float DerivativeFilteringCoefficient;	///< 滤波器系数
}Derivative_Filter;

/**
  * @}
  */

/** @defgroup 	PID_Output_Limit
  * @{
  */

/** 
  * @brief   	This is the struct of Output Limit.
  * @warning	建议比例限幅MaxOUT与MaxOUT相同，参见 @ref PID_Proportion_Limit
  * @attention	积分抗饱和MaxOUT必须和MaxOUT相同，参见 @ref PID_Integral_Limit
  * @attention	故障检测必须打开输出限幅，参见 @ref PID_Output_Limit
  */ 
typedef struct
{
	uint8_t Status;	///< 使能状态
    float MaxOUT;	///< 输出限幅值 @warning 建议比例限幅MaxOUT与MaxOUT相同，参见 @ref PID_Proportion_Limit @attention 抗积分饱和MaxOUT必须和MaxOUT相同，参见 @ref PID_Integral_Limit
}Output_Limit;

/**
  * @}
  */

/** @defgroup 	PID_Output_Filter
  * @{
  */

/** 
  * @brief   	This is the struct of Output Filter.
  */ 
typedef struct
{
	uint8_t Status;						///< 使能状态
    float OutputFilteringCoefficient;	///< 滤波器系数
}Output_Filter;

/**
  * @}
  */

/** @defgroup 	PID_Error_Handle
  * @{
  */

/** 
  * @brief   	This is the struct of Error Handle.
  */ 
typedef struct
{
	uint8_t Status;				///< 使能状态
    uint64_t ERRORCount;		///< 故障计数
	uint8_t Motor_BlockedFlag;	///< 电机堵转标志位 @arg 0：未堵转 @arg 1：堵转
}Error_Handle;

/**
  * @}
  */

/** 
  * @brief   	This is the struct of PID Improvement
  */ 
typedef struct
{
	Proportion_Limit ProportionLimit;					///< 比例限幅 @warning 建议MaxOUT与输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
	Trapezoid_Intergral TrapezoidIntergral;				///< 梯形积分
	Integral_Separation IntegralSeparation;				///< 积分分离 @attention 变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Changing_Integral_Rate
	Changing_Integral_Rate ChangingIntegralRate;		///< 变速积分 @attention 变速积分和积分分离不能同时使用，变速积分优先级高，参见 @ref PID_Integral_Separation
	Integral_Limit IntegralLimit;						///< 积分抗饱和 @attention	MaxOUT必须和输出限幅MaxOUT相同，参见 @ref PID_Output_Limit
	Derivative_On_Measurement DerivativeOnMeasurement;	///< 微分先行
	Derivative_Filter DerivativeFilter;					///< 微分滤波
	Output_Limit OutputLimit;							///< 输出限幅 @warning 建议比例限幅MaxOUT与MaxOUT相同，参见 @ref PID_Proportion_Limit @attention 抗积分饱和MaxOUT必须和MaxOUT相同，参见 @ref PID_Integral_Limit @attention 故障检测必须打开输出限幅，参见 @ref PID_Output_Limit
	Output_Filter OutputFilter;							///< 输出滤波
	Error_Handle ErrorHandle;							///< 故障检测 @attention 故障检测必须打开输出限幅，参见 @ref PID_Output_Limit
}PID_Improvement;

/**
  * @}
  */

/** 
  * @brief   	This is the struct of PID  
  */ 
typedef struct
{
	float Target;					///< 目标值
    float Measure;					///< 测量值
    float Last_Measure;				///< 上一次测量值

	float Ek;						///< 误差
	float Ek_1;						///< 上一次误差
	float DeadBand;					///< 误差死区

	float Kp;						///< Kp
	float Ki;						///< Ki
	float Kd;						///< Kd

	float P_OUT;					///< 比例输出
	float I_OUT;					///< 积分输出
	float D_OUT;					///< 微分输出
    float I_Term;					///< 积分项
    float Last_DOUT;				///< 上一次微分输出
	
	float OUT;						///< 输出
	float Last_OUT;					///< 上一次输出
	
	PID_Improvement Improvement;	///< PID优化，参见 @ref PID_Improvement
}PID_Typedef;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  

/* PID Improvement Function ***************************************************/

/** @addtogroup PID_Improvement
  * @{
  */

/** @addtogroup PID_Proportion_Limit
  * @{
  */

void PID_EnableProportionLimit(PID_Typedef *PID,float MaxOUT);
void PID_DisableProportionLimit(PID_Typedef *PID);
void PID_SetProportionLimit(PID_Typedef *PID,float MaxOUT);

/**
  * @}
  */

/** @addtogroup PID_Trapezoid_Intergral
  * @{
  */

void PID_EnableTrapezoidIntergral(PID_Typedef *PID);
void PID_DisableTrapezoidIntergral(PID_Typedef *PID);

/**
  * @}
  */

/** @addtogroup PID_Integral_Separation
  * @{
  */

void PID_EnableIntegralSeparation(PID_Typedef *PID,float MaxErr);
void PID_DisableIntegralSeparation(PID_Typedef *PID);
void PID_SetIntegralSeparation(PID_Typedef *PID,float MaxErr);

/**
  * @}
  */

/** @addtogroup PID_Changing_Integral_Rate
  * @{
  */

void PID_EnableChangingIntegralRate(PID_Typedef *PID,float ScalarA,float ScalarB);
void PID_DisableChangingIntegralRate(PID_Typedef *PID);
void PID_SetChangingIntegralRate(PID_Typedef *PID,float ScalarA,float ScalarB);

/**
  * @}
  */

/** @addtogroup PID_Integral_Limit
  * @{
  */

void PID_EnableIntegralLimit(PID_Typedef *PID,float IntegralLimit,float MaxOUT);
void PID_DisableIntegralLimit(PID_Typedef *PID);
void PID_SetIntegralLimit(PID_Typedef *PID,float IntegralLimit);

/**
  * @}
  */

/** @addtogroup PID_Derivative_On_Measurement
  * @{
  */

void PID_EnableDerivativeOnMeasurement(PID_Typedef *PID);
void PID_DisableDerivativeOnMeasurement(PID_Typedef *PID);

/**
  * @}
  */

/** @addtogroup PID_Derivative_Filter
  * @{
  */

void PID_EnableDerivativeFilter(PID_Typedef *PID,float DerivativeFilteringCoefficient);
void PID_DisableDerivativeFilter(PID_Typedef *PID);
void PID_SetDerivativeFilter(PID_Typedef *PID,float DerivativeFilteringCoefficient);

/**
  * @}
  */

/** @addtogroup PID_Output_Limit
  * @{
  */

void PID_EnableOutputLimit(PID_Typedef *PID,float MaxOUT);
void PID_DisableOutputLimit(PID_Typedef *PID);
void PID_SetOutputLimit(PID_Typedef *PID,float MaxOUT);

/**
  * @}
  */

/** @addtogroup PID_Output_Filter
  * @{
  */

void PID_EnableOutputFilter(PID_Typedef *PID,float OutputFilteringCoefficient);
void PID_DisableOutputFilter(PID_Typedef *PID);
void PID_SetOutputFilter(PID_Typedef *PID,float OutputFilteringCoefficient);

/**
  * @}
  */

/** @addtogroup	PID_Error_Handle
  * @{
  */

void PID_EnableErrorHandle(PID_Typedef *PID);
void PID_DisableErrorHandle(PID_Typedef *PID);
void PID_ClearErrorHandle(PID_Typedef *PID);

/**
  * @}
  */

void PID_ImprovementReset(PID_Typedef *PID);

/**
  * @}
  */

/* PID Clean and Reset ********************************************************/

void PID_Clean(PID_Typedef *PID);
void PID_Reset(PID_Typedef *PID);

/* PID Structure Initialize ***************************************************/

void PID_Init(PID_Typedef *PID,float Target,float DeadBand);

/* PID Set Param and DeadBand *************************************************/

void PID_SetParameter(PID_Typedef *PID,float kp,float ki,float kd);
void PID_SetEkDeadBand(PID_Typedef *PID,float DeadBand);

/* PID Calculate **************************************************************/

void PID_Calculate(PID_Typedef *PID,float measure);

#endif /*__PID_H__ */

/**
  * @}
  */

/**
  * @}
  */
