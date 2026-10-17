#ifndef __ATTITUDEALGORITHMS_H
#define __ATTITUDEALGORITHMS_H

#include <stm32f4xx.h>

extern float AttitudeAlgorithms_RadYaw,AttitudeAlgorithms_RadPitch,AttitudeAlgorithms_RadRoll;//弧度制角度
extern float AttitudeAlgorithms_DegYaw,AttitudeAlgorithms_DegPitch,AttitudeAlgorithms_DegRoll;//角度制角度
extern int64_t AttitudeAlgorithms_YawR;//角度制偏航角圈数
extern float AttitudeAlgorithms_RaddYaw_E,AttitudeAlgorithms_RaddPitch_E;
extern float AttitudeAlgorithms_ThisDegYaw;//本次角度制偏航角

void AttitudeAlgorithms_Init(void);//姿态解算初始化

#endif
