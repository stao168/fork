#ifndef __IMUTEMPERATURECONTROL_H
#define __IMUTEMPERATURECONTROL_H

extern uint8_t IMUTemperatureControl_OpenFlag;//IMU恒温控制打开标志位

void IMUTemperatureControl_PIDInit(void);//IMU恒温控制PID初始化
void IMUTemperatureControl_Init(void);//IMU恒温控制初始化
void IMUTemperatureControl_TemperatureControl(void);//IMU恒温控制

#endif
