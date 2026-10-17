#ifndef __LASER_H
#define __LASER_H

void Laser_Init(void);//激光初始化
void Laser_ON(void);//激光普通模式打开激光
void Laser_OFF(void);//激光普通模式关闭激光
void Laser_PWMInit(void);//激光PWM模式初始化
void Laser_PWMON(float Brightness);//激光PWM模式以一定亮度打开激光
void Laser_PWMOFF(void);//激光PWM模式关闭激光

#endif
