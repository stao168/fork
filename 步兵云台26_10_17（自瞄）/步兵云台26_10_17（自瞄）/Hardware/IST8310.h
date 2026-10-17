#ifndef __IST8310_H
#define __IST8310_H

extern int16_t IST8310_RawData[];//IST8310原始数据,1~3为xyz轴磁场原始数据,4为温度原始数据
extern float IST8310_MagneticField[];//IST8310磁场数据,分别为xyz轴磁场数据(单位uT)
extern float IST8310_Temperature;//IST8310温度数据(单位℃)

void IST8310_Init(void);//IST8310初始化
void IST8310_Reset(void);//IST8310重启
void IST8310_GetData(void);//IST8310数据读取

#endif
