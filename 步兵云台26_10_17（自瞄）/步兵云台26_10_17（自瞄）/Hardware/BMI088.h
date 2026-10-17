#ifndef __BMI088_H
#define __BMI088_H

extern float BMI088_Accel[],BMI088_Gyro[],BMI088_Temperature;//BMI088的三轴加速度数据,三轴角速度数据和温度
extern int16_t BMI088_RawAccelData[],BMI088_RawGyroData[],BMI088_RawTemperatureData;//BMI088的三轴加速度原始数据,三轴角速度原始数据和温度原始数据

void BMI088_Init(void);//BMI088初始化

#endif
