#ifndef __LOCK_H
#define __LOCK_H

void Lock_Init(void);//安全锁初始化
uint8_t Lock_Check(void);//安全锁检测
void Lock_EraseFLASH(void);//安全锁触发

#endif
