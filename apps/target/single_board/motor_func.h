/*
 * @file    motor_func.h
 * @brief   target 兵种(靶车) — 电机 (DJI M3508 / 达妙 DM6220)
 */

#ifndef _MOTOR_FUNC_H_
#define _MOTOR_FUNC_H_

#include "remote_func.h"

/**
 * @brief 电机初始化 (注册到电机模块; 上电默认停转, 等按键)
 */
void motor_func_init(void);

/**
 * @brief 每周期调用: 方向键 → 转速, 其余按键 → 启停/换向/模式
 */
void motor_func(const Remote_Input_t *in);

#endif /* _MOTOR_FUNC_H_ */
