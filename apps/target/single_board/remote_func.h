/*
 * @file    remote_func.h
 * @brief   target 兵种(靶车) — 遥控器(PS2 手柄)输入解析
 */

#ifndef _REMOTE_FUNC_H_
#define _REMOTE_FUNC_H_

#include <stdbool.h>

/**
 * @brief 遥控器按键事件
 * @note  每个控制周期解析一次, 只有"按下那一刻"为 true (上升沿), 不会连发
 */
typedef struct
{
    bool dm_toggle;    /* 达妙 DM6220 开关   (L1) */
    bool dm_arm;       /* 达妙重发使能帧     (R1) */
    bool m3508_toggle; /* M3508 开关         (○)  */
} Remote_Btn_Event_t;

/**
 * @brief 遥控器输入初始化
 */
void remote_func_init(void);

/**
 * @brief 每周期调用: 读手柄数据并解析按键
 * @param evt 输出按键事件 (本周期无事件时全为 false)
 */
void remote_func(Remote_Btn_Event_t *evt);

#endif /* _REMOTE_FUNC_H_ */
