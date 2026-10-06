/*
 * @file    remote_func.h
 * @brief   target 兵种(靶车) — 遥控器(PS2 手柄)输入解析
 */

#ifndef _REMOTE_FUNC_H_
#define _REMOTE_FUNC_H_

#include <stdbool.h>

/**
 * @brief 遥控器输入 (每个控制周期刷新一次)
 * @note  除方向键外都是"按下那一刻"事件 (上升沿, 不会连发);
 *        方向键是"按住有效"的电平; 手柄掉线时全部强制为 0
 */
typedef struct
{
    /* 按键事件 (上升沿) */
    bool m3508_toggle; /* M3508 使能开关     (L1) */
    bool dm_power;     /* 达妙 总开关        (R1) */
    bool dm_clear;     /* 达妙 重发清错+启动 (R2) */
    bool dm_fixed;     /* 达妙 固定转速模式  (△)  */
    bool dm_throttle;  /* 达妙 油门模式      (□)  */
    bool dm_dir;       /* 达妙 正反转切换    (×)  */

    /* 方向键 (电平: 按住就为 true) */
    bool key_fwd;  /* ↑ : M3508 前进 */
    bool key_back; /* ↓ : M3508 后退 */
    bool key_acc;  /* ← : 达妙转速加 */
    bool key_dec;  /* → : 达妙转速减 */
} Remote_Input_t;

/**
 * @brief 遥控器输入初始化
 */
void remote_func_init(void);

/**
 * @brief 每周期调用: 读手柄数据并解析按键与方向键
 * @param in 输出本周期遥控输入 (无事件/未按住时全部为 false)
 */
void remote_func(Remote_Input_t *in);

#endif /* _REMOTE_FUNC_H_ */
