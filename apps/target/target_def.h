/*
 * @file    target_def.h
 * @brief   target 兵种(靶车) — 参数定义
 */

#ifndef _TARGET_DEF_H_
#define _TARGET_DEF_H_

#include <stdint.h>

/* ================================================================
 * 时间参数
 * ================================================================ */
// clang-format off

#define TARGET_CONTROL_PERIOD_MS   2        /* 主控制周期 (ms) */
#define TARGET_LOG_PERIOD_MS       500      /* PS2 通道打印周期 (ms, 调试用) */

// clang-format on

/* ================================================================
 * 达妙 DM6220 电机
 * ================================================================ */
// clang-format off

#define TARGET_MOTOR_CAN_TX_ID          0x01u                       /* 电机 CAN ID */
#define TARGET_MOTOR_CAN_RX_ID          0xF1u                       /* 主控 ID (必须大于 tx_id) */
#define TARGET_MOTOR_OFFLINE_TIMEOUT_MS 100u                        /* 掉线判定 (ms) */
#define TARGET_MOTOR_SPEED_RAD_S        (0.4f * 2.0f * 3.14159265f) /* 0.4 转/秒 = 2.513 rad/s */

/* PS2 按键位 (bit n-1): 11=L1 → bit10。改成别的键只需改这一行 */
#define TARGET_MOTOR_TOGGLE_BTN         (1u << 10)                  /* L1: 按一下开关电机 */

// clang-format on

#endif /* _TARGET_DEF_H_ */
