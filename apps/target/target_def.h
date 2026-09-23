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
#define TARGET_LOG_PERIOD_MS       500      /* 电机状态打印周期 (ms, 调试用) */

// clang-format on

/* ================================================================
 * 遥控器按键 (PS2; bit n-1, 完整按键表见 module_remote.h)
 *   L1=bit10  R1=bit11  △=bit12  ○=bit13  ×=bit14  □=bit15
 * ================================================================ */
// clang-format off

#define TARGET_DM_TOGGLE_BTN   (1u << 10) /* L1: 达妙 DM6220 开关 */
#define TARGET_DM_ARM_BTN      (1u << 11) /* R1: 达妙重发使能帧 (清错 + 启动) */
#define TARGET_3508_TOGGLE_BTN (1u << 13) /* ○ : M3508 开关 */

// clang-format on

/* ================================================================
 * DJI M3508 (C620 电调)
 * ================================================================ */
// clang-format off

#define TARGET_3508_CAN_ID             8u      /* C620 电调 ID (1~8); 反馈 ID 自动 = 0x200 + ID */
#define TARGET_3508_OFFLINE_TIMEOUT_MS 100u    /* 掉线判定 (ms) */
#define TARGET_3508_GEAR_RATIO         19.203f /* M3508 减速比 (3591/187) */

/* 转速: 速度环的 ref 单位是"转子 rad/s"(见 motor_algorithm.c, 不做齿轮比换算),
 * 所以输出轴 1 转/秒 = 1 × 2π × 减速比 ≈ 120.7 rad/s (转子 ≈ 1152 rpm) */
#define TARGET_3508_OUTPUT_REV_S 1.0f
#define TARGET_3508_SPEED_RAD_S  (TARGET_3508_OUTPUT_REV_S * 2.0f * 3.14159265f * TARGET_3508_GEAR_RATIO)

#define TARGET_3508_LQR_K           0.05f /* 速度环增益 (纯比例, 需实测调) */
#define TARGET_3508_MAX_TORQUE      5.32f
#define TARGET_3508_TORQUE_CONSTANT 0.016f

// clang-format on

/* ================================================================
 * 达妙 DM6220 (暂时不接入, 原因见 motor_func.c)
 * ================================================================ */
// clang-format off

#define TARGET_DM_CAN_TX_ID          0x01u /* 电机 CAN ID */
#define TARGET_DM_CAN_RX_ID          0xF1u /* 主控 ID (必须大于 tx_id) */
#define TARGET_DM_OFFLINE_TIMEOUT_MS 100u  /* 掉线判定 (ms) */

#define TARGET_DM_SPEED_RAD_S (0.4f * 2.0f * 3.14159265f) /* 0.4 转/秒 */

// clang-format on

#endif /* _TARGET_DEF_H_ */
