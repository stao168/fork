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
 * 遥控器 (PS2; 按键位 n-1, 完整按键表见 module_remote.h)
 *   方向键 ↑=bit4  →=bit5  ↓=bit6  ←=bit7
 *   L2=bit8  R2=bit9  L1=bit10  R1=bit11  △=bit12  ○=bit13  ×=bit14  □=bit15
 *   方向键是"按住有效"的电平, 其余按键是"按一下"的上升沿
 * ================================================================ */
// clang-format off

#define TARGET_3508_ON_BTN   (1u << 10) /* L1 : M3508 使能开关 */
#define TARGET_3508_FWD_BTN  (1u << 4)  /* ↑  : M3508 前进 (输出轴 +1 转/秒) */
#define TARGET_3508_BACK_BTN (1u << 6)  /* ↓  : M3508 后退 (输出轴 -1 转/秒) */

#define TARGET_DM_POWER_BTN    (1u << 11) /* R1 : 达妙 总开关 (按下清零转速, 并复位两个模式) */
#define TARGET_DM_CLEAR_BTN    (1u << 9)  /* R2 : 达妙 重发清错+启动 (电机晚上电时用) */
#define TARGET_DM_FIXED_BTN    (1u << 12) /* △  : 达妙 固定转速模式 */
#define TARGET_DM_THROTTLE_BTN (1u << 15) /* □  : 达妙 油门模式 (转速由 ←/→ 增减) */
#define TARGET_DM_DIR_BTN      (1u << 14) /* ×  : 达妙 正反转切换 */
#define TARGET_DM_ACC_BTN      (1u << 7)  /* ←  : 达妙 转速加 */
#define TARGET_DM_DEC_BTN      (1u << 5)  /* →  : 达妙 转速减 */

// clang-format on

/* ================================================================
 * DJI M3508 (C620 电调)
 * ================================================================ */
// clang-format off

/* C620 电调 ID (1~8), 反馈 ID 自动 = 0x200 + ID; ID 5~8 的控制帧走 0x1FF 组。
 * 当前电调拨码 = 8 ⇒ 控制帧 0x1FF、反馈帧 0x208。
 * 达妙 DM6220 是 MIT 模式(控制帧 0x001), 和这里不冲突, 不用特意错开 ID。 */
#define TARGET_3508_CAN_ID             8u
#define TARGET_3508_OFFLINE_TIMEOUT_MS 100u    /* 掉线判定 (ms) */
#define TARGET_3508_GEAR_RATIO         19.203f /* M3508 减速比 (3591/187) */

/* 速度环的 ref 单位是"转子 rad/s"(见 motor_algorithm.c, 不做齿轮比换算),
 * 所以输出轴 1 转/秒 = 1 × 2π × 减速比 ≈ 120.7 rad/s (转子 ≈ 1152 rpm)。
 * ↑/↓ 按住就以这个速度前进/后退, 松开(或两个都按)回到 0 */
#define TARGET_3508_SPEED_REV_S 1.0f
#define TARGET_3508_SPEED_RAD_S (TARGET_3508_SPEED_REV_S * 2.0f * 3.14159265f * TARGET_3508_GEAR_RATIO)

#define TARGET_3508_LQR_K           0.05f /* 速度环增益 (纯比例, 需实测调) */
#define TARGET_3508_MAX_TORQUE      5.32f
#define TARGET_3508_TORQUE_CONSTANT 0.016f

// clang-format on

/* ================================================================
 * 达妙 DM6220 (和 M3508 同在 CAN1, 电机内部是 MIT 模式)
 *   MIT 控制帧 ID = CAN_ID + 0x000 = 0x001, 反馈帧 ID = Master ID = 0xF1
 *   不动电机参数; 速度环由主控自己闭(见 motor_func.c), 输出力矩经 MIT 帧下发
 * ================================================================ */
// clang-format off

#define TARGET_DM_CAN_TX_ID          0x01u /* 电机 CAN_ID (保持不动) */
#define TARGET_DM_CAN_RX_ID          0xF1u /* Master ID (须大于 CAN_ID, 也在电机里设) */
#define TARGET_DM_OFFLINE_TIMEOUT_MS 100u  /* 掉线判定 (ms) */

/* 固定转速模式: 恒速 0.4 转/秒 (gear_ratio = 1 ⇒ rad/s 就是输出轴) */
#define TARGET_DM_FIXED_SPEED_REV_S 0.4f
#define TARGET_DM_FIXED_SPEED_RAD_S (TARGET_DM_FIXED_SPEED_REV_S * 2.0f * 3.14159265f)

/* 油门模式: ← 按住 = 转速增加, → 按住 = 减小, 松开保持当前值;
 * 限幅在 ±TARGET_DM_THROTTLE_MAX_REV_S, 按住 TARGET_DM_THROTTLE_RAMP_TIME_S 秒到满速 */
#define TARGET_DM_THROTTLE_MAX_REV_S   1.0f
#define TARGET_DM_THROTTLE_MAX_RAD_S   (TARGET_DM_THROTTLE_MAX_REV_S * 2.0f * 3.14159265f)
#define TARGET_DM_THROTTLE_RAMP_TIME_S 1.0f
#define TARGET_DM_THROTTLE_RAMP_RAD_S2 (TARGET_DM_THROTTLE_MAX_RAD_S / TARGET_DM_THROTTLE_RAMP_TIME_S)

/* 速度环 PID(输出直接就是力矩, Nm)。取自之前工程实测能用的达妙参数: 只留一个很小的 P。
 *   ⚠️ Ki 必须为 0 (pid.c 里 ITerm = Ki*Err*dt): 一旦有积分, 模式切换/清零转速之后
 *   积分残留会继续输出力矩推电机 —— 就是之前“越转越快”的病根。Kp 大 / 加 Ki 还会
 *   和电机内部环打架 ⇒ 左右振。
 * (本环只用 P, 无任何状态, 所以模式切换可以放心硬切 ref) */
#define TARGET_DM_MAX_TORQUE 3.0f  /* 力矩限幅 (Nm), 只是安全帽, 正常远达不到 */
#define TARGET_DM_SPEED_KP   0.05f
#define TARGET_DM_SPEED_KI   0.0f
#define TARGET_DM_SPEED_KD   0.0f

// clang-format on

#endif /* _TARGET_DEF_H_ */
