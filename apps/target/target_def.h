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

#endif /* _TARGET_DEF_H_ */
