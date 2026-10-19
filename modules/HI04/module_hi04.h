/*
 * @file    module_hi04.h
 * @brief   HI04(HiPNUC HI91) 外置 IMU 模块 —— UART 921600
 *
 * 硬件(dji_c): USART1 PA9=TX / PB7=RX + DMA2_Stream5 循环, 921600 8N1。
 *              base.ioc 已按此配置好, 和旧工程 步兵云台26_10_17(自瞄) 完全一致。
 * 协议: HiPNUC 私有帧, 解码器用官方 hipnuc_dec.c(纯 C, 无 HAL 依赖)。
 *
 * 旧工程用法(对照): HI04 装在俯仰框上, 数据只给自瞄(Visual.c 发四元数),
 *                   pitch 闭环仍用 DM4310 自带反馈。本模块只负责"收+解析+对外提供数据",
 *                   具体给谁用由上层决定。
 */
#ifndef _MODULE_HI04_H_
#define _MODULE_HI04_H_

#include <stdint.h>

#ifndef HI04_TASK_STACK_SIZE
#define HI04_TASK_STACK_SIZE 1024 /* 任务栈大小 */
#endif
#ifndef HI04_TASK_PRIORITY
#define HI04_TASK_PRIORITY 11 /* 任务优先级 */
#endif
#ifndef HI04_TIMEOUT_MS
#define HI04_TIMEOUT_MS 50 /* 超过这么久没有新帧就认为掉线(正常 500Hz ⇒ 2ms/帧) */
#endif

/* 最新一帧 HI91 数据(与旧工程 HI04_Data_t 字段对齐) */
typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float acc_g[3];      /* 加速度 (g) */
    float gyro_rad_s[3]; /* 机体系角速度 (rad/s, 已由 deg/s 换算) */
    float quat[4];       /* W, X, Y, Z */

    uint32_t device_time_ms; /* 模块自身时间戳 */
    uint32_t frame_count;    /* 成功解析的 HI91 帧数 */
    uint32_t crc_errors;     /* 解码器累计 CRC 错误 */
    uint32_t invalid_frames; /* 解码器累计格式错误 */

    uint8_t received; /* 收到过有效帧; 注意这不等于"在线", 掉线检测要另外做 */
} Hi04_Data_t;

void Module_HI04_init(void);

/**
 * @brief 取最新数据
 * @return 有效数据指针; 还没收到过帧、或已超时掉线时返回 NULL
 * @note  带掉线检测: 超过 HI04_TIMEOUT_MS 没新帧就返回 NULL
 */
const Hi04_Data_t *Module_HI04_get(void);

/**
 * @brief 取角速度数组首地址(gyro_rad_s[0..2])
 * @return 永远非 NULL 的常量指针, 供电机 other_speed_feedback_ptr 长期持有
 * @note  ⚠ 这个指针只保证地址有效, **不保证数据有效** —— 必须先判 Module_HI04_get() 非 NULL
 */
const float *Module_HI04_get_gyro_ptr(void);

#endif
