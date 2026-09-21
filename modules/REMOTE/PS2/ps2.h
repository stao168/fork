/**
 * @file    ps2.h
 * @brief   PS2 无线手柄 (WHEELTEC 2.4G 接收器, 4线同步串行) —— REMOTE 子源
 *
 *  协议: 主机主动轮询, 一帧固定 9 字节; 空闲 CLK=高, 下降沿输出 / 上升沿采样
 *        → 等价 SPI 模式3 (CPOL=High, CPHA=2Edge) + FirstBit = LSB
 *  实现: 纯 GPIO bit-bang(半周期 5us + 字节间隔 16us), 不占 SPI 外设, 4 个脚任意
 *  接线: CLK → 任意 GPIO   DO/CMD → 任意 GPIO   DI/DAT → 任意 GPIO(上拉输入)
 *        CS/SEL → 任意 GPIO(整帧 9 字节保持低)
 *  @note  4 个脚必须由**板级 CubeMX** 配好(与 REMOTE_PS2_* 宏一致), 模块不覆盖:
 *           CLK/DO/CS = 推挽输出, 初始高;  DI = 上拉输入
 *        接收器建议用 3.3V 供电(5V 会把 DI 上拉到 5V, 可能超 STM32 IO 耐压)
 *
 *  @note  调用方每帧之后必须自己 sleep 一个周期(主动轮询, 没有中断驱动)
 */
#ifndef _PS2_H_
#define _PS2_H_

#include "module_remote.h"
#include "module_offline.h"
#include <stdint.h>

/**
 * @brief PS2 初始化 (注册 offline); 握手(进模拟模式)推迟到解码任务
 * @note  引脚需已在板级 CubeMX 里配好(见文件头)
 * @param out_offline 输出离线设备句柄指针, NULL 则不输出
 * @return 0 成功, -1 失败
 */
int8_t remote_ps2_init(Offline_Device **out_offline);

/**
 * @brief PS2 轮询一帧并解码, 直接写入统一结构体
 * @note  必须在每次调用后 tx_thread_sleep(REMOTE_PS2_POLL_INTERVAL_MS)
 * @param data 统一遥控数据结构指针
 * @param offline 离线设备句柄 (用于心跳上报)
 */
void remote_ps2_decode(Remote_Data_t *data, Offline_Device *offline);

#endif // _PS2_H_
