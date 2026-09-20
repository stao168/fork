/**
 * @file    ps2.h
 * @brief   PS2 无线手柄 (WHEELTEC 2.4G 接收器, 4线同步串行) —— REMOTE 子源
 *
 *  协议: 主机主动轮询, 一帧固定 9 字节; 空闲 CLK=高, 下降沿输出 / 上升沿采样
 *        → SPI 模式3 (CPOL=High, CPHA=2Edge) + FirstBit = LSB
 *  接线: CLK→SCK   DO/CMD→MOSI   DI/DAT→MISO   CS/SEL→片选(整帧 9 字节保持低)
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
 * @brief PS2 初始化 (重配 SPI 为 mode3/LSB + 注册 offline + 进模拟模式并锁定)
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
