# REMOTE 遥控/图传模块文档

## 概述

REMOTE 模块统一封装**遥控器**（SBUS / DT7 / PS2）和**图传键鼠**（VT02 / VT03）。

## 架构设计

### 数据流

```
┌──────────────┐    ┌──────────────┐
│ REMOTE_UART  │    │REMOTE_VT_UART│
│ (huart5/3)   │    │ (huart1/6)   │
└──────┬───────┘    └──────┬───────┘
       │ DMA               │ DMA
       │ (IDLE 中断)       │ (IDLE 中断)
       ▼                   ▼
  BSP_UART_Read()    BSP_UART_Read()
       │                   │
       ▼                   ▼
  sbus_decode()       vt02_decode()
      or                    or
  dt7_decode()        vt03_decode()
      or
  ps2_decode()  ← 唯一的非 UART 子源: 主机主动轮询 SPI, 没有 IDLE 中断
       │                    │
       │  直接写入          │  直接写入
       ▼                    ▼
  ┌──────────────────────────────────┐
  │         g_remote_data            │
  │  Remote_Data_t (唯一全局实例)    │
  │  channels[16] / wheel / mouse …  │
  └──────────────────────────────────┘
       ▲
       │ const Remote_Data_t *
       │
  ┌────┴─────────────────────────────┐
  │         上层应用                 │
  │  Module_Remote_get_channel(1..16)│
  │  Module_Remote_get_mouse()       │
  │  Module_Remote_get_keyboard()    │
  │  Module_Remote_get_wheel()       │
  └──────────────────────────────────┘
```

### 子模块封装模式

各子模块对外仅暴露 `_init` / `_decode` 两个函数：

```
          sbus.h
   ┌─────────────────┐
   │ sbus_init()     │  ← 配置 UART / 注册 Offline
   │ sbus_decode()   │  ← for 循环扫描, 直接写入 Remote_Data_t
   └─────────────────┘
          sbus.c
   ┌──────────────────────────────┐
   │ static UART_Device *uart;    │
   │ static uint8_t rx_buf[64];   │  DMA 接收缓冲区
   │ static bool initialized;     │
   │ static sbus_decode();        │
   └──────────────────────────────┘
```

### `Remote_Data_t` 统一结构体

| 字段 | 类型 | 说明 |
|------|------|------|
| `channels[0..3]` | `int16_t` | 摇杆通道，零偏后 ±660 范围 (SBUS/DT7/VT03/PS2) |
| `channels[4..15]` | `int16_t` | 扩展通道，原始值 0..2047 (仅 SBUS) |
| `wheel` | `int16_t` | 滚轮值 (DT7 / VT03) |
| `sw1` | `uint8_t` | 拨杆1 (DT7: 1=UP, 2=DOWN, 3=MID) |
| `sw2` | `uint8_t` | 拨杆2 (DT7) |
| `switch_pos` | `uint8_t` | 拨杆位置 (VT03) |
| `ps2` | `ps2_custom_t` | PS2 专属: 16 位按键位图(1=按下) + 原始摇杆 + 模式 |
| `mouse` | `mouse_state_t` | 鼠标状态 (DT7 / VT02 / VT03) |
| `keyboard` | `keyboard_state_t` | 键盘状态 (DT7 / VT02 / VT03) |

无效字段保持为 0。

### 各子模块参数

| 子模块 | 波特率 | 帧长 | rx_buf | offline 超时 | 校验 |
|--------|--------|------|--------|:---:|------|
| SBUS | 100000 | 25 字节 | 64 B | 50 ms | 帧头 0x0F + 帧尾 0x00 |
| DT7 | 100000 | 18 字节 | 64 B | 50 ms | 通道边界检查 (364..1684) |
| VT02 | 115200 | 不定长 | 128 B | 100 ms | CRC8 (帧头 5 字节) |
| VT03 | 921600 | 21 字节 | 64 B | 100 ms | 帧头 0xA9 0x53 + CRC16 |
| PS2 | SPI ≤250kHz | 9 字节 | — | 150 ms | `Data[1]` ∈ {0x73, 0x79} 且 `Data[2]==0x5A` |

## API 接口说明

```c
void Module_Remote_init(void);
```

```c
const Remote_Data_t *Module_Remote_get_data(void);
```

```c
int16_t Module_Remote_get_channel(uint8_t ch);
```

```c
const mouse_state_t *Module_Remote_get_mouse(void);
```

```c
const keyboard_state_t *Module_Remote_get_keyboard(void);
```

```c
int16_t Module_Remote_get_wheel(void);
```

```c
ps2_custom_t *Module_Remote_get_ps2_custom(void);
```

## 使用示例

### SBUS + VT02 方案

```c
#include "module_remote.h"

/* 修改 module_remote.h 中编译期配置:
 *   #define REMOTE_SOURCE     1   // sbus
 *   #define REMOTE_VT_SOURCE  1   // vt02
 */

void app_init(void)
{
    Module_Offline_init();   /* 先初始化离线检测 */
    Module_Remote_init();    /* 再初始化遥控/图传 */
}

void chassis_task(ULONG unused)
{
    while (1)
    {
        /* 读取摇杆通道 (零偏后) */
        int16_t ch1 = Module_Remote_get_channel(1);
        int16_t ch2 = Module_Remote_get_channel(2);
        int16_t ch3 = Module_Remote_get_channel(3);
        int16_t ch4 = Module_Remote_get_channel(4);

        chassis_control(ch1, ch2, ch3, ch4);
        tx_thread_sleep(10);
    }
}

void gimbal_task(ULONG unused)
{
    while (1)
    {
        const mouse_state_t *mouse = Module_Remote_get_mouse();
        if (mouse)
        {
            gimbal_update(mouse->mouse_x, mouse->mouse_y, mouse->mouse_z);
        }

        /* 检查键盘按键 */
        const keyboard_state_t *kb = Module_Remote_get_keyboard();
        if (kb && kb->bit.KEY_Q)
        {
            trigger_action();
        }

        tx_thread_sleep(5);
    }
}
```

### 直接获取整个数据结构

```c
void debug_task(ULONG unused)
{
    while (1)
    {
        const Remote_Data_t *data = Module_Remote_get_data();
        if (data)
        {
            LOG_I("ch1=%d ch2=%d ch3=%d ch4=%d wheel=%d",
                  data->channels[0], data->channels[1],
                  data->channels[2], data->channels[3],
                  data->wheel);
        }
        tx_thread_sleep(100);
    }
}
```

## 内部实现细节

### 摇杆通道零偏与死区

SBUS / DT7 / VT03 的摇杆通道原始值范围 364..1684，中间值 1024 为零位。解码后将值减去 1024 得到 ±660 范围，再通过 `REMOTE_DEAD_ZONE` (10) 去除零漂：

```c
ch -= DT7_CH_VALUE_OFFSET;        // 零偏
if (abs(ch) <= REMOTE_DEAD_ZONE)  // 死区
    ch = 0;
```

### PS2 子源 (REMOTE_SOURCE == 3)

与其它子源最大的区别: **其它子源是被动等 UART 中断, PS2 是主机主动轮询**。

- **时序**: SPI 模式3 (CPOL=High / CPHA=2Edge) + FirstBit = **LSB**；一帧 9 字节，CS 整帧保持低。
  init 里会用 `HAL_SPI_Init()` 重配 SPI（同 sbus 重配 UART 波特率的做法）。
- **上电配置序列**: ShortPoll×3 → 进配置 → 设模拟模式并锁定(`0x03`) → 退配置 → 读一帧校验 `Data[1]==0x73`；
  失败会重试 3 次。不进模拟模式的话摇杆读数无效，且按 MODE 键会切走模式。
- **轮询**: 任务循环里 `remote_ps2_decode()` 之后**必须** `tx_thread_sleep(REMOTE_PS2_POLL_INTERVAL_MS)`，
  否则没有阻塞点会 100% 占 CPU。
- **摇杆映射**: 原始 0~255（中位 ≈128）→ 减 128 + 死区 → 归一化到 **±660**，与 DT7/VT03 同量纲。
  上推为正；若实测反向，把 `REMOTE_PS2_INVERT_Y` 置 1。
- **按键**: 原始位图低有效(bit=0 表示按下)，驱动里**归一化成 1=按下**后存入 `ps2.buttons`；
  bit(n-1) 对应: 1 SELECT / 2 L3 / 3 R3 / 4 START / 5 ↑ / 6 → / 7 ↓ / 8 ← / 9 L2 / 10 R2 / 11 L1 / 12 R1 / 13 △ / 14 ○ / 15 × / 16 □。
- **振动**: poll 帧的 `[3]`/`[4]` 是右小/左大电机强度，当前固定 0（不震），留了口子。

## 注意事项

1. **初始化顺序** — `Module_Offline_init()` → `Module_Remote_init()`，否则子模块 Offline 注册失败。

2. **UART 波特率被子模块修改** — 子模块 init 会调用 `HAL_UART_Init()` 重配波特率（SBUS/DT7: 100000, VT02: 115200, VT03: 921600），确保 `REMOTE_UART` / `REMOTE_VT_UART` 不与其它外设共用。

3. **PS2 需要独占一条 SPI** — CPOL/CPHA/FirstBit 是 SPI **外设级**属性，不能按设备区分，
   `REMOTE_PS2_SPI` 不能与其它 SPI 设备共用，换用时需要给其中一个换总线。

4. **PS2 需要板级 CubeMX 配置** — 本模块不配引脚：需要一条 SPI（SCK/MISO/MOSI）+ 一个 CS GPIO（推挽输出，空闲高），
   接收器建议 3.3V 供电（5V 会把 DI 上拉到 5V，可能超 STM32 IO 耐压）。


## 版本历史

| 版本 | 日期 | 说明 |
|------|------|------|
| v1.0 | 2026-05-10 | 初始版本 |
| v1.1 | 2026-05-23 | 适配最新bsp_uart |
| v1.2 | 2026-05-24 | SBUS/DT7/VT03 改为 for 循环帧头扫描模式，支持多帧累积解码 |
| v1.3 | 2026-09-20 | 新增 PS2 无线手柄子源 (REMOTE_SOURCE=3, SPI 主动轮询) |
