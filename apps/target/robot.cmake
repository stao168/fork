# target 兵种 — 靶车 (F1 控制)
# 模块: OFFLINE(离线检测) + REMOTE(PS2 手柄)
# 说明: 电机(DJI / 达妙)后续再加, 现在先只验证 PS2 通道读取

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型覆盖默认值)
set(MODULES_SINGLE   OFFLINE REMOTE)

# OFFLINE 参数
set(OFFLINE_WATCHDOG_ENABLE 1)    # 开启看门狗
set(OFFLINE_BEEP_ENABLE     0)    # 调试阶段先不蜂鸣
set(OFFLINE_TASK_STACK_SIZE 1024) # 任务栈大小
set(OFFLINE_TASK_PRIORITY   6)    # 任务优先级

# REMOTE 参数(PS2 无线手柄)
set(REMOTE_SOURCE           3)      # 3=ps2
set(REMOTE_VT_SOURCE        0)      # 不接图传
set(REMOTE_DEAD_ZONE        10)     # 死区
# PS2 引脚 (纯 GPIO bit-bang, 由模块自己翻转; 引脚对应关系见 ps2.h)
#   CLK -> PA5      DO/CMD -> PA7      DI/DAT -> PA6      CS/SEL -> PA1
set(REMOTE_PS2_CLK_PORT GPIOA)
set(REMOTE_PS2_CLK_PIN  GPIO_PIN_5)
set(REMOTE_PS2_DO_PORT  GPIOA)
set(REMOTE_PS2_DO_PIN   GPIO_PIN_7)
set(REMOTE_PS2_DI_PORT  GPIOA)
set(REMOTE_PS2_DI_PIN   GPIO_PIN_6)
set(REMOTE_PS2_CS_PORT  GPIOA)
set(REMOTE_PS2_CS_PIN   GPIO_PIN_1)
# 注意: 这 4 个脚必须在板级 CubeMX 里配好 —— CLK/DO/CS = 推挽输出(初始高), DI = 上拉输入

# f103_c8: SPI2 的三根线已改成普通 GPIO 给 PS2 用, CubeMX 不再生成 spi.c/h,
# 而 board/bsp/SPI/bsp_spi.c 无条件 #include "spi.h" → 关掉 BSP SPI 外设。
# 换到别的板子(仍用 SPI 外设)时删掉这一行。
set(BSP_SPI_ENABLE OFF)
