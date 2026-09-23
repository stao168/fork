# test 兵种 — 新模块接入测试骨架
# 用途: 新模块开发时在此单独验证 (见 robot_control.c 顶部工作流说明)
# 最小模块集: OFFLINE (测具体模块时, 把对应模块加入 MODULES_SINGLE)

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型覆盖默认值)
set(MODULES_SINGLE   OFFLINE REMOTE)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 测试兵种不蜂鸣

# PS2 用掉 SPI2 的三根线当普通 GPIO 后, 板级 CubeMX 不再生成 spi.c/h,
# 而 board/bsp/SPI/bsp_spi.c 无条件 #include "spi.h" → 关掉 BSP SPI 外设
set(BSP_SPI_ENABLE OFF)

# REMOTE 参数(测试 PS2 手柄)
set(REMOTE_SOURCE           3)      # 3=ps2
set(REMOTE_VT_SOURCE        0)      # 不接图传
# PS2 纯 GPIO bit-bang, 引脚由 module_remote.h 默认值给出
# (CLK=PB13 / DO=PB15 / DI=PB14 / CS=PA4), 要换板再在这里覆盖
# 注意: 这 4 个脚必须在板级 CubeMX 里配成 CLK/DO/CS=推挽输出(初始高) / DI=上拉输入