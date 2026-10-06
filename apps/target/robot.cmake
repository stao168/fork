# target 兵种 — 靶车 (F1 控制)
# 模块: OFFLINE(离线检测) + REMOTE(PS2 手柄) + MOTOR(达妙 DM6220 + DJI M3508)
# 说明: 方向键控制转速 → M3508(L1 使能 + ↑↓ 前进/后退) / 达妙(R1 总开关 + △固定转速 / □油门模式 + ←→ 加减转速 + ×正反转)

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型覆盖默认值)
set(MODULES_SINGLE   OFFLINE REMOTE MOTOR)

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
# 这 4 个脚必须是本板上空着、且**已在板级 CubeMX 里配成普通 GPIO** 的脚:
#   CLK / DO / CS = 推挽输出(初始高), DI = 上拉输入
# 两个板子接的不是同一组脚, 不能照搬:
#   dji_c : CLK=PB13  DO=PB15  DI=PB14  CS=PB12   (原 SPI2 的 3 个脚 + 一个空脚;
#           用它们得先在 .ioc 里把 SPI2 关掉, 否则引脚归属 SPI2 外设)
#   105_rc: CLK=PA5   DO=PA7   DI=PA6   CS=PA1    (原 SPI1 的 3 个脚 + 一个空脚)
#           ⚠️ 105_rc 的 PB12/PB13 被 CAN2 占用了, 用不了 dji_c 那组
# (CMAKE_SOURCE_DIR 就是被 -S 指定的板级目录, 据此判断当前在编哪块板)
get_filename_component(_target_board ${CMAKE_SOURCE_DIR} NAME)
if(_target_board STREQUAL "105_rc")
    set(REMOTE_PS2_CLK_PORT GPIOA)
    set(REMOTE_PS2_CLK_PIN  GPIO_PIN_5)
    set(REMOTE_PS2_DO_PORT  GPIOA)
    set(REMOTE_PS2_DO_PIN   GPIO_PIN_7)
    set(REMOTE_PS2_DI_PORT  GPIOA)
    set(REMOTE_PS2_DI_PIN   GPIO_PIN_6)
    set(REMOTE_PS2_CS_PORT  GPIOA)
    set(REMOTE_PS2_CS_PIN   GPIO_PIN_1)
else()
    set(REMOTE_PS2_CLK_PORT GPIOB)
    set(REMOTE_PS2_CLK_PIN  GPIO_PIN_13)
    set(REMOTE_PS2_DO_PORT  GPIOB)
    set(REMOTE_PS2_DO_PIN   GPIO_PIN_15)
    set(REMOTE_PS2_DI_PORT  GPIOB)
    set(REMOTE_PS2_DI_PIN   GPIO_PIN_14)
    set(REMOTE_PS2_CS_PORT  GPIOB)
    set(REMOTE_PS2_CS_PIN   GPIO_PIN_12)
endif()

