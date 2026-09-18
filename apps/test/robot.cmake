# test 兵种 — 新模块接入测试骨架
# 用途: 新模块开发时在此单独验证 (见 robot_control.c 顶部工作流说明)
# 最小模块集: OFFLINE (测具体模块时, 把对应模块加入 MODULES_SINGLE)

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型覆盖默认值)
set(MODULES_SINGLE   OFFLINE NRF24L01)

# OFFLINE 参数
set(OFFLINE_BEEP_ENABLE     0)    # 测试兵种不蜂鸣

# NRF24L01 参数(收发两端必须一致)
set(NRF24L01_TX_INTERVAL_MS 10)    # 100Hz
set(NRF24L01_TX_ENABLE      0)     # 1=发送端
set(NRF24L01_RX_ENABLE      1)     # 1=接收端; 两者互斥, 只能开一个
set(NRF24L01_ADDR           "0x11,0x22,0x33,0x44,0x55")  # 5字节地址, 两端一致
# 注意: SPI2 与 CE/CSN/IRQ 引脚需在板级 CubeMX 配好(见 module_nrf24l01.h 硬件说明)