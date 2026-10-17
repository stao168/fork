# 机器人 & 板型 配置
# 在这里选择机器人和板型；配置文件是构建配置的唯一来源

# 目标机器人 & 板型
# ⚠️ 必须用 if(NOT DEFINED ...) 包起来: 直接 set() 会无条件盖掉 -DROBOT/-DBOARD 传进来的
# cache 变量, 于是出现"命令行改了扇型却仍然构出默认扇型"; 两块板硬件完全相同, 烧错看不出来。
# 交互式改这里; CI / 多板型构建用 -DROBOT=xxx -DBOARD=xxx 覆盖。
if(NOT DEFINED ROBOT)
    set(ROBOT "infantry_guodong") # hero / engineer / infantry3 / infantry4 / infantry5 / infantry_guodong / drone / sentry / darts / customcontrol / target / test
endif()
if(NOT DEFINED BOARD)
    set(BOARD "chassis") # single / gimbal / chassis; sentry has no single_board
endif()

# 板型校验
if(NOT BOARD MATCHES "^(single|gimbal|chassis)$")
    message(FATAL_ERROR "Unknown BOARD '${BOARD}', expected: single / gimbal / chassis")
endif()

# 加载默认模块配置
include(${CMAKE_CURRENT_LIST_DIR}/../modules/module_config.cmake)

# 加载机器人差异配置
include(${CMAKE_CURRENT_LIST_DIR}/${ROBOT}/robot.cmake)

# 派生变量
string(TOUPPER ${ROBOT} ROBOT_UPPER)
string(TOUPPER ${BOARD} BOARD_UPPER)

message(STATUS "Robot: ${ROBOT}")
message(STATUS "Board: ${BOARD}")

# 板型宏（只有一个为 1）
set(SINGLE_BOARD  0)
set(GIMBAL_BOARD  0)
set(CHASSIS_BOARD 0)
set(${BOARD_UPPER}_BOARD 1)

# 模块开关
foreach(_m OFFLINE REMOTE BMI088 INS REFEREE SUPERCAP WT606 MOTOR VISION BOARDCOMM LORA VOFA)
    set(MODULE_${_m} 0)
endforeach()

set(_enabled ${MODULES_${BOARD_UPPER}})
foreach(_m ${_enabled})
    set(MODULE_${_m} 1)
endforeach()
