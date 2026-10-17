# 过洞步兵(双板:云台板 + 底盘板)
# 来源:标准库工程 C:\Users\A\Desktop\过洞步兵(底盘 / 步兵云台),STM32F407IGHx → 框架 board/dji_c
# 先加载默认模板,再覆盖差异

include(${CMAKE_CURRENT_LIST_DIR}/../../modules/module_config.cmake)

# 模块开关(按板型) —— 【阶段3: yaw 闭环已通, 开始接外置 IMU】
#   云台板:遥控(SBUS) + 板间下发 + BMI088/INS + MOTOR + HI04(俯仰框上的 HiPNUC HI91)
#           gimbal_func.c 里 GD_YAW_ONLY=1 => 只初始化 yaw(GM6020), pitch(DM4310) 不碰
#           HI04 现在只收+解析+打日志, 不进任何控制
#   底盘板:四轮全向轮 + 板间接收(无遥控、无 IMU)
# 后续: 用 HI04 的角速度做 pitch 反馈 / 给 VISION 做姿态源
set(MODULES_GIMBAL   OFFLINE REMOTE BMI088 INS MOTOR BOARDCOMM HI04)
set(MODULES_CHASSIS  OFFLINE MOTOR BOARDCOMM)

# ── OFFLINE ──
set(OFFLINE_WATCHDOG_ENABLE 1)   # 旧工程底盘未开狗、云台开 1.024s;统一交给 OFFLINE
set(OFFLINE_BEEP_ENABLE     1)

# ── REMOTE(旧工程:USART3 / 100000 波特 / 9bit+偶校验 / SBUS 25 字节 / 中位 1024) ──
set(REMOTE_UART         huart3)
set(REMOTE_SOURCE       1)       # 1=sbus(旧 Remote.c 校验 Data[0]==0x0F → SBUS)
set(REMOTE_VT_SOURCE    0)       # 旧工程图传/键鼠未启用
set(REMOTE_DEAD_ZONE    10)

# ── BMI088(C 板板载,旧工程 TIM10/PF6 加热恒温 40℃) ──
set(BMI088_TEMP_ENABLE  1)
set(BMI088_TEMP_SET     40.0)

# ── INS(旧 ins_task 1kHz,对应框架 INS 任务) ──
set(INS_TASK_PRIORITY   7)

# ── MOTOR ──
set(MOTOR_TASK_PRIORITY 12)

# ── BOARDCOMM ──
set(BOARDCOMM_CAN       BSP_CAN_HANDLE2)
