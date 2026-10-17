# infantry_guodong(过洞步兵 · 双板)—— 骨架

把 `C:\Users\A\Desktop\过洞步兵` 的两个 **STM32F407 标准库(StdPeriph)Keil 工程**接入本框架的**目录骨架**。

- **硬件**:两块 **大疆 C 板**(STM32F407IGHx),与框架 `board/dji_c` 一致
- **板型**:`gimbal`(云台板)/ `chassis`(底盘板)
- **电机清单与 CAN 分配**:见 [MOTORS.md](MOTORS.md)(数值均来自旧工程源码)

## 骨架包含什么 / 不包含什么

**已就位**
- 目录结构、`robot.cmake`(两个板型的模块开关与参数)、`guodong_def.h`(可核对的机械/电气参数)
- 各电机的**初始化**(总线 / tx_id / 减速比 / 转矩常数 / 力矩上限 / 闭环类型 / 反馈源)
- 两个板型的 `robot_control.c` **线程骨架**(2ms 调用链)
- 安全默认行为:所有执行器上电即停机;控制入口一律给零力

**未实现(TODO,故意留空)**
- 所有 PID 增益(统一留 `0.0f`,旧增益是 RPM/角度制口径,必须重标定)
- `motor_reverse_flag` / `feedback_reverse_flag`(按实车标定)
- 遥控映射(`RemoteControlSet`)、麦轮解算调用、跟随/小陀螺、云台闭环、摩擦轮
- 拨盘单发状态机(旧算法说明写在 `chassis_board/shoot_func/shoot_func.c` 注释里)
- 板间帧的填充/解析(帧格式本身也是**提案**,见 `guodong_def.h` 的【提案】段)
- 视觉自动模式、裁判系统真值(旧工程两板均未启用)

## 构建

```bash
cmake -S board/dji_c -B build/gd_gimbal  --preset Debug -DROBOT=infantry_guodong -DBOARD=gimbal  && cmake --build build/gd_gimbal -j
cmake -S board/dji_c -B build/gd_chassis --preset Debug -DROBOT=infantry_guodong -DBOARD=chassis && cmake --build build/gd_chassis -j
```

## 目标数据流(待实现)

```
遥控(SBUS huart3) + IMU/INS ─► 云台板 robot_control(2ms) ─► gimbal_func / shoot_func
                                        └─ 0x310 ─► CAN2 ─► 底盘板 robot_control(2ms)
                                                                 ├─ chassis_func(4xM3508 全向轮)
                                                                 ├─ shoot_func(M2006 拨盘)
                                                                 └─ 0x311 ─► CAN2(回传)
```

## 旧工程 -> 新框架对应

| 旧工程 | 新位置 |
|---|---|
| 云台 `main.c`/`Gimbal.c`(TIM6 2ms) | `gimbal_board/robot_control.c` + `gimbal_func/` |
| 底盘 `main.c`/`Mecanum.c`/`Dial.c` | `chassis_board/robot_control.c` + `chassis/` + `shoot_func/` |
| `Function/CToC.c`(4 个单向 CAN ID) | `module_boardcomm`(0x310/0x311,**提案待确认**) |
| `Function/LinkCheck.c`+`Warming.c` | `modules/OFFLINE` |
| `Hardware/Remote.c`(USART3, SBUS) | `modules/REMOTE`(`REMOTE_SOURCE=1`) |
| `INS/*` + `Hardware/BMI088.c` | `modules/INS` + `modules/BMI088` |
| `Control/PID.c` | `modules/algorithm/pid.c`(改善项位掩码不同,增益需重标定) |
| `User/Parameter.h` | `guodong_def.h` |

## 下一步(建议顺序)

1. 电机方向标定:逐个电机给固定 ref,确认转向与反馈符号
2. 遥控:用 `ulog` 打印 ch1~ch8,确认拨杆档位与通道含义
3. 增益标定:先轮速环,再跟随环,最后云台两轴
4. 拨盘单发:台架实测"一发对应的转子角"
5. 板间帧:确认字段设计后实现收发与离线安全
