/*
 * @brief 过洞步兵(双板:云台板 + 底盘板)机器人定义 —— 骨架
 *
 * 本文件只放【可从旧工程源码直接核对】的参数,来源:
 *   C:\Users\A\Desktop\过洞步兵\ 底盘26_10_11(拨盘单发控制) / 步兵云台26_10_11(加陀螺仪)
 *   User/Parameter.h、Motor/M3508.h、Motor/M2006.h、Motor/DM_J4310.h、
 *   CarBody/Mecanum.c、CarBody/Dial.c、CarBody/Gimbal.h、Hardware/Remote.c
 * 单位统一到框架口径:rad / rad/s / m / N·m
 */
#ifndef _GUODONG_DEF_H_
#define _GUODONG_DEF_H_

#include <stdint.h>

/* ================================================================
 * 机械 / 电气参数(数值有源码出处,非估算)
 * ================================================================ */
// clang-format off

/* -- 云台 -- */
/* ⚠️ 云台正前方的 Yaw 编码器参考: 云台板 Gimbal.h 给的是 3641, 底盘板 Parameter.h 给的是 3629 ——
 * 两板不一致, 而它直接决定跟随模式的机头偏角, 必须上台架标定取一个真值。 */
#define YAW_CHASSIS_ALIGN_ECD        3641
/* ⚠️ 俯仰限位取自旧 Gimbal.c 里【实际生效】的 PitchDownLimt / PitchUpLimt;
 * 旧 User/Parameter.h 的 Pitch_Angle*Linit(0.13 / -0.67) 没有任何调用者, 是过期残留, 不要用。
 * 限位作用于"反馈角 + PITCH_ZERO_RAD"之后的绝对角 ⇒ 换枪管 / 换电机后必须重标。 */
#define PITCH_LIMIT_DOWN_RAD         (-0.72f)  /* 旧 PitchDownLimt ≈ -41.3° */
#define PITCH_LIMIT_UP_RAD           (-1.45f)  /* 旧 PitchUpLimt  ≈ -83.1° */
#define PITCH_ZERO_RAD               0.354302f /* DM4310ZeroPitch */
#define GIMBAL_PITCH_STICK_RATE      5.0f      /* Gimbal_LeverSpeedMapRate */
#define GIMBAL_YAW_PITCH_RATE        1.5f      /* Gimbal_YawPitchSpeedRate */
#define GIMBAL_PITCH_FF_NM           (-0.85f)  /* 旧 Gimbal.c 常数前馈力矩 (N·m) */
/* -- yaw 闭环(角度环串速度环), 增益全部由旧 Gimbal.c 换算 -- */
/* 角度环: 旧 Kp = 0.55 [输出 rad/s per 度误差]、限幅 15.71 rad/s;
 * 框架角度环 measure 是 rad ⇒ Kp = 0.55 × (180/π) ≈ 31.5 [rad/s per rad] */
#define GIMBAL_YAW_ANGLE_KP          31.5f
#define GIMBAL_YAW_ANGLE_MAX_W       15.71f /* 角度环输出上限(rad/s), 即最大目标角速度 */
/* 速度环: 旧 Kp = 4000 [raw per rad/s]、限幅 16384 raw = 满电流;
 * 16384 raw ↔ TMAX_GM6020(2.223 N·m) ⇒ Kp = 4000 × 2.223/16384 ≈ 0.543 [N·m per rad/s] */
#define GIMBAL_YAW_SPEED_KP          0.543f
/* 摇杆满杆时【每个控制周期】累加的目标角增量(rad), 照搬旧工程节奏:
 * LeverSpeedMapRate(5.0) × YawPitchSpeedRate²(1.5²) × (180/4096) = 0.4944 度/周期
 * ⇒ 0.008628 rad / 2ms ⇒ 满杆约 4.3 rad/s (247°/s)
 * ⚠ 绑定 2ms 任务周期, 改周期要同步改。 */
#define GIMBAL_YAW_STICK_STEP_RAD    0.008628f

/* -- pitch 闭环(角度环串速度环) -- */
/* 速度环: 旧 Gimbal.c Kp=0.45、限幅 DM_J4310_TMAX。旧工程输出直给 MIT 力矩(N·m),
 * measure 是 DM4310 输出轴 rad/s ⇒ 已经是 N·m/(rad/s) 口径, 不用换算。
 * 方向已实测确认(ref/spd 同向、负反馈), 用回旧工程值。 */
#define GIMBAL_PITCH_SPEED_KP        0.30f
/* 积分: 旧工程 Ki=0。实测 Ki=0.05 时低速有"一顿一顿"的临界感, 且重力已由
 * GIMBAL_PITCH_FF_NM 前馈承担, 静态误差不是主要矛盾 ⇒ 先关积分, 需要再开。 */
#define GIMBAL_PITCH_SPEED_KI        0.0f
#define GIMBAL_PITCH_INTEGRAL_LIMIT  2.0f
/* 摇杆满杆对应的 pitch 目标角速度(rad/s)。阶段3(只速度环)用, 上角度环后改用累加目标角 */
#define GIMBAL_PITCH_TEST_W_MAX      3.0f
/* 角度环: 旧 Gimbal.c 就是 Kp=45 / MaxOut=12.566 / Ki=0 / Kd=0, 且它的闭环也在
 * TIM6 里以 **2ms(500Hz)** 跑, 速度和本工程一样 —— 前提是两个环都用电机自己的
 * Angle/Speed。之前 Kp=45 会震, 是因为当时内环订阅的是 HI04(装在框体上, 与电机
 * 之间隔一层柔性), 属于非共位反馈, 不是增益的问题。
 * 现在内环改回电机 Speed 后就可以直接用旧工程值: 位置刚度 = 45 × 0.45 = 20 N·m/rad。
 * 若上电后出现高频"嗡"震, 依次退 30 → 20 (我们的 angle 环比旧工程多 0~2ms 任务间延迟)。 */
#define GIMBAL_PITCH_ANGLE_KP        45.0f
#define GIMBAL_PITCH_ANGLE_MAX_W     12.566f
/* 【临时对比试验】pitch 角度反馈换成 HI04(装在俯仰框上的外置 IMU), 不用 DM4310 编码器:
 * 编码器在电机侧, 中间隔 10:1 减速 + 联轴器回差, 框体真实角度未必跟得上指令。
 * HI04 的欧拉角是 ±180° 卷绕的单圈角, 坐标系也和电机 total_angle 不同, 所以代码在
 * 拿到第一帧有效数据时记下"HI04角 - 电机角"的偏置, 之后把 HI04 角平移对齐到电机坐标系,
 * 已有的 pitch_target_rad / PITCH_LIMIT_* 继续有效。
 * ⚠ 安装朝向未确认时先看日志: fb= 应跟 ang= 同向同幅变化。反了就把 SIGN 改 -1.0f;
 *    如果 r= 不动而 p= 跟着动, 把 ROLL 改成 0。 */
#define PITCH_HI04_FB_SIGN           1.0f
#define PITCH_HI04_FB_ROLL           1 /* 1 = 用 roll_deg, 0 = 用 pitch_deg */
/* 摇杆满杆时【每个周期】累加的目标角增量(rad)。旧工程 Gimbal_PitchLeverSpeedMapRate = 0.0015,
 * 且不看摇杆幅度; 我们乘上归一化值, 满杆 0.0015 rad/2ms = 0.75 rad/s(43°/s), 走完行程约 1.2s。 */
#define GIMBAL_PITCH_STICK_STEP_RAD  0.0015f
/* 【慢外环】用 HI04 框体角吃掉传动回差: corr += KI × dt × (目标角 − 框体角), 加到目标上。
 * 轻点摇杆的步进(0.5~0.9°)比回差(≈1.1°)还小, 光靠编码器环电机在回差内就"到位"了,
 * 框体不动 ⇒ 顿。慢环会一直补到框体真到位。
 * 带宽 = KI rad/s: 1.0 ≈ 0.16Hz, 必须远低于内环(≈3Hz)。调大=补得快但可能和内环打架。 */
#define PITCH_IMU_CORR_KI            1.0f
#define PITCH_IMU_CORR_LIMIT_RAD     0.15f /* 最多补 ±8.6°, 只够回差, 不允许跑偏 */
/* 【限位标定】车上的 total_angle 与旧工程绝对角差一个常数偏置(旧工程叫 DM4310ZeroPitch)。
 * 实测(俯仰头很沉, 断电就落到底部):
 *   机械下限: ang = -0.99  ↔ HI04 r = +14.3°
 *   抬到 ang = -1.45 时 HI04 r = -13.2°(Δang 0.46 rad ↔ Δr 27.5°, 1:1 对得上)
 *   手动能抬到的机械上限: r = -39.9° ⇒ 推算 ang ≈ -1.94(行程 0.95 rad / 54°)
 * 让机械下限对上旧工程 PitchDownLimt(-0.72): 偏置 = -0.72 - (-0.99) = +0.27, 取 0.29。
 * ⇒ 软限位 [-1.45-0.29, -0.72-0.29] = [-1.74, -1.01], 行程 0.73 rad(42°), 距机械顶还剩 11°。
 * 标定不对的症状: 上电后目标被夹、云台自己往上顶一下 ⇒ 把偏置往 +0.01 方向微调。 */
#define PITCH_ANGLE_OFFSET_RAD       0.29f

/* -- 底盘(全向轮; 旧工程文件名叫 Mecanum.c, 但实车是全向轮) -- */
#define CHASSIS_WHEEL_RADIUS_M       0.081f    /* Mecanum_WheelRadius = 8.1cm */
/* 实车: 40cm × 40cm 正方轮距, 轮子 ±45° 【切向 X 型】布局。
 * 逆解统一用模块层 Chassis_Omni_Calc(自转项 vw*chassis_radius, 四轮同号), 见 modules/algorithm/chassis_type.c */
#define CHASSIS_WHEEL_BASE_X_M       0.40f     /* 前后轮距 (m) */
#define CHASSIS_WHEEL_BASE_Y_M       0.40f     /* 左右轮距 (m) */
/* 轮子中心到几何中心的距离 = √((x/2)²+(y/2)²) = √(0.2²+0.2²) = 0.2828 m。
 * 正方轮距时它等于 (a+b)*0.7071, 也就是切向布局逆解里的自转力臂。 */
#define CHASSIS_RADIUS_M             0.2828f
/* ⚠️ 15.7647 = 268/17, 是旧 M3508.h 里的【实车】值; 旧工程同时还留着标准齿箱的 3591/187 ≈ 19.203。
 * 它同时决定麦轮逆解系数和转矩常数, 上台架前必须实测确认实车用的是哪套齿箱。 */
#define CHASSIS_GEAR_RATIO           15.7647f
/* ⚠️ 旧 Mecanum.c 实际生效的摇杆映射是 (raw/784)*3.0 再按半径 3.0 做圆形限幅;
 * 旧 Parameter.h 的 Mecanum_LeverSpeedMapRate = 1.2/660 没有任何调用者, 不是满杆速度。 */
#define CHASSIS_MAX_SPEED_MPS        2.0f
#define CHASSIS_SPIN_W_RAD_S         1.0f      /* 旧 Mecanum_GyroScopeAngularVelocity(小陀螺, rad/s) */
/* 速度 → 轮速(转子 RPM)换算, 旧 Mecanum_InverseMotionControl 同款:
 *   RPM(转子) = (vx ± vy + wz*(a+b)) × 0.7071 / 0.081 × 15.7647 × 60/(2π) = raw × 1314
 *   纯前进 3.0 m/s      → 3943 RPM  (M3508 空载 ≈ 9000 RPM, 44%)
 *   斜推满杆(圆形限幅后) → 5576 RPM  (63%)
 *   小陀螺 5 rad/s      → 2629 RPM  (29%)
 *   小陀螺 + 满前进     → 6572 RPM  (73%)  ← 最坏工况, 仍低于空载转速 */

/* 四轮速度环: 输入【转子 rad/s】, 输出【输出侧 N·m】。
 * 旧工程是 RPM/原始电流口径: out_raw = 11.8 * err_RPM, 限幅 ±15000;
 * 换算到框架: 1 raw = TMAX_M3508/16384 = 0.000301 N·m(输出侧), 1 RPM = 0.10472 rad/s
 *   Kp = 11.8 * 0.000301 / 0.10472 ≈ 0.034
 * ⚠️ 必须非零: Kp=0 或 MaxOut=0 都会让 PID 输出恒为 0(轮子一点都不转),
 *    MaxOut 用 TMAX_M3508(max_torque), 否则 f_Output_Limit 会直接夹到 0。 */
#define CHASSIS_WHEEL_SPEED_KP       0.034f
#define CHASSIS_WHEEL_SPEED_KI       0.0f
#define CHASSIS_WHEEL_SPEED_KD       0.0f

/* -- 发射 -- */
#define REDUCTION_RATIO_LOADER       36.0f     /* M2006 减速比 */
/* ⚠️ 43° 是旧 Dial.c 的实测值(【输出轴】角), 而 NUM_PER_CIRCLE=6 理论上一发是 60° ——
 * 两者不可能同时成立: 若真是 6 孔, 用 43° 判据每发会累积 2° 偏差(8 发 16°), 旧代码也没有复位机制。
 * 首次上台架必须实测"一发对应多少输出轴角", 再 ×REDUCTION_RATIO_LOADER 换成框架的转子角。 */
#define ONE_BULLET_DELTA_DEG         43.0f
#define NUM_PER_CIRCLE               6
/* ⚠️ 4200 来自旧 Parameter.h 的 Gimbal_FrictionWheelSpeed(无调用者);
 * 旧 Gimbal.c 实际下发的是 ±5200 RPM(转子口径, 左负右正), 这里按实际值取 5200。 */
#define FRICTION_WHEEL_RPM           5200.0f
#define LOADER_SHOOT_RPM             6480.0f   /* Dial.c 单发/连发转子目标转速 */

/* 遥控(SBUS 25 字节帧, 11 位通道值): 摇杆量程/三档开关值直接用 module_remote.h 里的
 * SBUS_CHX_BIAS(中位 1024) / SBUS_CHX_UP(240) / SBUS_CHX_DOWN(1807) —— 与步兵三一致,
 * 不在这里另定义一套。⚠️ 三档开关的真实值首次上电必须看日志里的 sw= 核对。 */

/* -- 电机转矩常数(转子侧, N·m/A)--
 * M3508 输出侧 = 0.3 * (268/17)/(3591/187) = 0.2463 => 转子侧 0.2463/15.7647 = 0.01562 */
#define KT_M3508_ROTOR               0.01562f
#define KT_M2006_ROTOR               0.005f    /* 输出侧 0.18 / 36 */
#define KT_GM6020                    0.741f
#define KT_DM4310                    0.093f
/* 输出侧力矩上限 */
#define TMAX_M3508                   4.93f     /* 20A * 0.2463 */
#define TMAX_M2006                   1.8f      /* 10A * 0.18 */
#define TMAX_GM6020                  2.223f    /* 3A * 0.741 */
#define TMAX_DM4310                  10.0f     /* DM_J4310.h TMAX */

// clang-format on

/* ================================================================
 * 控制模式(沿用框架模板枚举)
 * ================================================================ */
#pragma pack(1)

typedef enum
{
    gimbal_zero_force = 0,
    gimbal_gyro_mode,
    gimbal_auto_mode,
} gimbal_mode_e;

typedef struct
{
    float         yaw;
    float         pitch;
    uint8_t       auto_search;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_t;

typedef enum
{
    shoot_off = 0,
    shoot_on,
} shoot_mode_e;

typedef enum
{
    friction_off = 0,
    friction_on,
} friction_mode_e;

typedef enum
{
    load_stop = 0,
    load_reverse,
    load_1_bullet,
    load_burstfire,
} loader_mode_e;

typedef struct
{
    shoot_mode_e    shoot_mode;
    loader_mode_e   load_mode;
    friction_mode_e friction_mode;
} Shoot_Ctrl_Cmd_t;

typedef enum
{
    chassis_zero_force = 0,
    chassis_follow_gimbal_yaw,
    chassis_rotate,
    chassis_rotate_reverse,
} chassis_mode_e;

typedef struct
{
    float          vx;           /* 云台系速度 (m/s): 底盘板再用 offset_angle 旋转到底盘系 */
    float          vy;           /* 云台系速度 (m/s) */
    float          wz;           /* 自转角速度 (rad/s); 小陀螺/跟随由底盘板按 chassis_mode 决定 */
    float          offset_angle; /* 云台与底盘夹角 (度) */
    chassis_mode_e chassis_mode;
} Chassis_Ctrl_Cmd_t;

#pragma pack()

/* ================================================================
 * 【板间帧:设计提案,尚未确认】
 * 旧工程用 4 个单向 CAN ID(0x149 摇杆 / 0x189 状态+开关 / 0x109 未用 / 0x119 拨弹许可),
 * 且 底盘->云台 方向未实现。下面是"合并为 2 帧 8B"的提案,先别依赖:
 * 字段取舍、模式编码、是否沿用其它 ID,都应由你确认后再定。
 * ================================================================ */
#define GD_WZ_UNIT_RAD_S        0.1f /* wz_x10 * 0.1 = rad/s(提案) */
#define GD_G2C_FLAG_MODE_MASK   0x07u
#define GD_G2C_FLAG_SHOOT_PERM  0x08u
#define GD_G2C_FLAG_FRICTION_ON 0x10u
#define GD_G2C_FLAG_SPIN        0x20u
#define GD_G2C_FLAG_BURST       0x40u

#pragma pack(1)
typedef struct
{
    int16_t vx_mmps;
    int16_t vy_mmps;
    int8_t  wz_x10;
    int16_t offset_ecd;
    uint8_t flags;
} Gd_GimbalToChassis_t;
_Static_assert(sizeof(Gd_GimbalToChassis_t) == 8, "must be 8 bytes");

typedef struct
{
    uint8_t  robot_color;
    uint8_t  game_progress;
    uint16_t current_hp;
    uint16_t bullet_allow;
    uint16_t shooter_heat_pct;
} Gd_ChassisToGimbal_t;
_Static_assert(sizeof(Gd_ChassisToGimbal_t) == 8, "must be 8 bytes");
#pragma pack()

#endif /* _GUODONG_DEF_H_ */
