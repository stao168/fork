#ifndef __DAMIAO_MOTOR_H
#define __DAMIAO_MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "pid.h"
#include "offline.h"
#include <stdint.h>

/* ========== 达妙电机模式定义 ========== */
#define DAMIAO_MIT_MODE        0x000

/* ========== 达妙电机命令 ========== */
typedef enum {
    DAMIAO_CMD_MOTOR_START   = 0xfc,  // 启动
    DAMIAO_CMD_MOTOR_STOP    = 0xfd,  // 停止
    DAMIAO_CMD_ZERO_POSITION = 0xfe,  // 设置零位
    DAMIAO_CMD_CLEAR_ERROR   = 0xfb   // 清除错误
} DAMIAO_Motor_Cmd_e;

/* ========== 达妙电机错误码 ========== */
typedef enum {
    DAMIAO_NO_ERROR               = 0x00,
    DAMIAO_OVERVOLTAGE_ERROR      = 0x08,
    DAMIAO_UNDERVOLTAGE_ERROR     = 0x09,
    DAMIAO_OVERCURRENT_ERROR      = 0x0A,
    DAMIAO_MOS_OVERTEMP_ERROR     = 0x0B,
    DAMIAO_MOTOR_COIL_OVERTEMP    = 0x0C,
    DAMIAO_COMMUNICATION_LOST     = 0x0D,
    DAMIAO_OVERLOAD_ERROR         = 0x0E,
} DAMIAO_Motor_Error_e;

/* ========== 电机测量值结构体 ========== */
typedef struct {
    uint8_t id;                    // 电机ID
    float position;                // 单圈位置 (-12.5 ~ 12.5 rad)
    float velocity;                // 速度 (-30 ~ 30 rad/s)
    float torque;                  // 扭矩 (-10 ~ 10 N·m)
    float mos_temp;                // MOS管温度
    float motor_temp;              // 电机线圈温度
    float total_angle;             // 总角度（多圈）
    int32_t total_round;           // 总圈数
    DAMIAO_Motor_Error_e error_code; // 错误码
} DAMIAO_Motor_Measure_t;

/* ========== 达妙电机结构体 ========== */
typedef struct {
    DAMIAO_Motor_Measure_t measure;    // 测量值
    
    /* 控制器部分 */
    PID_t speed_pid;                   // 速度环PID（输出所需扭矩）
    
    /* CAN通信配置 */
    CAN_HandleTypeDef *can_handle;     // CAN句柄
    uint32_t can_tx_id;                // CAN TX ID
    uint32_t can_rx_id;                // CAN RX ID
    CAN_TxHeaderTypeDef tx_header;     // CAN发送头
    CAN_RxHeaderTypeDef rx_header;     // CAN接收头
    uint8_t tx_data[8];                // 发送数据缓冲
    uint8_t rx_data[8];                // 接收数据缓冲
    
    /* 离线检测和状态 */
    uint8_t offline_index;             // Offline监督索引
    uint8_t offline_timeout_ms;        // 离线超时时间(ms)
    uint8_t stop_flag;                 // 停止标志
} DAMIAO_Motor_t;

/* ========== API函数声明 ========== */
void DAMIAO_MotorInit(DAMIAO_Motor_t *motor);
void DAMIAO_MotorSendCmd(DAMIAO_Motor_t *motor, DAMIAO_Motor_Cmd_e cmd);
void DAMIAO_MotorMitControl(DAMIAO_Motor_t *motor, float torque);
void DAMIAO_MotorDecode(DAMIAO_Motor_t *motor);
void DAMIAO_MotorEnable(DAMIAO_Motor_t *motor);
void DAMIAO_MotorDisable(DAMIAO_Motor_t *motor);
void damiao_motor_control(void);

#ifdef __cplusplus
}
#endif

#endif /* __DAMIAO_MOTOR_H */
