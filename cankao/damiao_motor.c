#include "damiao_motor.h"
#include <string.h>
#include <math.h>

/* ========== 数据转换宏定义 ========== */
#define DAMIAO_P_MIN      -12.5f
#define DAMIAO_P_MAX      12.5f
#define DAMIAO_V_MIN      -45.0f
#define DAMIAO_V_MAX      45.0f
#define DAMIAO_T_MIN      -10.0f
#define DAMIAO_T_MAX      10.0f

/* ========== 全局电机指针 ========== */
static DAMIAO_Motor_t *g_motor = NULL;

/**
  * @brief  浮点数转换为无符号整数
  */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    return (uint16_t)((offset / span) * ((1 << bits) - 1));
}

/**
  * @brief  无符号整数转换为浮点数
  */
static float uint_to_float(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return (x / (float)((1 << bits) - 1)) * span + x_min;
}

/**
  * @brief  达妙电机初始化
  * @param  motor: 电机结构体指针（包含所有配置）
  * @note   motor中应该已配置：can_handle, can_tx_id, can_rx_id, offline_timeout_ms
  */
void DAMIAO_MotorInit(DAMIAO_Motor_t *motor)
{
    if (motor == NULL || motor->can_handle == NULL) {
        return;
    }
    
    g_motor = motor;
    
    /* 初始化CAN头 */
    motor->tx_header.StdId = motor->can_tx_id;
    motor->tx_header.IDE = CAN_ID_STD;
    motor->tx_header.RTR = CAN_RTR_DATA;
    motor->tx_header.DLC = 8;
    
    /* 注册Offline监督 */
    motor->offline_index = OFFLINE_Register("damiao_motor", motor->offline_timeout_ms, 1);
    
    /* 初始化测量值 */
    motor->measure.error_code = DAMIAO_NO_ERROR;
    
    /* 发送初始化命令 */
    DAMIAO_MotorSendCmd(motor, DAMIAO_CMD_CLEAR_ERROR);
    HAL_Delay(10);
    DAMIAO_MotorSendCmd(motor, DAMIAO_CMD_MOTOR_START);
    HAL_Delay(10);
}

/**
  * @brief  发送电机命令
  * @param  motor: 电机结构体指针
  * @param  cmd: 命令
  */
void DAMIAO_MotorSendCmd(DAMIAO_Motor_t *motor, DAMIAO_Motor_Cmd_e cmd)
{
    if (motor == NULL) {
        return;
    }
    
    uint32_t tx_mailbox;
    HAL_StatusTypeDef status;
    
    memset(motor->tx_data, 0xFF, 7);
    motor->tx_data[7] = (uint8_t)cmd;
    
    motor->tx_header.StdId = motor->can_tx_id + DAMIAO_MIT_MODE;
    
    status = HAL_CAN_AddTxMessage(motor->can_handle, &motor->tx_header, motor->tx_data, &tx_mailbox);
}

/**
  * @brief  MIT模式控制（只发送力矩）
  * @param  motor: 电机结构体指针
  * @param  torque: 目标力矩 (-10 ~ 10 N·m)
  */
void DAMIAO_MotorMitControl(DAMIAO_Motor_t *motor, float torque)
{
    if (motor == NULL) {
        return;
    }
    
    uint32_t tx_mailbox;
    uint16_t torque_tmp;
    HAL_StatusTypeDef status;
    
    /* 限制力矩范围 */
    if (torque > DAMIAO_T_MAX) torque = DAMIAO_T_MAX;
    if (torque < DAMIAO_T_MIN) torque = DAMIAO_T_MIN;
    
    /* 力矩编码为12位 (4+8) */
    torque_tmp = float_to_uint(torque, DAMIAO_T_MIN, DAMIAO_T_MAX, 12);
    
    motor->tx_header.StdId = motor->can_tx_id + DAMIAO_MIT_MODE;
    motor->tx_header.DLC = 8;
    
    /* MIT模式：位置、速度、Kp、Kd、力矩 */
    motor->tx_data[0] = 0;                      // 位置高字节
    motor->tx_data[1] = 0;                      // 位置低字节
    motor->tx_data[2] = 0;                      // 速度高4位
    motor->tx_data[3] = 0;                      // 速度低4位 + Kp高4位
    motor->tx_data[4] = 0;                      // Kp低字节
    motor->tx_data[5] = 0;                      // Kd高4位
    motor->tx_data[6] = (torque_tmp >> 8) & 0x0F;  // Kd低4位 + 力矩高4位
    motor->tx_data[7] = torque_tmp & 0xFF;     // 力矩低8位
    
    status = HAL_CAN_AddTxMessage(motor->can_handle, &motor->tx_header, motor->tx_data, &tx_mailbox);
}

/**
  * @brief  电机数据解码
  * @param  motor: 电机结构体指针
  */
void DAMIAO_MotorDecode(DAMIAO_Motor_t *motor)
{
    if (motor == NULL) {
        return;
    }
    
    uint8_t *rxbuff = motor->rx_data;
    
    /* 更新Offline时间戳 */
    OFFLINE_UpdateTime(motor->offline_index);
    
    /* 解析错误码 */
    uint8_t error_code = (rxbuff[0] >> 4) & 0x0F;
    motor->measure.id = rxbuff[0] & 0x0F;
    motor->measure.error_code = (error_code >= 0x08 && error_code <= 0x0E) 
                                ? (DAMIAO_Motor_Error_e)error_code 
                                : DAMIAO_NO_ERROR;
    
    /* 如果错误码是13，重新发送启动命令 */
    if (error_code == 13) {
        //DAMIAO_MotorSendCmd(motor, DAMIAO_CMD_CLEAR_ERROR);
        //DAMIAO_MotorSendCmd(motor, DAMIAO_CMD_MOTOR_START);
    }
    
    /* 解析位置 */
    float last_pos = motor->measure.position;
    uint16_t pos_tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    motor->measure.position = uint_to_float(pos_tmp, DAMIAO_P_MIN, DAMIAO_P_MAX, 16);
    
    /* 解析速度 */
    uint16_t vel_tmp = (uint16_t)((rxbuff[3] << 4) | (rxbuff[4] >> 4));
    motor->measure.velocity = uint_to_float(vel_tmp, DAMIAO_V_MIN, DAMIAO_V_MAX, 12);
    
    /* 解析力矩 */
    uint16_t torque_tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    motor->measure.torque = uint_to_float(torque_tmp, DAMIAO_T_MIN, DAMIAO_T_MAX, 12);
    
    /* 解析温度 */
    motor->measure.mos_temp = (float)rxbuff[6];
    motor->measure.motor_temp = (float)rxbuff[7];
    
    /* 计算多圈角度 */
    float delta_pos = motor->measure.position - last_pos;
    if (delta_pos > 6.25f) {
        motor->measure.total_round--;
    } else if (delta_pos < -6.25f) {
        motor->measure.total_round++;
    }
    motor->measure.total_angle = motor->measure.total_round * 25.0f + motor->measure.position;
}

/**
  * @brief  电机使能
  * @param  motor: 电机结构体指针
  */
void DAMIAO_MotorEnable(DAMIAO_Motor_t *motor)
{
    if (motor == NULL) {
        return;
    }
    
    motor->stop_flag = 0;
}

/**
  * @brief  电机停止
  * @param  motor: 电机结构体指针
  */
void DAMIAO_MotorDisable(DAMIAO_Motor_t *motor)
{
    if (motor == NULL) {
        return;
    }
    
    motor->stop_flag = 1;
}

/**
  * @brief  达妙电机控制函数（由motor_control_task调用）
  * @note   使用speed_pid计算所需扭矩，发送控制指令
  */
void damiao_motor_control(void)
{
    if (g_motor == NULL || g_motor->can_handle == NULL) {
        return;
    }
    
    DAMIAO_Motor_t *motor = g_motor;
    float control_output = 0.0f;
    
    /* 检查电机是否掉线或停止 */
    if (OFFLINE_IsOffline(motor->offline_index) == 1 || motor->stop_flag) {
        /* 掉线或停止：发送0力矩 */
        control_output = 0.0f;
    } else {
        /* 正常：使用speed_pid计算所需扭矩 */
        control_output = PID_Calculate(&motor->speed_pid);
    }
    
    /* 发送力矩控制指令 */
    DAMIAO_MotorMitControl(motor, control_output);
}
