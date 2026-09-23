/*
 * @Author: AdoreElysia w2006825@qq.com
 * @Date: 2026-02-26 19:23:29
 * @LastEditors: shentianao 2755930351@qq.com
 * @LastEditTime: 2026-07-24 15:15:36
 * @FilePath: \bache\task\motor_task.c
 * @Description: 达妙 + MF9025电机驱动模块
 */
#include "motor_task.h"
#include "pid.h"
#include "can.h"
#include "tim.h"
#include "mf9025_motor.h"
#include <string.h>

/* ========== 全局变量 ========== */
static DAMIAO_Motor_t damiao_motor;
static uint32_t tim2_counter = 0;  // TIM2计数器1ms周期

/* ========== MF9025电机列表 ========== */
static MF9025_Motor_t mf9025_motor_list[MF9025_MOTOR_MAX];  // MF9025电机列表

/**
  * @brief  电机任务初始化
  * @note   配置CAN并初始化达妙电机和MF9025电机参数
  */
void motor_task_init(void)
{    
    /* 配置CAN过滤器，接受所有ID */
    CAN_FilterTypeDef canfilterconfig;
    canfilterconfig.FilterIdHigh = 0x0000;
    canfilterconfig.FilterIdLow = 0x0000;
    canfilterconfig.FilterMaskIdHigh = 0x0000;
    canfilterconfig.FilterMaskIdLow = 0x0000;
    canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    canfilterconfig.FilterBank = 0;
    canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
    canfilterconfig.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);
    
    /* 开启CAN接收中断 */
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_Start(&hcan);
	
    HAL_Delay(50);
	
    /* 启动定时器2中断，周期1ms */
    HAL_TIM_Base_Start_IT(&htim2);
	
    /* 初始化TIM2计数器 */
    tim2_counter = 0;
	
    /* ========== 达妙电机初始化 ========== */
    memset(&damiao_motor, 0, sizeof(DAMIAO_Motor_t));
    
    /* 配置CAN参数 */
    damiao_motor.can_handle = &hcan;           // CAN句柄
    damiao_motor.can_tx_id = 0x01;             // CAN TX ID
    damiao_motor.can_rx_id = 0xf1;             // CAN RX ID
    damiao_motor.offline_timeout_ms = 50;      // 离线超时50ms

    /* 配置电机参数并初始化CAN通信 */
    DAMIAO_MotorInit(&damiao_motor);
	
    /* 默认失能电机，防止意外转动 */
    DAMIAO_MotorDisable(&damiao_motor);
	    
    /* ========== MF9025电机列表初始化 ========== */
    MF9025_MotorListInit(mf9025_motor_list);   // 初始化列表指针
    
    /* 注册第一个MF9025电机 */
    MF9025_MotorRegister(1, &hcan, 100);
    
    /* 注册第二个MF9025电机 */
    MF9025_MotorRegister(2, &hcan, 100);
}

/**
  * @brief  获取达妙电机指针
  * @retval 电机指针
  */
DAMIAO_Motor_t* get_damiao_motor_ptr(void)
{
    return &damiao_motor;
}

/**
  * @brief  获取MF9025电机指针
  * @retval 电机指针
  */
MF9025_Motor_t* get_mf9025_motor_ptr(void)
{
    return mf9025_motor_list;
}

/**
  * @brief  CAN RX FIFO0消息接收回调
  * @note   流程：接收消息 -> 匹配ID -> 数据映射
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) {
        return;
    }
    
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK) {
        /* 达妙电机处理 */
        if ((rx_header.StdId & 0xFF) == (damiao_motor.can_rx_id & 0xFF)) {
            memcpy(damiao_motor.rx_data, rx_data, 8);
            DAMIAO_MotorDecode(&damiao_motor);
            return;
        }
        
        /* MF9025电机处理 */
        for (uint8_t i = 0; i < MF9025_GetMotorCount(); i++) {
            MF9025_Motor_t *motor = MF9025_GetMotor(i);
            if (motor != NULL && (rx_header.StdId & 0xFF) == (motor->can_rx_id & 0xFF)) {
                memcpy(motor->rx_data, rx_data, 8);
                MF9025_MotorDecode(motor);
                return;
            }
        }
    }
}

/**
  * @brief  CAN RX FIFO1消息接收回调
  * @note   流程：接收消息 -> 匹配ID -> 数据映射
  */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) {
        return;
    }
    
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK) {
        /* 达妙电机处理 */
        if ((rx_header.StdId & 0xFF) == (damiao_motor.can_rx_id & 0xFF)) {
            memcpy(damiao_motor.rx_data, rx_data, 8);
            DAMIAO_MotorDecode(&damiao_motor);
            return;
        }
    }
}

/**
  * @brief  电机定时器中断服务函数（TIM2 1ms周期调用）
  * @note   奇偶ms交替执行：
  *         - 奇数ms(1,3,5...): 发送MF9025查询指令(0x9C)
  *         - 偶数ms(0,2,4...): 发送MF9025控制指令(0xA1) + 执行达妙电机控制(每2ms)
  */
void motor_timer_routine(void)
{    
    if (tim2_counter % 2 == 1) {
        /* 奇数ms: 发送MF9025查询指令(0x9C) */
        MF9025_MotorGetStatus();
    } else {
        /* 偶数ms: 发送MF9025控制指令(0xA1) */
        MF9025_motor_control();
        
        /* 达妙电机每2ms执行一次控制 */
        damiao_motor_control();
    }
    
    /* TIM2计数器递增 */
    tim2_counter++;
    if (tim2_counter >= 1000) {
        tim2_counter = 0;  // 每1000ms复位
    }
}
