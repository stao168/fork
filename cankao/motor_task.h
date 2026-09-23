#ifndef __MOTOR_TASK_H
#define __MOTOR_TASK_H

#ifdef __cplusplus
extern "C" {
#endif

#include "damiao_motor.h"
#include "mf9025_motor.h"

/* ========== ���������� ========== */
#define DAMIAO_MOTOR_MAX  1
#define MF9025_MOTOR_MAX  2

#define PS2_STICK_DEADZONE     8      // ҡ�˹�����������8 ����Ϊ����
#define RAMP_RATE_MAX_RPS_S    2.0f   // ҡ������ʱ��ÿ��Ӽ����� rps
#define DAMIAO_REF_MAX         1.0f
/* ========== API�������� ========== */
void motor_task_init(void);
void motor_timer_routine(void);
DAMIAO_Motor_t* get_damiao_motor_ptr(void);
MF9025_Motor_t* get_mf9025_motor_ptr(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_TASK_H */
