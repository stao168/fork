#ifndef HI04_H
#define HI04_H

#include <stdint.h>

typedef struct
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float acc_g[3];          /* 加速度：G */
    float gyro_rad_s[3];     /* 机体系角速度：rad/s */
    float quat[4];           /* W、X、Y、Z */

    uint32_t device_time_ms;
    uint32_t frame_count;    /* 成功解析的 HI91 帧数 */

    uint8_t received;        /* 收到有效帧后置 1；不是超时判断 */
} HI04_Data_t;

typedef struct
{
    uint32_t crc_errors;
    uint32_t invalid_frames;
    uint32_t buffer_overruns; /* 缓冲覆盖事件数，不是丢失帧数 */
    uint32_t hardware_errors; /* UART/DMA 错误恢复次数 */
} HI04_Stats_t;

/* 与视觉一致的云台姿态，不覆盖 HI04 原始数据 */
typedef struct
{
    float yaw_rad;       /* 单圈 yaw，范围 (-PI, PI] */
    float pitch_rad;     /* 范围 [-PI/2, PI/2] */
    float roll_rad;      /* 范围 (-PI, PI] */
} HI04_VisualAttitude_t;

extern volatile HI04_Data_t HI04_Data;
extern volatile HI04_Stats_t HI04_Stats;

void HI04_Init(void);
void HI04_Poll(void);
int HI04_QuaternionToVisual(const float quat[4], HI04_VisualAttitude_t *result);

#endif
