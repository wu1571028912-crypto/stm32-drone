#ifndef __PID_H
#define __PID_H

#include <stdint.h>



typedef struct
{
    float wc;       /* 控制器带宽 */
    float wo;       /* 观测器带宽 */
    float b0;       /* 被控对象增益 */

    float z1;       /* 被控量估计 */
    float z2;       /* 总扰动估计 */

    float u;        /* 实际输出 */
    float limit;    /* 输出物理限制 */

    uint8_t saturated;
} LADRC1_TypeDef;

typedef struct
{
    /* Roll角度外环 */
    LADRC1_TypeDef roll_angle;

    /* Roll角速度内环 */
    LADRC1_TypeDef roll_rate;

    /* Pitch角度外环 */
    LADRC1_TypeDef pitch_angle;

    /* Pitch角速度内环 */
    LADRC1_TypeDef pitch_rate;

    /* Yaw角速度内环：遥控器无Yaw通道时用于抑制原地旋转 */
    LADRC1_TypeDef yaw_rate;
} CascadeLADRC_TypeDef;


extern CascadeLADRC_TypeDef flight_ladrc;


void LADRC1_Init(
    LADRC1_TypeDef *adrc,
    float wc,
    float wo,
    float b0,
    float limit
);

void LADRC1_Reset(
    LADRC1_TypeDef *adrc,
    float measurement
);

float LADRC1_Update(
    LADRC1_TypeDef *adrc,
    float target,
    float measurement,
    float applied_input,
    float dt
);

void CascadeLADRC_Init(
    CascadeLADRC_TypeDef *controller
);

void CascadeLADRC_Reset(
    CascadeLADRC_TypeDef *controller,
    float roll_angle,
    float pitch_angle,
    float roll_rate,
    float pitch_rate,
    float yaw_rate
);

void CascadeLADRC_Update(
    CascadeLADRC_TypeDef *controller,
    float target_roll,
    float target_pitch,
    float target_yaw_rate,
    float actual_roll,
    float actual_pitch,
    float actual_roll_rate,
    float actual_pitch_rate,
    float actual_yaw_rate,
    float previous_applied_roll_pwm,
    float previous_applied_pitch_pwm,
    float previous_applied_yaw_pwm,
    float dt,
    float *roll_pwm,
    float *pitch_pwm,
    float *yaw_pwm
);		
#endif
		
		
