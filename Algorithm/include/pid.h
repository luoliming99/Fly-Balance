#ifndef __PID_H
#define __PID_H

#include "ch32f20x.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float limit_integral;   /* 积分项限幅 */
    
    float error;            /* 当前偏差 */
    float integral;         /* 积分项 */
    float differ;           /* 微分项 */
    float pre_error;        /* 上一次偏差 */
    
    float out;
} pid_param_t;

void pid_postion_cal(pid_param_t *pid, float target, float measure);
void pid_incremental_cal(pid_param_t *pid, float target, float measure);

void balance_control(pid_param_t *pid, float target, float measure);
void speed_control(pid_param_t *pid, float target, float measure);
void turn_control(pid_param_t *pid, float target, float measure);

#endif
