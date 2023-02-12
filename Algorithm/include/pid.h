#ifndef __PID_H
#define __PID_H

#include "ch32f20x.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float limit_integral;   /* �������޷� */
    
    float error;            /* ��ǰƫ�� */
    float integral;         /* ������ */
    float differ;           /* ΢���� */
    float pre_error;        /* ��һ��ƫ�� */
    float ppre_error;       /* ���ϴ�ƫ�����PID�ã� */
    
    float out;
} pid_param_t;

void balance_control(pid_param_t *pid, float target, float measure);
void speed_control(pid_param_t *pid, float target, float measure);
void turn_control(pid_param_t *pid, float target, float measure);

#endif
