#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32f20x.h"
#include "common.h"


typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_RUN,
} motor_status_e;

#if (PRODUCT == FLY)

typedef enum
{
    MOTOR_LF = 0,   /* ��ǰ����� */
    MOTOR_RF,       /* ��ǰ����� */
    MOTOR_LB,       /* ��󷽵�� */
    MOTOR_RB,       /* �Һ󷽵�� */
    MOTOR_NUM       /* ������� */
} which_motor_e;

#elif (PRODUCT == CAR)

typedef enum
{
    MOTOR_L = 0,    /* �󷽵�� */
    MOTOR_R,        /* �ҷ���� */
    MOTOR_NUM       /* ������� */
} which_motor_e;

#endif

void motor_driver(which_motor_e motor, int16_t pwm);
void motor_driver_all(int16_t pwm[MOTOR_NUM]);
void motor_stop_all(void);

#endif
