#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32f20x.h"

typedef enum
{
    MOTOR_L = 0,    /* �󷽵�� */
    MOTOR_R,        /* �ҷ���� */
    MOTOR_NUM       /* ������� */
} which_motor_e;

void motor_driver(which_motor_e motor, int16_t pwm);
void motor_driver_all(int16_t pwm[4]);

#endif
