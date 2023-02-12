#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32f20x.h"

typedef enum
{
    MOTOR_L = 0,    /* 左方电机 */
    MOTOR_R,        /* 右方电机 */
    MOTOR_NUM       /* 电机个数 */
} which_motor_e;

void motor_driver(which_motor_e motor, int16_t pwm);
void motor_driver_all(int16_t pwm[4]);

#endif
