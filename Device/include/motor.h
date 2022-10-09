#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32f20x.h"

typedef enum
{
    MOTOR_LF = 0,   /* 左前方电机 */
    MOTOR_RF,       /* 右前方电机 */
    MOTOR_LB,       /* 左后方电机 */
    MOTOR_RB        /* 右后方电机 */
} which_motor_e;

void motor_driver(which_motor_e motor, int16_t pwm);
void motor_driver_all(int16_t pwm[4]);

#endif
