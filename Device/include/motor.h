#ifndef __MOTOR_H
#define __MOTOR_H

#include "ch32f20x.h"
#include "common.h"

#ifdef FLY

typedef enum
{
    MOTOR_LF = 0,   /* 左前方电机 */
    MOTOR_RF,       /* 右前方电机 */
    MOTOR_LB,       /* 左后方电机 */
    MOTOR_RB,       /* 右后方电机 */
    MOTOR_NUM       /* 电机个数 */
} which_motor_e;

void motor_driver(which_motor_e motor, int16_t pwm);
void motor_driver_all(int16_t pwm[MOTOR_NUM]);
void motor_stop_all(void);

#else

typedef enum
{
    MOTOR_L = 0,    /* 左方电机 */
    MOTOR_R,        /* 右方电机 */
    MOTOR_NUM       /* 电机个数 */
} which_motor_e;

void motor_driver(which_motor_e motor, int16_t pwm);
void motor_driver_all(int16_t pwm[MOTOR_NUM]);
void motor_brake_all(uint16_t factor);

#endif

#endif
