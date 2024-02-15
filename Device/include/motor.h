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
    MOTOR_LF = 0,   /* 左前方电机 */
    MOTOR_RF,       /* 右前方电机 */
    MOTOR_LB,       /* 左后方电机 */
    MOTOR_RB,       /* 右后方电机 */
    MOTOR_NUM       /* 电机个数 */
} which_motor_e;

#elif (PRODUCT == CAR)

typedef enum
{
    MOTOR_L = 0,    /* 左方电机 */
    MOTOR_R,        /* 右方电机 */
    MOTOR_NUM       /* 电机个数 */
} which_motor_e;

#endif

void motor_driver(which_motor_e motor, int16_t pwm);
void motor_driver_all(int16_t pwm[MOTOR_NUM]);
void motor_stop_all(void);

#endif
