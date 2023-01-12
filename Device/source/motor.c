#include "motor.h"
#include "bsp_pwm.h"

#define __MOTOR_STOP_MAX_VAL    50      /* �����ֹ����ֵ */
#define __MOTOR_STAT_MIN_VAL    650     /* ���ת������ֵ */

/******************************************************************************/
void motor_driver(which_motor_e motor, int16_t pwm)
{
    /* ���ݵ�����ԣ���PWMռ�ձ���ת�� */
    if (pwm < -__MOTOR_STOP_MAX_VAL) {
        pwm -= __MOTOR_STAT_MIN_VAL + __MOTOR_STOP_MAX_VAL;
    } else if (pwm > __MOTOR_STOP_MAX_VAL) {
        pwm += __MOTOR_STAT_MIN_VAL - __MOTOR_STOP_MAX_VAL;
    }

    if (pwm < -1000)
    {
        pwm = -1000;
    } 
    else if (pwm > 1000)
    {
        pwm = 1000;
    }
    if (pwm > 0)    /* ��ת */
    {
        switch (motor)
        {
            case MOTOR_L: PWM_CH1_SET(0); PWM_CH2_SET(pwm); break;
            case MOTOR_R: PWM_CH4_SET(0); PWM_CH3_SET(pwm); break;
            default: break;
        }
    }
    else            /* ��ת */ 
    {
        switch (motor)
        {
            case MOTOR_L: PWM_CH1_SET(-pwm); PWM_CH2_SET(0); break;
            case MOTOR_R: PWM_CH4_SET(-pwm); PWM_CH3_SET(0); break;
            default: break;
        }
    }
}

/******************************************************************************/
void motor_driver_all(int16_t pwm[2])
{
    motor_driver(MOTOR_L, pwm[MOTOR_L]);
    motor_driver(MOTOR_R, pwm[MOTOR_R]);
}
