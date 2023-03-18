#include "motor.h"
#include "bsp_pwm.h"

#ifdef FLY

/******************************************************************************/
void motor_driver(which_motor_e motor, int16_t pwm)
{
    switch (motor)
    {
        case MOTOR_LF: PWM_CH1_SET(pwm); break;
        case MOTOR_RF: PWM_CH2_SET(pwm); break;
        case MOTOR_LB: PWM_CH3_SET(pwm); break;
        case MOTOR_RB: PWM_CH4_SET(pwm); break;
        default: break;
    }
}

/******************************************************************************/
void motor_driver_all(int16_t pwm[MOTOR_NUM])
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (pwm[i] < 0) {
            pwm[i] = 0;
        } else if (pwm[i] > 1000) {
            pwm[i] = 1000;
        }
        motor_driver((which_motor_e)i, pwm[i]);
    }
}

/******************************************************************************/
void motor_stop_all(void)
{
    motor_driver(MOTOR_LF, 0);
    motor_driver(MOTOR_RF, 0);
    motor_driver(MOTOR_LB, 0);
    motor_driver(MOTOR_RB, 0);
}

#else

#define __MOTOR_STAT_MIN_VAL    700     /* ���ת������ֵ */

static int16_t _real_pwm[MOTOR_NUM] = {0};

/******************************************************************************/
void motor_driver(which_motor_e motor, int16_t pwm)
{
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
    /* ���ݵ�����ԣ���PWMռ�ձ���ת�� */
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (pwm[i] < 0) {
            pwm[i] -= __MOTOR_STAT_MIN_VAL;
        } else if (pwm[i] > 0) {
            pwm[i] += __MOTOR_STAT_MIN_VAL;
        }

        if (pwm[i] < -1000)
        {
            pwm[i] = -1000;
        } 
        else if (pwm[i] > 1000)
        {
            pwm[i] = 1000;
        }
        
        _real_pwm[i] = pwm[i];
        
        motor_driver((which_motor_e)i, pwm[i]);
    }
}

/******************************************************************************/
void motor_brake_all(uint16_t factor)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (_real_pwm[i] > factor)
        {
            _real_pwm[i] -= factor;
        }
        else if (_real_pwm[i] < -factor)
        {
            _real_pwm[i] += factor;
        }
        else
        {
            _real_pwm[i] = 0;
        }
        
        motor_driver((which_motor_e)i, _real_pwm[i]);
    }
}

#endif
