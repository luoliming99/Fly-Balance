#include "motor.h"
#include "bsp_pwm.h"

#define __MOTOR_STAT_MIN_VAL    700     /* 电机转动下限值 */

static int16_t _real_pwm[MOTOR_NUM] = {0};

/******************************************************************************/
void motor_driver(which_motor_e motor, int16_t pwm)
{
    if (pwm > 0)    /* 正转 */
    {
        switch (motor)
        {
            case MOTOR_L: PWM_CH1_SET(0); PWM_CH2_SET(pwm); break;
            case MOTOR_R: PWM_CH4_SET(0); PWM_CH3_SET(pwm); break;
            default: break;
        }
    }
    else            /* 反转 */ 
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
    /* 根据电机特性，对PWM占空比做转换 */
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
