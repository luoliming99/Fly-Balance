#include "motor.h"
#include "bsp_pwm.h"


#if (PRODUCT == CAR)

#define __MOTOR_STAT_MIN_VAL    680     /* 电机转动下限值，不同电机微调范围±30 */

#endif

/******************************************************************************/
void motor_driver(which_motor_e motor, int16_t pwm)
{
#if (PRODUCT == FLY)
    switch (motor)
    {
        case MOTOR_LF: PWM_CH1_SET(pwm); break;
        case MOTOR_RF: PWM_CH2_SET(pwm); break;
        case MOTOR_LB: PWM_CH3_SET(pwm); break;
        case MOTOR_RB: PWM_CH4_SET(pwm); break;
        default: break;
    }
#elif (PRODUCT == CAR)
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
#endif
}

/******************************************************************************/
void motor_driver_all(int16_t pwm[MOTOR_NUM])
{
#if (PRODUCT == FLY)
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        if (pwm[i] < 0) {
            pwm[i] = 0;
        } else if (pwm[i] > 1000) {
            pwm[i] = 1000;
        }
    
        _real_pwm[i] = pwm[i];

        motor_driver((which_motor_e)i, pwm[i]);
    }
#elif (PRODUCT == CAR)
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
        
        motor_driver((which_motor_e)i, pwm[i]);
    }
#endif
}

/******************************************************************************/
void motor_stop_all(void)
{
    for (uint8_t i = 0; i < MOTOR_NUM; i++)
    {
        motor_driver((which_motor_e)i, 0);
    }
}
