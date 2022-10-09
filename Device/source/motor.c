#include "motor.h"
#include "bsp_pwm.h"

/******************************************************************************/
void motor_driver(which_motor_e motor, int16_t pwm)
{
    if (pwm > 999)
    {
        pwm = 999;
    } 
    else if (pwm < 0)
    {
        pwm = 0;
    }
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
void motor_driver_all(int16_t pwm[4])
{
    motor_driver(MOTOR_LF, pwm[MOTOR_LF]);
    motor_driver(MOTOR_RF, pwm[MOTOR_RF]);
    motor_driver(MOTOR_LB, pwm[MOTOR_LB]);
    motor_driver(MOTOR_RB, pwm[MOTOR_RB]);
}
