#include "pid.h"

/******************************************************************************/
void balance_control(pid_param_t *pid, float target, float measure)
{
    pid->error = target - measure;
    pid->differ = pid->error - pid->pre_error;
    
    
    pid->out = pid->kp * pid->error +
               pid->kd * pid->differ;
    
    pid->pre_error = pid->error;
}

/******************************************************************************/
void speed_control(pid_param_t *pid, float target, float measure)
{
    pid->error = target - measure;
    pid->integral += pid->error;
    
    if (pid->integral > pid->limit_integral)
    {
        pid->integral = pid->limit_integral;
    }
    else if (pid->integral < -pid->limit_integral)
    {
        pid->integral = -pid->limit_integral;
    }
    
    pid->out = pid->kp * pid->error +
               pid->ki * pid->integral;
}

/******************************************************************************/
void turn_control(pid_param_t *pid, float target, float measure)
{
    pid->error = target - measure;
    pid->integral += pid->error;
    
    if (pid->integral > pid->limit_integral)
    {
        pid->integral = pid->limit_integral;
    }
    else if (pid->integral < -pid->limit_integral)
    {
        pid->integral = -pid->limit_integral;
    }
    
    pid->out = pid->kp * pid->error +
               pid->ki * pid->integral;
}
