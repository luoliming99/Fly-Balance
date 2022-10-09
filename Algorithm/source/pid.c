#include "pid.h"

/******************************************************************************/
void pid_postion_cal(pid_param_t *pid, float target, float measure)
{
    pid->error = target - measure;
    pid->integral += pid->error;
    pid->differ = pid->error - pid->pre_error;
    
    if (pid->integral > pid->limit_integral)
    {
        pid->integral = pid->limit_integral;
    }
    else if (pid->integral < -pid->limit_integral)
    {
        pid->integral = -pid->limit_integral;
    }
    
    pid->out = pid->kp * pid->error +
               pid->ki * pid->integral +
               pid->kd * pid->differ;
    
    pid->pre_error = pid->error;
}



/******************************************************************************/
void pid_incremental_cal(pid_param_t *pid, float target, float measure)
{
    pid->error = target - measure;
    
    /*
     * 位置PID与增量PID的转换公式推导：
     * out = kp * error + ki * (sum + error) + kd * (error - pre_error)
     * pre_out = kp * pre_error + ki * sum + kd * (pre_error - ppre_error)
     * incremental = out - pre_out
     *             = kp * (error - pre_error) + 
     *               ki * error + 
     *               kd * (error - 2 * pre_error + ppre_error)
     */
    pid->out = pid->kp * (pid->error - pid->pre_error) +
               pid->ki * pid->error +
               pid->kd * (pid->error - 2 * pid->pre_error + pid->ppre_error);
    
    pid->ppre_error = pid->pre_error;
    pid->pre_error = pid->error;
}
