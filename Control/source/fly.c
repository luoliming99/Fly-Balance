#include "fly.h"
#include "pid.h"
#include "motor.h"
#include "nrf24l01.h"
#include "led.h"
#include "bsp_uart.h"

#if (PRODUCT == FLY)

static pid_param_t pitch_rate_pid;     /* 俯仰角速度环 */
static pid_param_t yaw_rate_pid;       /* 偏航角速度环 */
static pid_param_t roll_rate_pid;      /* 横滚角速度环 */

static pid_param_t pitch_angle_pid;    /* 俯仰角度环 */
static pid_param_t yaw_angle_pid;      /* 偏航角度环 */
static pid_param_t roll_angle_pid;     /* 横滚角度环 */

/******************************************************************************/
void task_fly_pid_control(uint16_t accelerator, int16_t pitch_target, int16_t yaw_target, int16_t roll_target, mpu_result_t *mpu_data)
{
    int16_t motor_pwm[MOTOR_NUM] = {0};
    
    pitch_angle_pid.kp = 6;
    pitch_angle_pid.ki = 0;
    pitch_angle_pid.kd = 0;
    pid_postion_cal(&pitch_angle_pid, pitch_target, mpu_data->pitch);
    roll_angle_pid.kp = 6;
    roll_angle_pid.ki = 0;
    roll_angle_pid.kd = 0;
    pid_postion_cal(&roll_angle_pid, roll_target, mpu_data->roll);
    yaw_angle_pid.kp = 10;
    yaw_angle_pid.ki = 0;
    yaw_angle_pid.kd = 0;
    pid_postion_cal(&yaw_angle_pid, yaw_target, mpu_data->yaw);

    pitch_rate_pid.kp = 3;
    pitch_rate_pid.ki = 0;
    pitch_rate_pid.kd = 10;
    pid_postion_cal(&pitch_rate_pid, pitch_angle_pid.out, mpu_data->gyro_yout >> 4);
    roll_rate_pid.kp = 3;
    roll_rate_pid.ki = 0;
    roll_rate_pid.kd = 10;
    pid_postion_cal(&roll_rate_pid, roll_angle_pid.out, mpu_data->gyro_xout >> 4);
    yaw_rate_pid.kp = 1;
    yaw_rate_pid.ki = 0;
    yaw_rate_pid.kd = 0;
    pid_postion_cal(&yaw_rate_pid, yaw_angle_pid.out, mpu_data->gyro_zout >> 4);

    motor_pwm[MOTOR_LF] = pitch_rate_pid.out - roll_rate_pid.out - yaw_rate_pid.out + accelerator;
    motor_pwm[MOTOR_RF] = pitch_rate_pid.out + roll_rate_pid.out + yaw_rate_pid.out + accelerator;
    motor_pwm[MOTOR_LB] = -pitch_rate_pid.out - roll_rate_pid.out + yaw_rate_pid.out + accelerator;
    motor_pwm[MOTOR_RB] = -pitch_rate_pid.out + roll_rate_pid.out - yaw_rate_pid.out + accelerator;
    motor_driver_all(motor_pwm);
}

/******************************************************************************/
int task_fly_communication(uint16_t *accelerator, int16_t *pitch_target, int16_t *yaw_target, int16_t *roll_target, key_status_e *key_val,
                            float batt_volt, mpu_result_t *mpu_data)
{
    int ret = 0;
    uint8_t nrf_rx_buf[PLOAD_WIDTH_MAX] = {0};
    uint8_t nrf_tx_buf[PLOAD_WIDTH_MAX] = {0};
    
    /* 装载要回传给遥控器的数据 */
    *(uint16_t *)&nrf_tx_buf[0] = (uint16_t)(batt_volt * 100 + 0.5);    /* 电池电量 */
    *(int16_t *)&nrf_tx_buf[2]  = (int16_t)(mpu_data->pitch + 0.5);     /* 俯仰角 */
    *(int16_t *)&nrf_tx_buf[4]  = (int16_t)(mpu_data->roll + 0.5);      /* 横滚角 */
    *(int16_t *)&nrf_tx_buf[6]  = (int16_t)(mpu_data->yaw + 0.5);       /* 偏航角 */
    
    ret = nrf24l01_rx_packet_ack_with_payload(nrf_rx_buf, nrf_tx_buf, 8);
    
    if (ret == 0)
    {   
        *accelerator  = *(uint16_t *)&nrf_rx_buf[0];
        *pitch_target = *(int16_t *)&nrf_rx_buf[2];
        *roll_target  = *(int16_t *)&nrf_rx_buf[4];
        *key_val  = (key_status_e)*(uint8_t *)&nrf_rx_buf[6];
//        printf("%d %d %d %d\r\n", *accelerator, *pitch_target, *roll_target, *key_val);
    }
    
    return ret;
}

/******************************************************************************/
void task_fly_recv_data_handler(unlock_status_e *unlock_status, uint16_t *accelerator, int16_t *yaw_target, key_status_e key_val)
{
    static float rudder_val = 180.0;   /* 打舵值(0° ~ 360°) */
    
    if (*unlock_status != UNLOCK_SUCCESS)
    {
        switch (*unlock_status)
        {
            case UNLOCK_INIT:
                if (*accelerator > 850)
                {
                    *unlock_status = UNLOCK_MAX_ACC;
                }
            break;
            case UNLOCK_MAX_ACC:
                if (*accelerator < 50)
                {
                    *unlock_status = UNLOCK_SUCCESS;
                    led_set(LED_RB, ON);
                }
            break;
            default: break;
        }
        *accelerator = 0;
    }
    else
    {
        if (*accelerator < 300)
        {
            *accelerator = 0;
        }
        switch (key_val)
        {
            case KEY_L_PRESS: 
                rudder_val -= 0.4;
                if (rudder_val <= 0.0)
                {
                    rudder_val = 360;
                }
            break;
            case KEY_R_PRESS:
                rudder_val += 0.4;
                if (rudder_val >= 360.0)
                {
                    rudder_val = 0;
                }
            break;
            default: break;
        }
        *yaw_target = 180 - rudder_val;
    } 
}


#endif





