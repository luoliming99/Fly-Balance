#include "fly.h"
#include "pid.h"
#include "motor.h"
#include "nrf24l01.h"
#include "led.h"
#include "bsp_uart.h"
#include "string.h"     /* memcpy(), strcmp() */

#if (PRODUCT == FLY)

/* 俯仰角速度环 */
static pid_param_t pitch_angle_pid =
{
    .kp = 6,
    .ki = 0,
    .kd = 0,
    .pre_error = 0,
};
/* 横滚角速度环 */
static pid_param_t roll_angle_pid =
{
    .kp = 6,
    .ki = 0,
    .kd = 0,
    .pre_error = 0,
};
/* 偏航角速度环 */
static pid_param_t yaw_angle_pid =
{
    .kp = 10,
    .ki = 0,
    .kd = 0,
    .pre_error = 0,
};

/* 俯仰角度环 */
static pid_param_t pitch_rate_pid =
{
    .kp = 3,
    .ki = 0,
    .kd = 10,
    .pre_error = 0,
};
/* 横滚角度环 */
static pid_param_t roll_rate_pid =
{
    .kp = 3,
    .ki = 0,
    .kd = 10,
    .pre_error = 0,
};
/* 偏航角度环 */
static pid_param_t yaw_rate_pid =
{
    .kp = 1,
    .ki = 0,
    .kd = 0,
    .pre_error = 0,
};

/******************************************************************************/
void task_fly_pid_control_5ms(uint16_t accelerator, int16_t pitch_target, int16_t yaw_target, int16_t roll_target, mpu_result_t *mpu_data)
{
    int16_t motor_pwm[MOTOR_NUM] = {0};
    static uint8_t flag = 0;
    
    if (accelerator < 100)
    {
        flag = 0;
    }
    else if (accelerator > 300)
    {
        flag = 1;
    }
    
    if (flag)
    {
        pid_postion_cal(&pitch_angle_pid, pitch_target, mpu_data->pitch);
        pid_postion_cal(&roll_angle_pid, roll_target, mpu_data->roll);
        pid_postion_cal(&yaw_angle_pid, yaw_target, mpu_data->yaw);

        pid_postion_cal(&pitch_rate_pid, pitch_angle_pid.out, (int16_t)mpu_data->gyro_y >> 4);
        pid_postion_cal(&roll_rate_pid, roll_angle_pid.out, (int16_t)mpu_data->gyro_x >> 4);
        pid_postion_cal(&yaw_rate_pid, yaw_angle_pid.out, (int16_t)mpu_data->gyro_z >> 4);

        motor_pwm[MOTOR_LF] = pitch_rate_pid.out - roll_rate_pid.out - yaw_rate_pid.out + accelerator;
        motor_pwm[MOTOR_RF] = pitch_rate_pid.out + roll_rate_pid.out + yaw_rate_pid.out + accelerator;
        motor_pwm[MOTOR_LB] = -pitch_rate_pid.out - roll_rate_pid.out + yaw_rate_pid.out + accelerator;
        motor_pwm[MOTOR_RB] = -pitch_rate_pid.out + roll_rate_pid.out - yaw_rate_pid.out + accelerator;
        motor_driver_all(motor_pwm);
    }
    else
    {
        motor_stop_all();
    }
}

/******************************************************************************/
int task_fly_communication(unlock_status_e *unlock_status, uint16_t *accelerator, int16_t *pitch_target, int16_t *yaw_target, int16_t *roll_target, 
                        key_status_e *key_val, float batt_volt, mpu_result_t *mpu_data)
{
    int ret = 0;
    uint8_t nrf_rx_buf[PLOAD_WIDTH_MAX] = {0};
    uint8_t nrf_tx_buf[PLOAD_WIDTH_MAX] = {0};
    
    /* 装载要回传给遥控器的数据 */
    memcpy(nrf_tx_buf, (const char *)"FLZ", 4);
    *(uint16_t *)&nrf_tx_buf[4] = (uint16_t)(batt_volt * 100 + 0.5);    /* 电池电量 */
    *(int16_t *)&nrf_tx_buf[6]  = (int16_t)(mpu_data->pitch + 0.5);     /* 俯仰角 */
    *(int16_t *)&nrf_tx_buf[8]  = (int16_t)(mpu_data->roll + 0.5);      /* 横滚角 */
    *(int16_t *)&nrf_tx_buf[10]  = (int16_t)(mpu_data->yaw + 0.5);      /* 偏航角 */
    
    ret = nrf24l01_rx_packet_ack_with_payload(nrf_rx_buf, nrf_tx_buf, 12);
    
    if (ret == 0)
    {   
        if (strcmp((const char *)nrf_rx_buf, "FLY") != 0)   /* 未连接到飞行器遥控器 */
        {
            return -1;
        }

        switch (*unlock_status)
        {
            case UNLOCK_INIT:
                if (*(uint16_t *)&nrf_rx_buf[4] < 50)   /* 油门摇杆在最小位置才能解锁成功 */
                {
                    *unlock_status = UNLOCK_SUCCESS;
                    led_set(LED_RB, ON);
                } 
            break;
            case UNLOCK_SUCCESS:
                *accelerator  = *(uint16_t *)&nrf_rx_buf[4];
                *pitch_target = *(int16_t *)&nrf_rx_buf[6];
                *roll_target  = *(int16_t *)&nrf_rx_buf[8];
                *key_val  = (key_status_e)*(uint8_t *)&nrf_rx_buf[10];
            break;
            default: break;
        }
//        printf("%d %d %d %d %d\r\n", *unlock_status, *accelerator, *pitch_target, *roll_target, *key_val);
    }
    
    return ret;
}

/******************************************************************************/
void task_fly_recv_data_handler(int16_t *yaw_target, key_status_e key_val)
{
    static float rudder_val = 180.0;   /* 打舵值(0° ~ 360°) */

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


#endif





