#include "fly.h"
#include "pid.h"
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
void task_fly_pid_control_5ms(uint16_t accelerator, int16_t pitch_target, int16_t yaw_target, int16_t roll_target, mpu_result_t *mpu_data, motor_status_e *motor_status)
{
    int16_t motor_pwm[MOTOR_NUM] = {0};
    
    if (accelerator < 100)
    {
        *motor_status = MOTOR_STOP;
    }
    else if (accelerator > 300)
    {
        *motor_status = MOTOR_RUN;
    }
    
    if (*motor_status == MOTOR_RUN)
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
int task_fly_communication(unlock_status_e *unlock_status, uint16_t *accelerator, int16_t *pitch_target, int16_t *yaw_target,
                            int16_t *roll_target, key_status_e *key_val, mpu_result_t *p_data, float batt_volt)
{
    int ret = 0;
    uint8_t nrf_rx_buf[PLOAD_WIDTH_MAX] = {0};
    uint8_t nrf_tx_buf[PLOAD_WIDTH_MAX] = {0};
    
    /* 装载要回传给遥控器的数据 */
    memcpy(nrf_tx_buf, (const char *)"FLZ", 4);
    
    nrf_tx_buf[4] = (int16_t)p_data->accel_x >> 8;
    nrf_tx_buf[5] = (int16_t)p_data->accel_x;
        
    nrf_tx_buf[6] = (int16_t)p_data->accel_y >> 8;
    nrf_tx_buf[7] = (int16_t)p_data->accel_y;
    
    nrf_tx_buf[8] = (int16_t)p_data->accel_z >> 8;
    nrf_tx_buf[9] = (int16_t)p_data->accel_z;
    
    nrf_tx_buf[10] = (int16_t)p_data->gyro_x >> 8;
    nrf_tx_buf[11] = (int16_t)p_data->gyro_x;

    nrf_tx_buf[12] = (int16_t)p_data->gyro_y >> 8;
    nrf_tx_buf[13] = (int16_t)p_data->gyro_y;

    nrf_tx_buf[14] = (int16_t)p_data->gyro_z >> 8;
    nrf_tx_buf[15] = (int16_t)p_data->gyro_z;
    
    nrf_tx_buf[16] = (int16_t)(p_data->roll * 100) >> 8;
    nrf_tx_buf[17] = (int16_t)(p_data->roll * 100);

    nrf_tx_buf[18] = (int16_t)(p_data->pitch * 100) >> 8;
    nrf_tx_buf[19] = (int16_t)(p_data->pitch * 100);

    nrf_tx_buf[20] = (int16_t)(p_data->yaw * 10) >> 8;
    nrf_tx_buf[21] = (int16_t)(p_data->yaw * 10);
    
    /* 加速度计寄存器原始值也发过去用上位机显示，和滤波后的值对比 */
    nrf_tx_buf[22] = (int16_t)p_data->accel_xreg >> 8;
    nrf_tx_buf[23] = (int16_t)p_data->accel_xreg;
        
    nrf_tx_buf[24] = (int16_t)p_data->accel_yreg >> 8;
    nrf_tx_buf[25] = (int16_t)p_data->accel_yreg;
    
    nrf_tx_buf[26] = (int16_t)p_data->accel_zreg >> 8;
    nrf_tx_buf[27] = (int16_t)p_data->accel_zreg;
    
    *(uint16_t *)&nrf_tx_buf[28] = (uint16_t)(batt_volt * 100 + 0.5);   /* 电池电量 */
    
    ret = nrf24l01_rx_packet_ack_with_payload(nrf_rx_buf, nrf_tx_buf, 30);
    
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





