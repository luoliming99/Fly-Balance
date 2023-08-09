#include "car.h"
#include "pid.h"
#include "motor.h"
#include "nrf24l01.h"
#include "led.h"
#include "bsp_uart.h"
#include "string.h"     /* memcpy(), strcmp() */

#if (PRODUCT == CAR)

/* 直立控制环 */
static pid_param_t car_pid = 
{
    .kp = 50,
    .kd = 50,
    .pre_error = 0,
};
/* 速度控制环 */
static pid_param_t speed_pid =
{
    .kp = 0.1,
    .ki = 0.003,
    .limit_integral = 4000,
    .pre_error = 0,
};
/* 转向控制环 */
static pid_param_t turn_pid = 
{
    .kp = 0.05,
    .ki = 0.005,
    .limit_integral = 20000,
    .pre_error = 0,
};


/******************************************************************************/
void task_car_pid_control_5ms(float angle_measure)
{
    int16_t motor_pwm[MOTOR_NUM] = {0};
    static uint8_t flag = 0;
    
    if (angle_measure < -45 || angle_measure > 45)
    {
        flag = 0;
    }
    else if (angle_measure > -30 && angle_measure < 30)
    {
        flag = 1;
    }
    
    if (flag)
    {
        balance_control(&car_pid, speed_pid.out, angle_measure);
        
        motor_pwm[MOTOR_L] = -car_pid.out - turn_pid.out;
        motor_pwm[MOTOR_R] = -car_pid.out + turn_pid.out;
        motor_driver_all(motor_pwm);
    }
    else
    {
        speed_pid.integral = 0;
        turn_pid.integral = 0;
        motor_stop_all();
    }
}

/******************************************************************************/
void task_car_pid_control_20ms(int16_t speed_target, int16_t turn_target, float speed_measure, float gyroz)
{
    speed_control(&speed_pid, speed_target, speed_measure);
    turn_control(&turn_pid, turn_target, gyroz);
}

/******************************************************************************/
int task_car_communication(unlock_status_e *unlock_status, int16_t *speed_target, int16_t *turn_target,
                            mpu_result_t *p_data, float batt_volt)
{
    int ret = 0;
    uint8_t nrf_rx_buf[PLOAD_WIDTH_MAX] = {0};
    uint8_t nrf_tx_buf[PLOAD_WIDTH_MAX] = {0};
    
    /* 装载要回传给遥控器的数据 */
    memcpy(nrf_tx_buf, (const char *)"CAS", 4);
    
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
        if (strcmp((const char *)nrf_rx_buf, "CAR") != 0)   /* 未连接到平衡车遥控器 */
        {
            return -1;
        }
        
        switch (*unlock_status)
        {
            case UNLOCK_INIT:
                if (*(int16_t *)&nrf_rx_buf[4] == 0 &&
                    *(int16_t *)&nrf_rx_buf[6] == 0)
                {
                    *unlock_status = UNLOCK_SUCCESS;
                    led_set(LED_RB, ON);
                }
            break;
            case UNLOCK_SUCCESS:
                *speed_target  = *(int16_t *)&nrf_rx_buf[4];
                *turn_target = *(int16_t *)&nrf_rx_buf[6];
            break;
            default: break;
        }
//        printf("%d %d\r\n", *speed_target, *turn_target);
    }
    
    return ret;
}

#endif
