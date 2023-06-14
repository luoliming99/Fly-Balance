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
    
    if (angle_measure > -30 && angle_measure < 30)
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
int task_car_communication(unlock_status_e *unlock_status, int16_t *speed_target, int16_t *turn_target, float batt_volt, float speed_measure, float gyroz)
{
    int ret = 0;
    uint8_t nrf_rx_buf[PLOAD_WIDTH_MAX] = {0};
    uint8_t nrf_tx_buf[PLOAD_WIDTH_MAX] = {0};
    
    /* 装载要回传给遥控器的数据 */
    memcpy(nrf_tx_buf, (const char *)"CAS", 4);
    *(uint16_t *)&nrf_tx_buf[4] = (uint16_t)(batt_volt * 100 + 0.5);    /* 电池电量 */
    *(int16_t *)&nrf_tx_buf[6]  = (int16_t)(speed_measure + 0.5);       /* 速度 */
    *(int16_t *)&nrf_tx_buf[8]  = (int16_t)(gyroz + 0.5);               /* Z轴角速度值 */
    
    ret = nrf24l01_rx_packet_ack_with_payload(nrf_rx_buf, nrf_tx_buf, 10);
    
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
