#include "balance.h"
#include "pid.h"
#include "motor.h"
#include "nrf24l01.h"
#include "led.h"
#include "bsp_uart.h"

#if (PRODUCT == CAR)

static pid_param_t balance_pid;        /* 直立控制环 */
static pid_param_t speed_pid;          /* 速度控制环 */
static pid_param_t turn_pid;           /* 转向控制环 */


/******************************************************************************/
void task_balance_pid_control_5ms(float angle_measure)
{
    int16_t motor_pwm[MOTOR_NUM] = {0};
    
    if (angle_measure > -30 && angle_measure < 30)
    {
        balance_pid.kp = 50;
        balance_pid.kd = 40;
        balance_control(&balance_pid, speed_pid.out, angle_measure);
        
        motor_pwm[MOTOR_L] = -balance_pid.out - turn_pid.out;
        motor_pwm[MOTOR_R] = -balance_pid.out + turn_pid.out;
        motor_driver_all(motor_pwm);
    }
    else
    {
        speed_pid.integral = 0;
        turn_pid.integral = 0;
        motor_brake_all(1);     /* 让电机慢慢减速，防止骤停导致电源纹波过大 */
    }
}

/******************************************************************************/
void task_balance_pid_control_20ms(int16_t speed_target, int16_t turn_target, float speed_measure, float gyroz)
{
    speed_pid.kp = 0.1;
    speed_pid.ki = 0.003;
    speed_pid.limit_integral = 4000;
    speed_control(&speed_pid, speed_target, speed_measure);

    turn_pid.kp = 0.05;
    turn_pid.ki = 0.005;
    turn_pid.limit_integral = 20000;
    turn_control(&turn_pid, turn_target, gyroz);
}

/******************************************************************************/
int task_balance_communication(int16_t *speed_target, int16_t *turn_target, float batt_volt, float speed_measure)
{
    int ret = 0;
    uint8_t nrf_rx_buf[PLOAD_WIDTH_MAX] = {0};
    uint8_t nrf_tx_buf[PLOAD_WIDTH_MAX] = {0};
    
    /* 装载要回传给遥控器的数据 */
    *(uint16_t *)&nrf_tx_buf[0] = (uint16_t)(batt_volt * 100 + 0.5);    /* 电池电量 */
    *(int16_t *)&nrf_tx_buf[2]  = (int16_t)(speed_measure + 0.5);       /* 速度 */
    
    ret = nrf24l01_rx_packet_ack_with_payload(nrf_rx_buf, nrf_tx_buf, 4);
    
    if (ret == 0)
    {   
        *speed_target  = *(int16_t *)&nrf_rx_buf[0];
        *turn_target = *(int16_t *)&nrf_rx_buf[2];
//        printf("%d %d\r\n", *speed_target, *turn_target);
    }
    
    return ret;
}

/******************************************************************************/
void task_balance_recv_data_handler(unlock_status_e *unlock_status, int16_t *speed_target, int16_t *turn_target)
{
    if (UNLOCK_SUCCESS != *unlock_status)
    {
        switch (*unlock_status)
        {
            case UNLOCK_INIT:
                if (*speed_target > -20 && *speed_target < 20 &&
                    *turn_target > -100 && *turn_target < 100)
                {
                    *unlock_status = UNLOCK_SUCCESS;
                    led_set(LED_RB, ON);
                }
            break;
            default: break;
        }
        *speed_target = 0;
        *turn_target = 0;
    }
    else
    {
        if (*speed_target > -20 && *speed_target < 20)
        {
            *speed_target = 0;
        }
        if (*turn_target > -100 && *turn_target < 100)
        {
            *turn_target = 0;
        }
    }
}
    
#endif
