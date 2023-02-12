#include "ch32f20x.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_tim.h"
#include "bsp_adc.h"
#include "bsp_encoder.h"

#include "led.h"
#include "mpu6500.h"
#include "niming.h"
#include "nrf24l01.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "filter.h"


extern uint8_t g_2ms_flag;
extern uint8_t g_5ms_flag;
extern uint8_t g_20ms_flag;
extern uint8_t g_200ms_flag;

typedef enum
{
    UNLOCK_INIT = 0,
    UNLOCK_SUCCESS
} unlock_state_e;

static unlock_state_e _unlock_state = UNLOCK_INIT;

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main( void )
{
    int ret = 0;
    mpu_result_t mpu_raw_data;      /* mpu原始数据 */
    mpu_result_t mpu_data;          /* 滤波、姿态解算后的数据 */
    float angle_measure, speed_measure, speed_after_filter, gyroz_after_filter;

    uint8_t nrf_buf[RX_PLOAD_WIDTH];
    float batt_volt;                /* 电池电压，单位：V */

    pid_param_t balance_pid;        /* 直立控制环 */
    pid_param_t speed_pid;          /* 速度控制环 */
    pid_param_t turn_pid;           /* 转向控制环 */

    int16_t motor_pwm[MOTOR_NUM] = {0};
    int16_t speed_target = 0;   /* 行走速度，单位：r/min */
    int16_t turn_target  = 0;   /* 转向速度 */

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    systick_init();
    uart_init();
    i2c_init();
    pwm_init();
//    tim_init(); /* 大BUG，TIM1与SPI2同时开启冲突，SPI通信不正常 */
    encoder_l_tim_init();
    encoder_r_tim_init();
    adc_init();
    /* 
     * SYSCLK = 144 MHz
     * HCLK   = 144 MHz
     * PCLK1  = 144 MHz
     * PCLK2  = 72  MHz
     */
    printf("System clock frequency: %d Hz\r\n", SystemCoreClock);

    led_init();

    delay_ms(1000);
    ret = mpu_init(i2c_read, i2c_write);
    printf("mpu6500_dmp_init %d\r\n", ret);
    
    ret = nrf24l01_init();
    printf("nrf24l01_init %d\r\n", ret);
    nrf24l01_rx_mode();
                
    while (1)
    {
        if (1 == g_2ms_flag)
        {
            led_set(LED_LF, TOGGLE);
            g_2ms_flag = 0;
            mpu_read_raw_data(&mpu_raw_data);
            mpu_raw_data_calibration(&mpu_raw_data);
            mpu_raw_data_filter(&mpu_raw_data, &mpu_data);
            imu_update(&mpu_data);
        }
        if (1 == g_5ms_flag)
        {
            led_set(LED_RF, TOGGLE);
            g_5ms_flag = 0;
            get_euler_angle(&mpu_data);
            angle_measure = mpu_data.roll;
            
//            niming_report_imu(&mpu_data);
//            niming_report_data(&mpu_data, speed_after_filter);
            
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
                /* 让电机慢慢减速，谨防骤停导致电源纹波过大 */
                if (balance_pid.out > 300)
                {
                    balance_pid.out -= (balance_pid.out - 300);
                }
                else if (balance_pid.out < -300)
                {
                    balance_pid.out -= (balance_pid.out + 300);
                }
                else if (balance_pid.out > 1)
                {
                    balance_pid.out -= 1;
                }
                else if (balance_pid.out < -1)
                {
                    balance_pid.out += 1;
                }
                else
                {
                    balance_pid.out = 0;
                }
                motor_pwm[MOTOR_L] = -balance_pid.out;
                motor_pwm[MOTOR_R] = -balance_pid.out;
                motor_driver_all(motor_pwm);
            }
        }
        if (1 == g_20ms_flag)
        {
            led_set(LED_LB, TOGGLE);
            g_20ms_flag = 0;

            speed_measure = (encoder_l_speed_get() + encoder_r_speed_get()) / 2;
            speed_after_filter = aver_speed_filter(speed_measure);

            speed_pid.kp = 0.1;
            speed_pid.ki = 0.002;
            speed_pid.limit_integral = 5000;
            speed_control(&speed_pid, speed_target, speed_after_filter);
        
            gyroz_after_filter = aver_gyroz_filter(mpu_data.gyro_zout);
            
            turn_pid.kp = 0.05;
            turn_pid.ki = 0.005;
            turn_pid.limit_integral = 20000;
            turn_control(&turn_pid, turn_target, gyroz_after_filter);
        }
        if (1 == g_200ms_flag)
        {
            g_200ms_flag = 0;
            ret = nrf24l01_rx_packet(nrf_buf);  /* 100us */
            if (ret == 0)
            {
                speed_target  = *(int16_t *)&nrf_buf[0];
                turn_target = *(int16_t *)&nrf_buf[2];
//                printf("%d %d\r\n", speed_target, turn_target);
                
                if (_unlock_state != UNLOCK_SUCCESS)
                {
                    switch (_unlock_state)
                    {
                        case UNLOCK_INIT:
                            if (speed_target > -20 && speed_target < 20)
                            {
                                _unlock_state = UNLOCK_SUCCESS;
                            }
                        break;
                        default: break;
                    }
                    speed_target = 0;
                    turn_target = 0;
                }
                else
                {
                    if (speed_target > -20 && speed_target < 20)
                    {
                        speed_target = 0;
                    }
                    if (turn_target > -100 && turn_target < 100)
                    {
                        turn_target = 0;
                    }
                }
            }

            float batt_last = get_batt_volt();
            batt_volt = aver_batt_filter(batt_last);
            if (batt_volt < 3.0)
            {
                led_set(LED_RB, TOGGLE);
            }
            printf("speed_l=%d speed_r=%d speed=%.2f angle=%.2f batt=%.2f\r\n", encoder_l_speed_get(), encoder_r_speed_get(), speed_after_filter, angle_measure, batt_volt);
        }
    }
}
