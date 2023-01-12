#include "ch32f20x.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_tim.h"

#include "led.h"
#include "mpu6500.h"
#include "niming.h"
#include "nrf24l01.h"
#include "motor.h"
#include "pid.h"
#include "filter.h"


extern uint8_t g_2ms_flag;
extern uint8_t g_5ms_flag;
extern uint8_t g_10ms_flag;
extern uint8_t g_200ms_flag;

typedef enum
{
    UNLOCK_INIT = 0,
    UNLOCK_MAX_ACC,
    UNLOCK_SUCCESS
} unlock_state_e;

typedef enum
{
    NO_KEY_PRESS = 0,
    KEY_L_PRESS,
    KEY_R_PRESS,
} key_status_e;

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
    uint8_t nrf_buf[RX_PLOAD_WIDTH];
    float batt_volt;                /* 电池电压，单位：V */

    pid_param_t pitch_rate_pid;     /* 俯仰角速度环 */
    pid_param_t yaw_rate_pid;       /* 偏航角速度环 */
    pid_param_t roll_rate_pid;      /* 横滚角速度环 */
    
    pid_param_t pitch_angle_pid;    /* 俯仰角度环 */
    pid_param_t yaw_angle_pid;      /* 偏航角度环 */
    pid_param_t roll_angle_pid;     /* 横滚角度环 */
    
    int16_t     motor_pwm[4] = {0};
    
    uint16_t accelerator  = 0;  /* 30 ~ 900 */
    int16_t  pitch_target = 0;  /* -30° ~ 30° */
    int16_t  yaw_target   = 0;  /* -180° ~ 180° */
    int16_t  roll_target  = 0;  /* -30° ~ 30° */
    
    key_status_e  key_val = NO_KEY_PRESS;
    float rudder_val = 180.0;   /* 打舵值(0° ~ 360°) */

    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    systick_init();
    uart_init();
    i2c_init();
    pwm_init();
    adc_init();
    tim_init();
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
    
    motor_pwm[MOTOR_LF] = 1000;
    motor_pwm[MOTOR_RF] = 0;
    motor_pwm[MOTOR_LB] = 1000;
    motor_pwm[MOTOR_RB] = 0;
    motor_driver_all(motor_pwm);
                
    while (1)
    {
        if (1 == g_2ms_flag && UNLOCK_SUCCESS == _unlock_state)    /* 400us */
        {
            led_set(LED_LF, TOGGLE);
            g_2ms_flag = 0;
            mpu_read_raw_data(&mpu_raw_data);
            mpu_raw_data_calibration(&mpu_raw_data);
            mpu_raw_data_filter(&mpu_raw_data, &mpu_data);
//            mpu_data = mpu_raw_data;
            imu_update(&mpu_data);
        }
        if (1 == g_5ms_flag && UNLOCK_SUCCESS == _unlock_state)    /* 50us */
        {
            led_set(LED_RF, TOGGLE);
            g_5ms_flag = 0;
            get_euler_angle(&mpu_data);
                
//            niming_report_imu(&mpu_data);
//            niming_report_data(&mpu_data);

            if (accelerator == 0)
            {
                motor_pwm[MOTOR_LF] = 0;
                motor_pwm[MOTOR_RF] = 0;
                motor_pwm[MOTOR_LB] = 0;
                motor_pwm[MOTOR_RB] = 0;
                motor_driver_all(motor_pwm);
            }
            else
            {
                pitch_angle_pid.kp = 6;
                pitch_angle_pid.ki = 0;
                pitch_angle_pid.kd = 0;
                pid_postion_cal(&pitch_angle_pid, pitch_target, mpu_data.pitch);
                roll_angle_pid.kp = 6;
                roll_angle_pid.ki = 0;
                roll_angle_pid.kd = 0;
                pid_postion_cal(&roll_angle_pid, roll_target, mpu_data.roll);
                yaw_angle_pid.kp = 10;
                yaw_angle_pid.ki = 0;
                yaw_angle_pid.kd = 0;
                pid_postion_cal(&yaw_angle_pid, yaw_target, mpu_data.yaw);

                pitch_rate_pid.kp = 3;
                pitch_rate_pid.ki = 0;
                pitch_rate_pid.kd = 10;
                pid_postion_cal(&pitch_rate_pid, pitch_angle_pid.out, mpu_data.gyro_yout >> 4);
                roll_rate_pid.kp = 3;
                roll_rate_pid.ki = 0;
                roll_rate_pid.kd = 10;
                pid_postion_cal(&roll_rate_pid, roll_angle_pid.out, mpu_data.gyro_xout >> 4);
                yaw_rate_pid.kp = 1;
                yaw_rate_pid.ki = 0;
                yaw_rate_pid.kd = 0;
                pid_postion_cal(&yaw_rate_pid, yaw_angle_pid.out, mpu_data.gyro_zout >> 4);

                motor_pwm[MOTOR_LF] =  pitch_rate_pid.out - roll_rate_pid.out - yaw_rate_pid.out + accelerator;
                motor_pwm[MOTOR_RF] =  pitch_rate_pid.out + roll_rate_pid.out + yaw_rate_pid.out + accelerator;
                motor_pwm[MOTOR_LB] = -pitch_rate_pid.out - roll_rate_pid.out + yaw_rate_pid.out + accelerator;
                motor_pwm[MOTOR_RB] = -pitch_rate_pid.out + roll_rate_pid.out - yaw_rate_pid.out + accelerator;
                motor_driver_all(motor_pwm);
            }
        }
        if (1 == g_10ms_flag)
        {
            g_10ms_flag = 0;
            ret = nrf24l01_rx_packet(nrf_buf);  /* 100us */
            if (ret == 0)
            {   
                accelerator  = *(uint16_t *)&nrf_buf[0];
                pitch_target = *(int16_t *)&nrf_buf[2];
                roll_target  = *(int16_t *)&nrf_buf[4];
                key_val  = (key_status_e)*(uint8_t *)&nrf_buf[6];
//                printf("%d %d %d %d\r\n", accelerator, pitch_target, roll_target, key_val);
                
                if (_unlock_state != UNLOCK_SUCCESS)
                {
                    switch (_unlock_state)
                    {
                        case UNLOCK_INIT:
                            if (accelerator > 850)
                            {
                                _unlock_state = UNLOCK_MAX_ACC;
                                led_set(LED_LF, ON);
                                led_set(LED_RF, ON);
                            }
                        break;
                        case UNLOCK_MAX_ACC:
                            if (accelerator < 50)
                            {
                                _unlock_state = UNLOCK_SUCCESS;
                                led_set(LED_LB, ON);
//                                led_set(LED_RB, ON);
                            }
                        break;
                        default: break;
                    }
                    accelerator = 0;
                }
                else
                {
                    if (accelerator < 300)
                    {
                        accelerator = 0;
                    }
                    switch (key_val)
                    {
                        case KEY_L_PRESS: 
                            rudder_val -= 0.2;
                            if (rudder_val <= 0.0)
                            {
                                rudder_val = 360;
                            }
                        break;
                        case KEY_R_PRESS:
                            rudder_val += 0.2;
                            if (rudder_val >= 360.0)
                            {
                                rudder_val = 0;
                            }
                        break;
                        default: break;
                    }
                    yaw_target = 180 - rudder_val;
//                    printf("%d\r\n", yaw_target);
                }
            }
        }
        if (1 == g_200ms_flag)
        {
            g_200ms_flag = 0;
            float batt_last = (float)g_adc_val[0] / 4095 * 3.3 * 1.36;
            batt_volt = batt_aver_filter(batt_last);
//            printf("%.2f\r\n", batt_volt);
            if (batt_volt < 2.6)
            {
                led_set(LED_LB, TOGGLE);
            }
        }
    }
}
