#include "ch32f20x.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_exti.h"
#include "bsp_adc.h"

#include "led.h"
#include "mpu6500.h"
#include "niming.h"
#include "nrf24l01.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"

uint8_t g_mpu_int = 0;

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
    
    mpu_result_t mpu_result_data;
    uint8_t nrf_buf[RX_PLOAD_WIDTH];

    pid_param_t balance_pid;        /* 直立控制环 */
    pid_param_t speed_pid;          /* 速度控制环 */
    
    int16_t     motor_pwm[4] = {0};
    int16_t     angle_target = 0;
    int16_t     speed_target = 0;   /* 单位：r/min/2 */
    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    systick_init();
    uart_init();
    i2c_init();
    pwm_init();
    exti_init();
    /*
     * SYSCLK = 144 MHz
     * HCLK   = 144 MHz
     * PCLK1  = 144 MHz
     * PCLK2  = 72  MHz
     */
    printf("System clock frequency: %d Hz\r\n", SystemCoreClock);

    led_init();
    encoder_init();
    
    delay_ms(500);
    ret = mpu_dmp_init();
    printf("mpu_dmp_init %d\r\n", ret);
    
    ret = nrf24l01_init();
    printf("nrf24l01_init %d\r\n", ret);
    nrf24l01_rx_mode();
    
    led_set(LED_LF, TOGGLE);
    delay_ms(400);
    led_set(LED_LF, TOGGLE);
    led_set(LED_RF, TOGGLE);
    delay_ms(400);
    led_set(LED_RF, TOGGLE);
    led_set(LED_LB, TOGGLE);
    delay_ms(400);
    led_set(LED_LB, TOGGLE);
    led_set(LED_RB, TOGGLE);
    delay_ms(400);
    led_set(LED_RB, TOGGLE);
    
    motor_pwm[MOTOR_L] = 1000;
    motor_pwm[MOTOR_R] = 1000;
    motor_driver_all(motor_pwm);
                
    while (1);
    {
        led_set(LED_LF, TOGGLE);
        
        if (1 == g_mpu_int) /* 100Hz */
        {
            g_mpu_int = 0;
            
            ret = mpu_dmp_get_data(&mpu_result_data);   /* 12.5us */
            if (ret == 0)   
            {   
                led_set(LED_RF, TOGGLE);
                niming_report_imu(&mpu_result_data);
                
                balance_pid.kp = 50;
                balance_pid.ki = 0;
                balance_pid.kd = 50;
                pid_postion_cal(&balance_pid, angle_target, mpu_result_data.roll);
                
                speed_pid.kp = 0;
                speed_pid.ki = 0.;
                speed_pid.kd = 0;
                speed_pid.limit_integral = 1000;
                pid_postion_cal(&speed_pid, speed_target, encoder_l_speed_get() + encoder_r_speed_get());
                
                motor_pwm[MOTOR_L] = -balance_pid.out + speed_pid.out;
                motor_pwm[MOTOR_R] = -balance_pid.out + speed_pid.out;
                motor_driver_all(motor_pwm);
                
//                printf("%.2f, %d, %d\r\n", mpu_result_data.roll, mpu_result_data.gyro_yout, motor_pwm[MOTOR_L]);
                
            }
        }
        
        ret = nrf24l01_rx_packet(nrf_buf);
        if (ret == 0)
        {   
            led_set(LED_LB, TOGGLE);
            
        }
    }
}
