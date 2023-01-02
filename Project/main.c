#include "ch32f20x.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_exti.h"
#include "bsp_adc.h"
#include "bsp_tim.h"

#include "led.h"
#include "mpu6500.h"
#include "niming.h"
#include "nrf24l01.h"
#include "motor.h"
#include "pid.h"

uint8_t g_mpu_int = 0;
uint8_t g_2ms_flag = 0;

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

    pid_param_t pitch_rate_pid;     /* 俯仰角速度环 */
    pid_param_t yaw_rate_pid;       /* 偏航角速度环 */
    pid_param_t roll_rate_pid;      /* 横滚角速度环 */
    
    pid_param_t pitch_angle_pid;    /* 俯仰角度环 */
    pid_param_t yaw_angle_pid;      /* 偏航角度环 */
    pid_param_t roll_angle_pid;     /* 横滚角度环 */
    
    int16_t     motor_pwm[4] = {0};
    
    uint16_t accelerator  = 0;
    int16_t  pitch_target = 0;
    int16_t  roll_target  = 0;

    
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    systick_init();
    uart_init();
    i2c_init();
    pwm_init();
    exti_init();
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
    
    delay_ms(500);
    ret = mpu_dmp_init();
//    ret = mpu6500_init(i2c_read, i2c_write);
    printf("mpu6500_dmp_init %d\r\n", ret);
    
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
    
//    motor_pwm[MOTOR_LF] = 300;
//    motor_pwm[MOTOR_RF] = 300;
//    motor_pwm[MOTOR_LB] = 300;
//    motor_pwm[MOTOR_RB] = 300;
//    motor_driver_all(motor_pwm);
                
    while (1)
    {
//        printf("%.2f\r\n", (float)g_adc_val[0] / 4095 * 3.3 * 1.36);
        
        led_set(LED_LF, TOGGLE);
        if (1 == g_mpu_int) /* 200Hz */
        {
            g_mpu_int = 0;
            ret = mpu_dmp_get_data(&mpu_result_data);   /* 12.5us */
//        if (1 == g_2ms_flag)
//        {
//            g_2ms_flag = 0;
//            ret = MpuGetData(&mpu_result_data);
//            GetAngle(&mpu_result_data, 0.002);
            if (ret == 0)   /* 200Hz */
            {   
                led_set(LED_RF, TOGGLE);
                niming_report_imu(&mpu_result_data);
                niming_report_data(&mpu_result_data);

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
                    pid_postion_cal(&pitch_angle_pid, pitch_target, mpu_result_data.pitch);
//                    roll_angle_pid.kp = 8;
//                    roll_angle_pid.ki = 0;
//                    roll_angle_pid.kd = 0;
//                    pid_postion_cal(&roll_angle_pid, roll_target, mpu_result_data.roll);
//                    yaw_angle_pid.kp = 5;
//                    yaw_angle_pid.ki = 0;
//                    yaw_angle_pid.kd = 0;
//                    pid_postion_cal(&yaw_angle_pid, 0, mpu_result_data.yaw);

                    pitch_rate_pid.kp = 3;
                    pitch_rate_pid.ki = 0;
                    pitch_rate_pid.kd = 10;
                    pid_postion_cal(&pitch_rate_pid, pitch_angle_pid.out, mpu_result_data.gyro_yout >> 4);
//                    roll_rate_pid.kp = 2.5;
//                    roll_rate_pid.ki = 0;
//                    roll_rate_pid.kd = 4;
//                    pid_postion_cal(&roll_rate_pid, roll_angle_pid.out, mpu_result_data.gyro_xout >> 4);
//                    yaw_rate_pid.kp = 1;
//                    yaw_rate_pid.ki = 0;
//                    yaw_rate_pid.kd = 0;
//                    pid_postion_cal(&yaw_rate_pid, yaw_angle_pid.out, mpu_result_data.gyro_zout >> 4);

//                    motor_pwm[MOTOR_LF] =  pitch_rate_pid.out - roll_rate_pid.out - yaw_rate_pid.out + accelerator;
//                    motor_pwm[MOTOR_RF] =  pitch_rate_pid.out + roll_rate_pid.out + yaw_rate_pid.out + accelerator;
//                    motor_pwm[MOTOR_LB] = -pitch_rate_pid.out - roll_rate_pid.out + yaw_rate_pid.out + accelerator;
//                    motor_pwm[MOTOR_RB] = -pitch_rate_pid.out + roll_rate_pid.out - yaw_rate_pid.out + accelerator;
                    motor_pwm[MOTOR_LF] = 300;
                    motor_pwm[MOTOR_RF] = 300;
//                    motor_pwm[MOTOR_LB] = 300;
//                    motor_pwm[MOTOR_RB] = 300;
                    motor_driver_all(motor_pwm);
                }
            }
        }
        
        ret = nrf24l01_rx_packet(nrf_buf);
        if (ret == 0)
        {   
            led_set(LED_LB, TOGGLE);
            accelerator  = *(uint16_t *)&nrf_buf[0];
            pitch_target = *(int16_t *)&nrf_buf[2]; 
            roll_target  = *(int16_t *)&nrf_buf[4];
            
//            printf("%d %d %d\r\n", accelerator, pitch_target, roll_target);
            
            if (accelerator < 300)
            {
                accelerator = 0;
            }
            else
                accelerator = 300;
        }
    }
}
