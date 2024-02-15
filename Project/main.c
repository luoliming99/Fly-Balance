#include "ch32f20x.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "bsp_i2c.h"
#include "bsp_pwm.h"
#include "bsp_adc.h"
#include "bsp_tim.h"
#include "bsp_encoder.h"
#include "bsp_iwdg.h"

#include "led.h"
#include "mpu6500.h"
#include "niming.h"
#include "nrf24l01.h"
#include "motor.h"
#include "pid.h"
#include "encoder.h"
#include "filter.h"

#include "common.h"
#include "fly.h"
#include "car.h"
#include "string.h"     /* memset() */


extern uint8_t g_5ms_flag;
extern uint8_t g_20ms_flag;
extern uint8_t g_200ms_flag;

mpu_result_t g_mpu_data;        /* 姿态数据 */
uint8_t      g_sys_init_ok = 0; /* 系统初始化完成标志 */

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
    float batt_volt = 0;            /* 电池电压，单位：V */
    
    unlock_status_e unlock_status = UNLOCK_INIT;
    motor_status_e  motor_status  = MOTOR_STOP;
    
#if (PRODUCT == FLY)
    uint16_t accelerator  = 0;  /* 30 ~ 900 */
    int16_t  pitch_target = 0;  /* -30°~ 30° */
    int16_t  yaw_target   = 0;  /* -180°~ 180° */
    int16_t  roll_target  = 0;  /* -30°~ 30° */
    
    key_status_e    key_val         = NO_KEY_PRESS;
#elif (PRODUCT == CAR)
    float speed_measure, speed_after_filter = 0, gyroz_after_filter = 0;
    int16_t speed_target = 0;   /* 行走速度，单位：r/min */
    int16_t turn_target  = 0;   /* 转向速度 */
#endif

    memset(&g_mpu_data, 0, sizeof(mpu_result_t));


    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    systick_init();
    uart_init();
    i2c_init();
    pwm_init();
    adc_init();

#if (PRODUCT == CAR)
    encoder_l_tim_init();
    encoder_r_tim_init();
#endif
    
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
    
    iwdg_feed_init(IWDG_Prescaler_32, 4000);    /* 看门狗复位时间3.2s */
    
    g_sys_init_ok = 1;  /* 标志系统初始化完成 */
        
    while (1)
    {
        if (1 == g_5ms_flag)
        {
            led_set(LED_RF, TOGGLE);
            g_5ms_flag = 0;
            
            get_euler_angle(&g_mpu_data);       /* 60us */
            
//            niming_report_imu(&g_mpu_data);     /* 600us */
//            niming_report_data(&g_mpu_data);    /* 700us */
            
#if (PRODUCT == FLY)
            if (UNLOCK_SUCCESS == unlock_status)
            {
                task_fly_pid_control_5ms(accelerator, pitch_target, yaw_target, roll_target, &g_mpu_data, &motor_status);
                comm_wdg_enable();
            }
            
            ret = task_fly_communication(&unlock_status, &accelerator, &pitch_target, &yaw_target,
                                            &roll_target, &key_val, &g_mpu_data, batt_volt);    /* 120us */

            if (ret == 0)
            {   
                led_set(LED_LB, TOGGLE);
                task_fly_recv_data_handler(&yaw_target, key_val);
            }
            else
            {
                led_set(LED_LB, OFF);
            }
#elif (PRODUCT == CAR)
            task_car_pid_control_5ms(g_mpu_data.roll, &motor_status);
            
            ret = task_car_communication(&unlock_status, &speed_target, &turn_target, 
                                            &g_mpu_data, batt_volt);                            /* 120us */                           

            if (ret == 0)
            {   
                led_set(LED_LB, TOGGLE);
            }
            else
            {
                led_set(LED_LB, OFF);
            }
#endif
            
            if (ret == 0 || motor_status == MOTOR_STOP)
            {
                comm_wdg_feed();
            }
            if (is_comm_wdg_interrupted())
            {
#if (PRODUCT == FLY)
                accelerator = 0;
#elif (PRODUCT == CAR)
                speed_target = 0;
                turn_target = 0;
#endif
                comm_wdg_disable();
            }
        }
        if (1 == g_20ms_flag)
        {
            g_20ms_flag = 0;
#if (PRODUCT == CAR)           
            speed_measure = (encoder_l_speed_get() + encoder_r_speed_get()) / 2;
            speed_after_filter = speed_lpf_filter(speed_measure, speed_after_filter);  
            gyroz_after_filter = gyroz_lpf_filter(g_mpu_data.gyro_z, gyroz_after_filter);
            
            task_car_pid_control_20ms(speed_target, turn_target, speed_after_filter, gyroz_after_filter);
#endif
        }
        if (1 == g_200ms_flag)
        {
            g_200ms_flag = 0;
            float batt_last = (float)g_adc_val[0] / 4095 * 3.3 * 1.36;
            batt_volt = batt_aver_filter(batt_last);
        }
        IWDG_ReloadCounter();
    }
}
