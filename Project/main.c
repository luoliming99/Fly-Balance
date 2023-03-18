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

#include "common.h"
#include "fly.h"


extern uint8_t g_2ms_flag;
extern uint8_t g_5ms_flag;
extern uint8_t g_20ms_flag;
extern uint8_t g_200ms_flag;


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
    mpu_result_t mpu_data;          /* MPU滤波、姿态解算后的数据 */
    
    float batt_volt = 0;            /* 电池电压，单位：V */
    
    uint16_t accelerator  = 0;  /* 30 ~ 900 */
    int16_t  pitch_target = 0;  /* -30°~ 30° */
    int16_t  yaw_target   = 0;  /* -180°~ 180° */
    int16_t  roll_target  = 0;  /* -30°~ 30° */
    
    key_status_e    key_val         = NO_KEY_PRESS;
    unlock_status_e unlock_status   = UNLOCK_INIT;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    systick_init();
    uart_init();
    i2c_init();
    pwm_init();
    adc_init();
//    tim_init(); /* TIM1与SPI2同时开启冲突，SPI通信不正常 */
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
        if (1 == g_2ms_flag)    /* 400us */
        {
            led_set(LED_LF, TOGGLE);
            g_2ms_flag = 0;
#ifdef FLY
            if (UNLOCK_SUCCESS == unlock_status)
            {
                task_imu_update(&mpu_data);
            }
#else
            task_imu_update(&mpu_data);
#endif    
        }
        if (1 == g_5ms_flag && UNLOCK_SUCCESS == unlock_status)    /* 50us */
        {
            led_set(LED_RF, TOGGLE);
            g_5ms_flag = 0;
            
            get_euler_angle(&mpu_data);
                
//            niming_report_imu(&mpu_data);
//            niming_report_data(&mpu_data);
#ifdef FLY
            if (accelerator == 0)
            {
                motor_stop_all();
            }
            else
            {
                task_fly_pid_control(accelerator, pitch_target, yaw_target, roll_target, &mpu_data);
            }
#else
            
#endif
        }
        if (1 == g_20ms_flag)
        {
            g_20ms_flag = 0;
#ifdef FLY            
            ret = task_fly_communication(&accelerator, &pitch_target, &yaw_target, &roll_target, &key_val,
                                            batt_volt, &mpu_data);
            if (ret == 0)
            {   
                task_fly_recv_data_handler(&unlock_status, &accelerator, &yaw_target, key_val);
            }
#else
            
#endif
        }
        if (1 == g_200ms_flag)
        {
            g_200ms_flag = 0;
            float batt_last = (float)g_adc_val[0] / 4095 * 3.3 * 1.36;
            batt_volt = batt_aver_filter(batt_last);
//            printf("%.2f\r\n", batt_volt);
            if (batt_volt < BATT_VOLT_LIMIT)
            {
                led_set(LED_RB, TOGGLE);
            }
        }
    }
}
