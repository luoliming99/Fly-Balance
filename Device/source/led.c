#include "led.h"

/******************************************************************************/
void led_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 

	/* 使能相关时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = LED_LF_PIN | LED_RF_PIN | 
                                  LED_LB_PIN | LED_RB_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    
    led_set(LED_LF, OFF);
    led_set(LED_RF, OFF);
    led_set(LED_LB, OFF);
    led_set(LED_RB, OFF);
}

/******************************************************************************/
void led_set(which_led_e led, led_status_e status)
{
    switch (led)
    {
        case LED_LF:
            if (status == ON)
                GPIO_ResetBits(LED_PORT, LED_LF_PIN);
            else if (status == OFF)
                GPIO_SetBits(LED_PORT, LED_LF_PIN);
            else if (status == TOGGLE)
                GPIO_WriteBit(LED_PORT, LED_LF_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_PORT, LED_LF_PIN)));
        break;
            
        case LED_RF:
            if (status == ON)
                GPIO_ResetBits(LED_PORT, LED_RF_PIN);
            else if (status == OFF)
                GPIO_SetBits(LED_PORT, LED_RF_PIN);
            else if (status == TOGGLE)
                GPIO_WriteBit(LED_PORT, LED_RF_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_PORT, LED_RF_PIN)));
        break;
            
        case LED_LB:
            if (status == ON)
                GPIO_ResetBits(LED_PORT, LED_LB_PIN);
            else if (status == OFF)
                GPIO_SetBits(LED_PORT, LED_LB_PIN);
            else if (status == TOGGLE)
                GPIO_WriteBit(LED_PORT, LED_LB_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_PORT, LED_LB_PIN)));
        break;
            
        case LED_RB:
            if (status == ON)
                GPIO_ResetBits(LED_PORT, LED_RB_PIN);
            else if (status == OFF)
                GPIO_SetBits(LED_PORT, LED_RB_PIN);
            else if (status == TOGGLE)
                GPIO_WriteBit(LED_PORT, LED_RB_PIN, (BitAction)(1 - GPIO_ReadOutputDataBit(LED_PORT, LED_RB_PIN)));
        break;
            
        default: break;
    }
}
