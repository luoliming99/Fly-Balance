#ifndef __LED_H
#define __LED_H

#include "ch32f20x.h"

#define LED_PORT    GPIOA   
#define LED_LF_PIN  GPIO_Pin_4
#define LED_RF_PIN  GPIO_Pin_5
#define LED_LB_PIN  GPIO_Pin_6
#define LED_RB_PIN  GPIO_Pin_8

typedef enum
{
    LED_LF = 0, /* 左前方LED */
    LED_RF,     /* 右前方LED */
    LED_LB,     /* 左后方LED */
    LED_RB      /* 右后方LED */
} which_led_e;

typedef enum
{
    ON = 0,
    OFF,
    TOGGLE
} led_status_e;

void led_init(void);
void led_set(which_led_e led, led_status_e status);

#endif
