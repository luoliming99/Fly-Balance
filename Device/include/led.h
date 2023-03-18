#ifndef __LED_H
#define __LED_H

#include "ch32f20x.h"

#define LED_PORT1   GPIOA
#define LED_PORT2   GPIOB

#define LED_LF_PIN  GPIO_Pin_4
#define LED_RF_PIN  GPIO_Pin_5
#define LED_LB_PIN  GPIO_Pin_0  /* PB0 */
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
