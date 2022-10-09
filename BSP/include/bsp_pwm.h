#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "ch32f20x.h"

#define PWM_CLK_CMD         RCC_APB1PeriphClockCmd
#define PWM_CLK             RCC_APB1Periph_TIM2

#define PWM_GPIO_CLK_CMD    RCC_APB2PeriphClockCmd
#define PWM_GPIO_CLK        RCC_APB2Periph_GPIOA

#define PWM_TIM_x       TIM2
#define PWM_CH_PORT     GPIOA
#define PWM_CH1_PIN     GPIO_Pin_0
#define PWM_CH2_PIN     GPIO_Pin_1
#define PWM_CH3_PIN     GPIO_Pin_2
#define PWM_CH4_PIN     GPIO_Pin_3

#define PWM_CH1_SET(x)  (PWM_TIM_x->CH1CVR = x)
#define PWM_CH2_SET(x)  (PWM_TIM_x->CH2CVR = x)
#define PWM_CH3_SET(x)  (PWM_TIM_x->CH3CVR = x)
#define PWM_CH4_SET(x)  (PWM_TIM_x->CH4CVR = x)

void pwm_init(void);

#endif
