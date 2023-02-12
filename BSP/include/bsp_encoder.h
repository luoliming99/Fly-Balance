#ifndef __BSP_ENCODER_H
#define __BSP_ENCODER_H

#include "ch32f20x.h"

/**
 * \brief “˝Ω≈≈‰÷√
 */
#define  ENCODER_L_GPIO_CLK_CMD     RCC_APB2PeriphClockCmd
#define  ENCODER_L_GPIO_CLK         RCC_APB2Periph_GPIOA

#define  ENCODER_L_GPIO_PORT        GPIOA
#define  ENCODER_L_GPIO_PIN         (GPIO_Pin_6 | GPIO_Pin_7)

#define  ENCODER_R_GPIO_CLK_CMD     RCC_APB2PeriphClockCmd
#define  ENCODER_R_GPIO_CLK         RCC_APB2Periph_GPIOB

#define  ENCODER_R_GPIO_PORT        GPIOB
#define  ENCODER_R_GPIO_PIN         (GPIO_Pin_6 | GPIO_Pin_7)

/**
 * \brief ∂® ±∆˜≈‰÷√
 */
#define ENCODER_L_TIM_CLK_CMD       RCC_APB1PeriphClockCmd
#define ENCODER_L_TIM_CLK           RCC_APB1Periph_TIM3

#define ENCODER_L_TIM_x             TIM3

#define ENCODER_L_TIM_IRQ           TIM3_IRQn
#define ENCODER_L_TIM_IRQHandler    TIM3_IRQHandler

#define ENCODER_R_TIM_CLK_CMD       RCC_APB1PeriphClockCmd
#define ENCODER_R_TIM_CLK           RCC_APB1Periph_TIM4

#define ENCODER_R_TIM_x             TIM4

#define ENCODER_R_TIM_IRQ           TIM4_IRQn
#define ENCODER_R_TIM_IRQHandler    TIM4_IRQHandler

void encoder_l_tim_init(void);
void encoder_r_tim_init(void);

#endif
