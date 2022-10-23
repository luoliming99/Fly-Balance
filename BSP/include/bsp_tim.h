#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "ch32f20x.h"

/**
 * \brief  ±÷”≈‰÷√
 */
#define TIM_CLK_CMD         RCC_APB1PeriphClockCmd
#define TIM_CLK             RCC_APB1Periph_TIM4

#define TIM_x               TIM4

#define TIM_IRQ             TIM4_IRQn
#define TIM_IRQHandler      TIM4_IRQHandler

void tim_init(void);

#endif
