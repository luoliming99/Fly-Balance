#ifndef __BSP_TIM_H
#define __BSP_TIM_H

#include "ch32f20x.h"

/**
 * \brief  ±÷”≈‰÷√
 */
#define TIM_CLK_CMD         RCC_APB2PeriphClockCmd
#define TIM_CLK             RCC_APB2Periph_TIM1

#define TIM_x               TIM1

#define TIM_IRQ             TIM1_UP_IRQn
#define TIM_IRQHandler      TIM1_UP_IRQHandler

void tim_init(void);

#endif
