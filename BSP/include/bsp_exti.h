#ifndef __BSP_EXTI_H
#define __BSP_EXTI_H

#include "ch32f20x.h"

/**
 * \brief  ±÷”≈‰÷√
 */
#define EXTI_GPIO_CLK_CMD   RCC_APB2PeriphClockCmd
#define EXTI_GPIO_CLK       (RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO)

#define EXTI_PORT           GPIOA
#define EXTI_PIN            GPIO_Pin_11

#define EXTI_PORTSOURCE     GPIO_PortSourceGPIOA
#define EXTI_PINSOURCE      GPIO_PinSource11
#define EXTI_LINE           EXTI_Line11

#define EXTI_IRQ            EXTI15_10_IRQn
#define EXTI_IRQHandler     EXTI15_10_IRQHandler


void exti_init(void);

#endif
