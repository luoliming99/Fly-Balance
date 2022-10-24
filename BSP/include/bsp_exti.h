#ifndef __BSP_EXTI_H
#define __BSP_EXTI_H

#include "ch32f20x.h"

/**
 * \brief  ±÷”≈‰÷√
 */
#define EXTI_GPIO_CLK_CMD   RCC_APB2PeriphClockCmd
#define EXTI_GPIO_CLK       (RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO)

#define EXTI_MPU_INT_PORT           GPIOB
#define EXTI_MPU_INT_PIN            GPIO_Pin_5
#define EXTI_MPU_INT_PORTSOURCE     GPIO_PortSourceGPIOB
#define EXTI_MPU_INT_PINSOURCE      GPIO_PinSource5
#define EXTI_MPU_INT_LINE           EXTI_Line5
#define EXTI_MPU_INT_IRQ            EXTI9_5_IRQn
#define EXTI_MPU_INT_IRQHandler     EXTI9_5_IRQHandler

#define EXTI_MOTOR_L_PORT           GPIOA
#define EXTI_MOTOR_L_PIN            GPIO_Pin_11
#define EXTI_MOTOR_L_PORTSOURCE     GPIO_PortSourceGPIOA 
#define EXTI_MOTOR_L_PINSOURCE      GPIO_PinSource11
#define EXTI_MOTOR_L_LINE           EXTI_Line11
#define EXTI_MOTOR_L_IRQ            EXTI15_10_IRQn
#define EXTI_MOTOR_L_IRQHandler     EXTI15_10_IRQHandler


void exti_init(void);

#endif
