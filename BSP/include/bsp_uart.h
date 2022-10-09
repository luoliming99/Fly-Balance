#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "ch32f20x.h"
#include <stdio.h>

// 串口1-USART1
#define  DEBUG_USART                    USART1
#define  DEBUG_USART_CLK                RCC_APB2Periph_USART1
#define  DEBUG_USART_APBxClkCmd         RCC_APB2PeriphClockCmd
#define  DEBUG_USART_BAUDRATE           500000

// USART GPIO 引脚宏定义
#define  DEBUG_USART_GPIO_CLK           RCC_APB2Periph_GPIOA
#define  DEBUG_USART_GPIO_APBxClkCmd    RCC_APB2PeriphClockCmd
    
#define  DEBUG_USART_TX_GPIO_PORT       GPIOA
#define  DEBUG_USART_TX_GPIO_PIN        GPIO_Pin_9
#define  DEBUG_USART_RX_GPIO_PORT       GPIOA
#define  DEBUG_USART_RX_GPIO_PIN        GPIO_Pin_10

#define  DEBUG_USART_IRQ                USART1_IRQn
#define  DEBUG_USART_IRQHandler         USART1_IRQHandler

void uart_init(void);
void uart_send_one_byte(USART_TypeDef *pUSARTx, uint8_t ch);
void uart_send_string(USART_TypeDef *pUSARTx, char *str);
void uart_send_data_buf(USART_TypeDef *pUSARTx, uint8_t *pBuf, uint32_t len);

#endif
