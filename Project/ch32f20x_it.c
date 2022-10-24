/********************************** (C) COPYRIGHT *******************************
* File Name          : ch32f20x_it.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/08/08
* Description        : Main Interrupt Service Routines.
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
#include "ch32f20x_it.h"
#include "bsp_uart.h"
#include "bsp_systick.h"
#include "bsp_tim.h"
#include "bsp_exti.h"
#include "encoder.h"

/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
  while (1)
  {
  }	
}

/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
void HardFault_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      MemManage_Handler
 *
 * @brief   This function handles Memory Manage exception.
 *
 * @return  none
 */
void MemManage_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      BusFault_Handler
 *
 * @brief   This function handles Bus Fault exception.
 *
 * @return  none
 */
void BusFault_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      UsageFault_Handler
 *
 * @brief   This function handles Usage Fault exception.
 *
 * @return  none
 */
void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      SVC_Handler
 *
 * @brief   This function handles SVCall exception.
 *
 * @return  none
 */
void SVC_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      DebugMon_Handler
 *
 * @brief   This function handles Debug Monitor exception.
 *
 * @return  none
 */
void DebugMon_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      PendSV_Handler
 *
 * @brief   This function handles PendSVC exception.
 *
 * @return  none
 */
void PendSV_Handler(void)
{
  while (1)
  {
  }
}

/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   This function handles SysTick Handler.
 *
 * @return  none
 */
void SysTick_Handler(void)
{
  g_systick_cnt++;
}

void DEBUG_USART_IRQHandler(void) 
{
    uint8_t ch;
    
	if (USART_GetITStatus(DEBUG_USART, USART_IT_RXNE) != RESET)
	{		
		ch = USART_ReceiveData(DEBUG_USART);
        USART_SendData(DEBUG_USART, ch);    
	}	 
}

extern uint8_t g_mpu_int;
void EXTI_MPU_INT_IRQHandler(void)
{
	if (EXTI_GetITStatus(EXTI_MPU_INT_LINE) != RESET) 
	{
        g_mpu_int = 1;
        
		EXTI_ClearITPendingBit(EXTI_MPU_INT_LINE);     
	}
}

void EXTI_MOTOR_L_IRQHandler(void)
{
    encoder_dir_e dir;

	if (EXTI_GetITStatus(EXTI_MOTOR_L_LINE) != RESET) 
	{
        dir = encoder_dir_get();
        
        if (dir == DIR_POS)
            encoder_cnt_inc(1);
        else
            encoder_cnt_inc(-1);
        
		EXTI_ClearITPendingBit(EXTI_MOTOR_L_LINE);     
	}
}

void  TIM_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM_x, TIM_IT_Update) != RESET) 
	{	
        encoder_speed_calc();
        encoder_cnt_clr();
        printf("speed=%d (r/min)\r\n", encoder_speed_get());

		TIM_ClearITPendingBit(TIM_x , TIM_FLAG_Update);  		 
	}		 	
}

