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

uint8_t g_2ms_flag = 0;
uint8_t g_5ms_flag = 0;
uint8_t g_20ms_flag = 0;
uint8_t g_200ms_flag = 0;

void SysTick_Handler(void)
{
    if ((g_systick_cnt % 2) == 0)
    {
        g_2ms_flag = 1;
    }
    if ((g_systick_cnt % 5) == 0)
    {
        g_5ms_flag = 1;
    } 
    if ((g_systick_cnt % 20) == 0)
    {
        g_20ms_flag = 1;
    } 
    if ((g_systick_cnt % 200) == 0)
    {
        g_200ms_flag = 1;
    } 
    g_systick_cnt++;
    if (g_systick_cnt == 1000000)
    {
        g_systick_cnt = 0;
    }
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

void  TIM_IRQHandler(void)
{
//    static uint32_t cnt = 0;
//	if (TIM_GetITStatus(TIM_x, TIM_IT_Update) != RESET) 
//	{	
//        if ((cnt % 2) == 0)
//        {
//            g_2ms_flag = 1;
//        }
//        if ((cnt % 5) == 0)
//        {
//            g_5ms_flag = 1;
//        } 
//        if ((cnt % 10) == 0)
//        {
//            g_10ms_flag = 1;
//        } 
//        if ((cnt % 200) == 0)
//        {
//            g_200ms_flag = 1;
//        }
//        cnt++;
//        if (cnt == 1000000)
//        {
//            cnt = 0;
//        }

//		TIM_ClearITPendingBit(TIM_x , TIM_FLAG_Update);  		 
//	}		 	
}


