#include "bsp_tim.h"

static void __tim_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* 配置中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = TIM_IRQ;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

static void __pulse_counter_mode_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    
    TIM_CLK_CMD(TIM_CLK, ENABLE);

    /* 
     * 中断频率 = 144M / (TIM_Prescaler + 1) / (TIM_Period + 1)
     *          = 1Hz
     */
	TIM_TimeBaseStructure.TIM_Period = 9999;
	TIM_TimeBaseStructure.TIM_Prescaler = 14399;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM_x, &TIM_TimeBaseStructure);

    TIM_ClearFlag(TIM_x, TIM_FLAG_Update);      /* 清除计数器中断标志位 */
    TIM_ITConfig(TIM_x, TIM_IT_Update, ENABLE); /* 使能计数器中断 */
    
    TIM_Cmd(TIM_x, ENABLE);
}

/******************************************************************************/
void tim_init(void)
{
    __tim_nvic_config();
    __pulse_counter_mode_config(); 
}
