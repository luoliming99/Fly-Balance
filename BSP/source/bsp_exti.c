#include "bsp_exti.h"

static void __exti_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* 配置中断源 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ;
    /* 抢断优先级*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* 子优先级 */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    /* 使能中断 */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* 初始化配置NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

static void __exti_gpio_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0}; 

	/* 使能相关时钟 */
	EXTI_GPIO_CLK_CMD(EXTI_GPIO_CLK, ENABLE);
	
    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = EXTI_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(EXTI_PORT, &GPIO_InitStructure);
}

static void __exti_mode_config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};

    GPIO_EXTILineConfig(EXTI_PORTSOURCE, EXTI_PINSOURCE);   /* 选择EXTI的信号源 */
    
    EXTI_InitStructure.EXTI_Line = EXTI_LINE;               /* 配置外部中断线 */
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     /* EXTI为中断模式 */
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; /* 下降沿中断 */
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;               /* 使能中断 */
    EXTI_Init(&EXTI_InitStructure);
}

/******************************************************************************/
void exti_init(void)
{
    __exti_nvic_config();
    __exti_gpio_config();
    __exti_mode_config();
}

