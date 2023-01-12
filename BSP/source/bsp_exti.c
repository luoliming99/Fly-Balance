#include "bsp_exti.h"

static void __exti_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* MPU中断优先级配置 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI_MPU_INT_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 平衡车左电机编码器中断优先级配置 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI_MOTOR_L_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
    
    /* 平衡车右电机编码器中断优先级配置 */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI_MOTOR_R_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_Init(&NVIC_InitStructure);
}

static void __exti_gpio_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0}; 

	/* 使能相关时钟 */
	EXTI_GPIO_CLK_CMD(EXTI_GPIO_CLK, ENABLE);
	
    /* MPU中断引脚配置 */
    GPIO_InitStructure.GPIO_Pin = EXTI_MPU_INT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(EXTI_MPU_INT_PORT, &GPIO_InitStructure);
    
    /* 平衡车左电机编码器引脚配置 */
    GPIO_InitStructure.GPIO_Pin = EXTI_MOTOR_L_PIN;
    GPIO_Init(EXTI_MOTOR_L_PORT, &GPIO_InitStructure);
    
    /* 平衡车右电机编码器引脚配置 */
    GPIO_InitStructure.GPIO_Pin = EXTI_MOTOR_R_PIN;
    GPIO_Init(EXTI_MOTOR_R_PORT, &GPIO_InitStructure);
}

static void __exti_mode_config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};

    /* MPU中断模式配置 */
    GPIO_EXTILineConfig(EXTI_MPU_INT_PORTSOURCE, EXTI_MPU_INT_PINSOURCE);   /* 配置信号源 */
    EXTI_InitStructure.EXTI_Line = EXTI_MPU_INT_LINE;                       /* 配置中断线 */
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;                     /* 配置为中断模式 */
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;                 /* 下降沿中断 */
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;                               /* 使能中断 */
    EXTI_Init(&EXTI_InitStructure);
    
    /* 平衡车左电机编码器中断模式配置 */
    GPIO_EXTILineConfig(EXTI_MOTOR_L_PORTSOURCE, EXTI_MOTOR_L_PINSOURCE);   /* 配置信号源 */
    EXTI_InitStructure.EXTI_Line = EXTI_MOTOR_L_LINE;                       /* 配置中断线 */
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                               /* 使能中断 */
    EXTI_Init(&EXTI_InitStructure);
    
    /* 平衡车右电机编码器中断模式配置 */
    GPIO_EXTILineConfig(EXTI_MOTOR_R_PORTSOURCE, EXTI_MOTOR_R_PINSOURCE);   /* 配置信号源 */
    EXTI_InitStructure.EXTI_Line = EXTI_MOTOR_R_LINE;                       /* 配置中断线 */
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                               /* 使能中断 */
    EXTI_Init(&EXTI_InitStructure);
}

/******************************************************************************/
void exti_init(void)
{
    __exti_nvic_config();
    __exti_gpio_config();
    __exti_mode_config();
}

