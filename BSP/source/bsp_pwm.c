#include "bsp_pwm.h"

static void __pwm_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    PWM_GPIO_CLK_CMD(PWM_GPIO_CLK, ENABLE);     /* 使能GPIO时钟 */
    
    GPIO_InitStructure.GPIO_Pin     = PWM_CH1_PIN;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(PWM_CH_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin     = PWM_CH2_PIN;
    GPIO_Init(PWM_CH_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin     = PWM_CH3_PIN;
    GPIO_Init(PWM_CH_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin     = PWM_CH4_PIN;
    GPIO_Init(PWM_CH_PORT, &GPIO_InitStructure);
}

static void __pwm_mode_config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    
    PWM_CLK_CMD(PWM_CLK, ENABLE);               /* 使能PWM时钟 */

    /* 
     * PWM频率 = 144M / (TIM_Prescaler + 1) / (TIM_Period + 1)
     *         = 16kHz
     */
    TIM_TimeBaseStructure.TIM_Period = 999;
    TIM_TimeBaseStructure.TIM_Prescaler = 8;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(PWM_TIM_x, &TIM_TimeBaseStructure);
    
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    /* 配置输出比较通道1 */    
    TIM_OC1Init(PWM_TIM_x, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(PWM_TIM_x, TIM_OCPreload_Enable);
    
    /* 配置输出比较通道2 */
    TIM_OC2Init(PWM_TIM_x, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(PWM_TIM_x, TIM_OCPreload_Enable);
    
    /* 配置输出比较通道3 */
    TIM_OC3Init(PWM_TIM_x, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(PWM_TIM_x, TIM_OCPreload_Enable);
    
    /* 配置输出比较通道4 */
    TIM_OC4Init(PWM_TIM_x, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(PWM_TIM_x, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(PWM_TIM_x, ENABLE);  /* 使能自动重装载预装载 */
    TIM_Cmd(PWM_TIM_x, ENABLE);
}

/******************************************************************************/
void pwm_init(void)
{
    __pwm_gpio_config();
    __pwm_mode_config(); 
}
