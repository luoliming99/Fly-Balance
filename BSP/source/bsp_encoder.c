#include "bsp_encoder.h"

void encoder_l_tim_init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0}; 
    
    /* 时钟使能 */
    ENCODER_L_GPIO_CLK_CMD(ENCODER_L_GPIO_CLK, ENABLE);
    ENCODER_L_TIM_CLK_CMD(ENCODER_L_TIM_CLK, ENABLE);

    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = ENCODER_L_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENCODER_L_GPIO_PORT, &GPIO_InitStructure);
    
    /* 定时器模式配置 */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_L_TIM_x, &TIM_TimeBaseStructure);

    /* 编码器参数配置 */
    TIM_EncoderInterfaceConfig(ENCODER_L_TIM_x, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    /* 编码器模式3：CH1、CH2同时计数，四分频 */
    
    /* 捕获滤波配置 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100：采样频率 Fsampling=Fdts/16，N=8 */
    TIM_ICInit(ENCODER_L_TIM_x, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100：采样频率 Fsampling=Fdts/16，N=8 */
    TIM_ICInit(ENCODER_L_TIM_x, &TIM_ICInitStructure);
    
    ENCODER_L_TIM_x->CNT = 0x7FFF;  /* 为了测量正反转，设置初始值为0x7FFF */

    TIM_Cmd(ENCODER_L_TIM_x, ENABLE);
}

void encoder_r_tim_init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0}; 
    
    /* 时钟使能 */
    ENCODER_R_GPIO_CLK_CMD(ENCODER_R_GPIO_CLK, ENABLE);
    ENCODER_R_TIM_CLK_CMD(ENCODER_R_TIM_CLK, ENABLE);

    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = ENCODER_R_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENCODER_R_GPIO_PORT, &GPIO_InitStructure);
    
    /* 定时器模式配置 */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_R_TIM_x, &TIM_TimeBaseStructure);

    /* 编码器参数配置 */
    TIM_EncoderInterfaceConfig(ENCODER_R_TIM_x, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    /* 编码器模式3：CH1、CH2同时计数，四分频 */
    
    /* 捕获滤波配置 */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100：采样频率 Fsampling=Fdts/16，N=8 */
    TIM_ICInit(ENCODER_R_TIM_x, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100：采样频率 Fsampling=Fdts/16，N=8 */
    TIM_ICInit(ENCODER_R_TIM_x, &TIM_ICInitStructure);
    
    ENCODER_R_TIM_x->CNT = 0x7FFF;  /* 为了测量正反转，设置初始值为0x7FFF */

    TIM_Cmd(ENCODER_R_TIM_x, ENABLE);
}
