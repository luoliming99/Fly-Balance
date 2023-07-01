#include "bsp_encoder.h"

void encoder_l_tim_init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0}; 
    
    /* ʱ��ʹ�� */
    ENCODER_L_GPIO_CLK_CMD(ENCODER_L_GPIO_CLK, ENABLE);
    ENCODER_L_TIM_CLK_CMD(ENCODER_L_TIM_CLK, ENABLE);

    /* GPIO���� */
    GPIO_InitStructure.GPIO_Pin = ENCODER_L_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENCODER_L_GPIO_PORT, &GPIO_InitStructure);
    
    /* ��ʱ��ģʽ���� */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_L_TIM_x, &TIM_TimeBaseStructure);

    /* �������������� */
    TIM_EncoderInterfaceConfig(ENCODER_L_TIM_x, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    /* ������ģʽ3��CH1��CH2ͬʱ�������ķ�Ƶ */
    
    /* �����˲����� */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100������Ƶ�� Fsampling=Fdts/16��N=8 */
    TIM_ICInit(ENCODER_L_TIM_x, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100������Ƶ�� Fsampling=Fdts/16��N=8 */
    TIM_ICInit(ENCODER_L_TIM_x, &TIM_ICInitStructure);
    
    ENCODER_L_TIM_x->CNT = 0x7FFF;  /* Ϊ�˲�������ת�����ó�ʼֵΪ0x7FFF */

    TIM_Cmd(ENCODER_L_TIM_x, ENABLE);
}

void encoder_r_tim_init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure = {0};
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
    TIM_ICInitTypeDef       TIM_ICInitStructure = {0}; 
    
    /* ʱ��ʹ�� */
    ENCODER_R_GPIO_CLK_CMD(ENCODER_R_GPIO_CLK, ENABLE);
    ENCODER_R_TIM_CLK_CMD(ENCODER_R_TIM_CLK, ENABLE);

    /* GPIO���� */
    GPIO_InitStructure.GPIO_Pin = ENCODER_R_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(ENCODER_R_GPIO_PORT, &GPIO_InitStructure);
    
    /* ��ʱ��ģʽ���� */
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(ENCODER_R_TIM_x, &TIM_TimeBaseStructure);

    /* �������������� */
    TIM_EncoderInterfaceConfig(ENCODER_R_TIM_x, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);    /* ������ģʽ3��CH1��CH2ͬʱ�������ķ�Ƶ */
    
    /* �����˲����� */
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100������Ƶ�� Fsampling=Fdts/16��N=8 */
    TIM_ICInit(ENCODER_R_TIM_x, &TIM_ICInitStructure);
    
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x0C; /* 1100������Ƶ�� Fsampling=Fdts/16��N=8 */
    TIM_ICInit(ENCODER_R_TIM_x, &TIM_ICInitStructure);
    
    ENCODER_R_TIM_x->CNT = 0x7FFF;  /* Ϊ�˲�������ת�����ó�ʼֵΪ0x7FFF */

    TIM_Cmd(ENCODER_R_TIM_x, ENABLE);
}
