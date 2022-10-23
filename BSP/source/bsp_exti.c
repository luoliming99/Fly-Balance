#include "bsp_exti.h"

static void __exti_nvic_config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    /* �����ж�Դ */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI_IRQ;
    /* �������ȼ�*/
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    /* �����ȼ� */
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    /* ʹ���ж� */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* ��ʼ������NVIC */
    NVIC_Init(&NVIC_InitStructure);
}

static void __exti_gpio_config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0}; 

	/* ʹ�����ʱ�� */
	EXTI_GPIO_CLK_CMD(EXTI_GPIO_CLK, ENABLE);
	
    /* GPIO���� */
    GPIO_InitStructure.GPIO_Pin = EXTI_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(EXTI_PORT, &GPIO_InitStructure);
}

static void __exti_mode_config(void)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};

    GPIO_EXTILineConfig(EXTI_PORTSOURCE, EXTI_PINSOURCE);   /* ѡ��EXTI���ź�Դ */
    
    EXTI_InitStructure.EXTI_Line = EXTI_LINE;               /* �����ⲿ�ж��� */
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;     /* EXTIΪ�ж�ģʽ */
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; /* �½����ж� */
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;               /* ʹ���ж� */
    EXTI_Init(&EXTI_InitStructure);
}

/******************************************************************************/
void exti_init(void)
{
    __exti_nvic_config();
    __exti_gpio_config();
    __exti_mode_config();
}

