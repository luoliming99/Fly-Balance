#include "charge.h"


/******************************************************************************/
void charge_manager_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure; 

	/* ʹ�����ʱ�� */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
    /*
     * GPIO����
     * CH_STAT_PIN����Ϊ�������룬ʹ���ⲿ�жϼ��
     * CH_VOLT_SW_PIN����Ϊ�����������ʼ����͵�ƽ
     */
    GPIO_InitStructure.GPIO_Pin = CH_STAT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(CH_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = CH_VOLT_SW_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(CH_PORT, &GPIO_InitStructure);

    GPIO_SetBits(CH_PORT, CH_VOLT_SW_PIN);
}

/******************************************************************************/
charge_status_e charge_status_get(void)
{
    return (charge_status_e)GPIO_ReadInputDataBit(CH_PORT, CH_STAT_PIN);
}
