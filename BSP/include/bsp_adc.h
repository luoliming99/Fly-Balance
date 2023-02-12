#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "ch32f20x.h"


/**
 * \brief ʱ������
 * \note ADC1 - DMA1ͨ��1
 *       ADC3 - DMA2ͨ��5
 */
#define ADC_CLK_CMD         RCC_APB2PeriphClockCmd
#define ADC_CLK             RCC_APB2Periph_ADC1

#define ADC_GPIO_CLK_CMD    RCC_APB2PeriphClockCmd
#define ADC_GPIO_CLK        RCC_APB2Periph_GPIOB

#define ADC_DMA_CLK_CMD     RCC_AHBPeriphClockCmd
#define ADC_DMA_CLK         RCC_AHBPeriph_DMA1

/* GPIO���� */
#define ADC_PORT    GPIOB

#define ADC_PIN1    GPIO_Pin_1
#define ADC_CH1     ADC_Channel_9

/* DMA���� */
#define ADC_DMA_CH  DMA1_Channel1
#define ADC_x       ADC1    /* ADC������1 */
#define ADC_CH_NUM  1       /* ADCת��ͨ������ */

extern uint16_t g_adc_val[];

void adc_init(void);
float get_batt_volt(void);
    
#endif
