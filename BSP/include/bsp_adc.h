#ifndef __BSP_ADC_H
#define __BSP_ADC_H

#include "ch32f20x.h"


/**
 * \brief 时钟配置
 * \note ADC1 - DMA1通道1
 *       ADC3 - DMA2通道5
 */
#define ADC_CLK_CMD         RCC_APB2PeriphClockCmd
#define ADC_CLK             RCC_APB2Periph_ADC1

#define ADC_GPIO_CLK_CMD    RCC_APB2PeriphClockCmd
#define ADC_GPIO_CLK        RCC_APB2Periph_GPIOB

#define ADC_DMA_CLK_CMD     RCC_AHBPeriphClockCmd
#define ADC_DMA_CLK         RCC_AHBPeriph_DMA1

/* GPIO配置 */
#define ADC_PORT    GPIOB

#define ADC_PIN1    GPIO_Pin_1
#define ADC_CH1     ADC_Channel_9

/* DMA配置 */
#define ADC_DMA_CH  DMA1_Channel1
#define ADC_x       ADC1    /* ADC控制器1 */
#define ADC_CH_NUM  1       /* ADC转换通道个数 */

extern uint16_t g_adc_val[];

void adc_init(void);
float get_batt_volt(void);
    
#endif
