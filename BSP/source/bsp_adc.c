#include "bsp_adc.h"

uint16_t g_adc_val[ADC_CH_NUM] = {0};

static void __adc_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    ADC_GPIO_CLK_CMD(ADC_GPIO_CLK, ENABLE); /* ʹ��GPIOʱ�� */
    
    GPIO_InitStructure.GPIO_Pin     = ADC_PIN1;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(ADC_PORT, &GPIO_InitStructure);
}

static void __adc_mode_config(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};    /* ���ﲻ��ʼ��Ϊ0���Һ��治��ֵDMA_InitStructure.DMA_Priority������⣡ */
	ADC_InitTypeDef ADC_InitStructure = {0};
    
    ADC_DMA_CLK_CMD(ADC_DMA_CLK, ENABLE);   /* ʹ��DMAʱ�� */
    ADC_CLK_CMD(ADC_CLK, ENABLE);           /* ʹ��ADCʱ�� */
    
    /* DMA���� */
    DMA_DeInit(ADC_DMA_CH);                                                     /* ��λDMA������ */
    
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          /* DMA���� */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                /* ��ֹ�洢�����洢��ģʽ */
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC_x->RDATAR));    /* DMAԴ��ַ */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* �������ݴ�СΪ���� */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            /* Դ��ַ������ */
    
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_adc_val;                 /* DMAĿ���ַ */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         /* �ڴ����ݴ�СΪ���� */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     /* Ŀ���ַ���� */
    
    DMA_InitStructure.DMA_BufferSize = ADC_CH_NUM;                              /* DMA��������С������Ŀ���ַ��С */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	                            /* ѭ������ģʽ */
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         /* DMA����ͨ�����ȼ�Ϊ�� */
    DMA_Init(ADC_DMA_CH, &DMA_InitStructure);                                   /* ��ʼ��DMA */
    DMA_Cmd(ADC_DMA_CH, ENABLE);                                                /* ʹ��DMAͨ�� */
    
    /* ADC���� */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                          /* ֻʹ��һ��ADC�����������ڶ���ģʽ */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                /* ʹ��ɨ��ģʽ */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                          /* ʹ������ת��ģʽ */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;         /* �����ⲿ����ת������������ */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      /* ת������Ҷ��� */
    ADC_InitStructure.ADC_NbrOfChannel = ADC_CH_NUM;                            /* ת��ͨ������ */
    ADC_Init(ADC_x, &ADC_InitStructure);                                        /* ��ʼ��ADC */
    
    /*
     * ���� = fpclk / 8 = 9MHz
     * ת��ʱ�� = (����ʱ�� + 12.5)������
     *          = (55.5 + 12.5) * 1 / 9 (us)
     *          = 7.6 (us)
     */
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    /* ����ADCͨ��ת��˳��Ͳ���ʱ�� */
    ADC_RegularChannelConfig(ADC_x, ADC_CH1, 1, ADC_SampleTime_55Cycles5);
    
    ADC_DMACmd(ADC_x, ENABLE);                      /* ʹ��ADC DMA���� */
    ADC_Cmd(ADC_x, ENABLE);                         /* ʹ��ADCģ�� */
 
    ADC_ResetCalibration(ADC_x);                    /* ��ʼ��ADCУ׼�Ĵ��� */
	while(ADC_GetResetCalibrationStatus(ADC_x));    /* �ȴ�У׼�Ĵ�����ʼ����� */
    ADC_StartCalibration(ADC_x);                    /* ADC��ʼУ׼ */
    while(ADC_GetCalibrationStatus(ADC_x));         /* �ȴ�У׼��� */
     
	ADC_SoftwareStartConvCmd(ADC_x, ENABLE);        /* ��������ADCת�� */
}

/******************************************************************************/
void adc_init(void)
{
    __adc_gpio_config();
    __adc_mode_config(); 
}