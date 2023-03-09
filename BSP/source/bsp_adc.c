#include "bsp_adc.h"


uint16_t g_adc_val[ADC_CH_NUM] = {0};
static int16_t _conversion_val = 0;

static uint16_t _get_conversion_val(uint16_t raw_val)
{
    if ((raw_val + _conversion_val) < 0) return 0;
	if ((raw_val + _conversion_val) > 4095) return 4095;
	return (raw_val + _conversion_val);
}

static void _adc_gpio_config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    ADC_GPIO_CLK_CMD(ADC_GPIO_CLK, ENABLE); /* 使能GPIO时钟 */
    
    GPIO_InitStructure.GPIO_Pin     = ADC_PIN1;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AIN;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(ADC_PORT, &GPIO_InitStructure);
}

static void _adc_mode_config(void)
{
    DMA_InitTypeDef DMA_InitStructure = {0};    /* 这里不初始化为0，且后面不赋值DMA_InitStructure.DMA_Priority会出问题！ */
	ADC_InitTypeDef ADC_InitStructure = {0};
    
    ADC_DMA_CLK_CMD(ADC_DMA_CLK, ENABLE);   /* 使能DMA时钟 */
    ADC_CLK_CMD(ADC_CLK, ENABLE);           /* 使能ADC时钟 */
    
    /* DMA配置 */
    DMA_DeInit(ADC_DMA_CH);                                                     /* 复位DMA控制器 */
    
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                          /* DMA方向 */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;                                /* 禁止存储器到存储器模式 */
    
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(ADC_x->RDATAR));    /* DMA源地址 */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; /* 外设数据大小为半字 */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            /* 源地址不递增 */
    
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)g_adc_val;                 /* DMA目标地址 */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         /* 内存数据大小为半字 */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     /* 目标地址递增 */
    
    DMA_InitStructure.DMA_BufferSize = ADC_CH_NUM;                              /* DMA缓存区大小，等于目标地址大小 */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	                            /* 循环传输模式 */
//    DMA_InitStructure.DMA_Priority = DMA_Priority_High;                         /* DMA传输通道优先级为高 */
    DMA_Init(ADC_DMA_CH, &DMA_InitStructure);                                   /* 初始化DMA */
    DMA_Cmd(ADC_DMA_CH, ENABLE);                                                /* 使能DMA通道 */
    
    /* ADC配置 */
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                          /* 只使用一个ADC控制器，属于独立模式 */
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;                                /* 使能扫描模式 */
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;                          /* 使能连续转换模式 */
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;         /* 不用外部触发转换，软件启动 */
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;                      /* 转换结果右对齐 */
    ADC_InitStructure.ADC_NbrOfChannel = ADC_CH_NUM;                            /* 转换通道个数 */
    ADC_Init(ADC_x, &ADC_InitStructure);                                        /* 初始化ADC */
    
    /*
     * 周期 = fpclk / 8 = 9MHz
     * 转换时间 = (采样时间 + 12.5)个周期
     *          = (55.5 + 12.5) * 1 / 9 (us)
     *          = 7.6 (us)
     */
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    /* 设置ADC通道转换顺序和采样时间 */
    ADC_RegularChannelConfig(ADC_x, ADC_CH1, 1, ADC_SampleTime_55Cycles5);
    
    ADC_DMACmd(ADC_x, ENABLE);                      /* 使能ADC DMA请求 */
    ADC_Cmd(ADC_x, ENABLE);                         /* 使能ADC模块 */
 
    ADC_ResetCalibration(ADC_x);                    /* 初始化ADC校准寄存器 */
	while(ADC_GetResetCalibrationStatus(ADC_x));    /* 等待校准寄存器初始化完成 */
    ADC_StartCalibration(ADC_x);                    /* ADC开始校准 */
    while(ADC_GetCalibrationStatus(ADC_x));         /* 等待校准完成 */
    _conversion_val = Get_CalibrationValue(ADC_x);	/* 获取校准值 */
     
	ADC_SoftwareStartConvCmd(ADC_x, ENABLE);        /* 软件触发ADC转换 */
}

/******************************************************************************/
void adc_init(void)
{
    _adc_gpio_config();
    _adc_mode_config(); 
}

/******************************************************************************/
float get_batt_volt(void)
{
    uint16_t adc_val = _get_conversion_val(g_adc_val[0]);
    return (float)adc_val / 4095 * 3.3 * 1.36;
}
