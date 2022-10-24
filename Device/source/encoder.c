#include "encoder.h"
#include "bsp_tim.h"
#include "bsp_exti.h"

static int32_t __g_pulse_cnt = 0;
static int32_t __g_speed = 0;

/******************************************************************************/
void encoder_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0}; 

	/* 使能相关时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = ENCODER_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ENCODER_PORT, &GPIO_InitStructure);
    
    tim_init();
}

/******************************************************************************/
encoder_dir_e encoder_dir_get(void)
{
    uint8_t val = GPIO_ReadInputDataBit(ENCODER_PORT, ENCODER_PIN);
    
    if (val == 0)
        return DIR_POS;
    else
        return DIR_NEG;
}

/******************************************************************************/
void encoder_cnt_inc(int8_t cnt)
{
    __g_pulse_cnt += cnt;
}

/******************************************************************************/
void encoder_cnt_clr(void)
{
    __g_pulse_cnt = 0;
}

/******************************************************************************/
void encoder_speed_calc(void)
{
    __g_speed = __g_pulse_cnt * 60 / 528;   /* 单位：r/min */
}

/******************************************************************************/
int32_t encoder_speed_get(void)
{
    return __g_speed;   /* 单位：r/min */
}
