#include "encoder.h"
#include "bsp_tim.h"
#include "bsp_exti.h"

static int32_t __g_l_pulse_cnt = 0;
static int32_t __g_l_speed = 0;

static int32_t __g_r_pulse_cnt = 0;
static int32_t __g_r_speed = 0;

/******************************************************************************/
void encoder_init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0}; 

	/* 使能相关时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
    /* GPIO配置 */
    GPIO_InitStructure.GPIO_Pin = ENCODER_L_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(ENCODER_L_PORT, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = ENCODER_R_PIN;
    GPIO_Init(ENCODER_R_PORT, &GPIO_InitStructure);
    
    tim_init();
}

/******************************************************************************/
encoder_dir_e encoder_l_dir_get(void)
{
    uint8_t val = GPIO_ReadInputDataBit(ENCODER_L_PORT, ENCODER_L_PIN);
    
    if (val == 0)
        return DIR_NEG;
    else
        return DIR_POS;
}

/******************************************************************************/
encoder_dir_e encoder_r_dir_get(void)
{
    uint8_t val = GPIO_ReadInputDataBit(ENCODER_R_PORT, ENCODER_R_PIN);
    
    if (val == 0)
        return DIR_POS;
    else
        return DIR_NEG;
}

/******************************************************************************/
void encoder_l_cnt_inc(int8_t cnt)
{
    __g_l_pulse_cnt += cnt;
}

/******************************************************************************/
void encoder_r_cnt_inc(int8_t cnt)
{
    __g_r_pulse_cnt += cnt;
}

/******************************************************************************/
void encoder_l_cnt_clr(void)
{
    __g_l_pulse_cnt = 0;
}

/******************************************************************************/
void encoder_r_cnt_clr(void)
{
    __g_r_pulse_cnt = 0;
}

/******************************************************************************/
void encoder_l_speed_calc(void)
{
    __g_l_speed = __g_l_pulse_cnt * 60 * 100 / 528;   /* 单位：r/min */
}

/******************************************************************************/
void encoder_r_speed_calc(void)
{
    __g_r_speed = __g_r_pulse_cnt * 60 * 100 / 528;   /* 单位：r/min */
}

/******************************************************************************/
int32_t encoder_l_speed_get(void)
{
    return __g_l_speed;   /* 单位：r/min */
}

/******************************************************************************/
int32_t encoder_r_speed_get(void)
{
    return __g_r_speed;   /* 单位：r/min */
}
