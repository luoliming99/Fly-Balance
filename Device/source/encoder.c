#include "encoder.h"
#include "bsp_encoder.h"


static int16_t _g_l_pulse_cnt = 0;
static int16_t _g_r_pulse_cnt = 0;


/******************************************************************************/
void encoder_l_cnt_clr(void)
{
    ENCODER_L_TIM_x->CNT = 0x7FFF;
}

/******************************************************************************/
void encoder_r_cnt_clr(void)
{
    ENCODER_R_TIM_x->CNT = 0x7FFF;
}

/******************************************************************************/
void encoder_l_cnt_get(void)
{
    _g_l_pulse_cnt = ENCODER_L_TIM_x->CNT;
    
    _g_l_pulse_cnt = _g_l_pulse_cnt - 0x7FFF;
}

/******************************************************************************/
void encoder_r_cnt_get(void)
{
    _g_r_pulse_cnt = ENCODER_R_TIM_x->CNT;
    
    _g_r_pulse_cnt = 0x7FFF - _g_r_pulse_cnt;
}

/******************************************************************************/
int16_t encoder_l_speed_get(void)
{
    return _g_l_pulse_cnt * 60 * 50 / (528 * 4);   /* 单位：r/min */
}

/******************************************************************************/
int16_t encoder_r_speed_get(void)
{
    return _g_r_pulse_cnt * 60 * 50 / (528 * 4);   /* 单位：r/min */
}
