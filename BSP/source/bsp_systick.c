#include "bsp_systick.h"


unsigned long g_systick_cnt = 0;

/******************************************************************************/
void systick_init(void)
{
    /* 1ms÷–∂œ“ª¥Œ */
    if (SysTick_Config(SystemCoreClock / 1000))
	{ 
		/* Capture error */ 
		while (1);
	}
}

/******************************************************************************/
void delay_ms(unsigned long nms)
{   
    unsigned long t_start = g_systick_cnt;
    while (g_systick_cnt - t_start <= nms && g_systick_cnt >= t_start);
}

/******************************************************************************/
void get_ms(unsigned long *nms)
{
    *nms = g_systick_cnt;
}

