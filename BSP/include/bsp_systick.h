#ifndef __SYSTICK_H
#define __SYSTICK_H

#include "ch32f20x.h"

extern unsigned long g_systick_cnt;

void systick_init(void);
void delay_ms(unsigned long nms);
void get_ms(unsigned long *nms);
    
#endif
