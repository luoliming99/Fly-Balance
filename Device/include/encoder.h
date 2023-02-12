#ifndef __ENCODER_H
#define __ENCODER_H

#include "ch32f20x.h"


void encoder_l_cnt_clr(void);
void encoder_r_cnt_clr(void);
void encoder_l_cnt_get(void);
void encoder_r_cnt_get(void);
int16_t encoder_l_speed_get(void);
int16_t encoder_r_speed_get(void);

#endif
