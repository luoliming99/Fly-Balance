#ifndef __ENCODER_H
#define __ENCODER_H

#include "ch32f20x.h"

#define ENCODER_PORT    GPIOA   
#define ENCODER_PIN     GPIO_Pin_10

typedef enum
{
    DIR_POS,  /* ��ת */
    DIR_NEG   /* ��ת */
} encoder_dir_e;


void encoder_init(void);
encoder_dir_e encoder_dir_get(void);
void encoder_cnt_inc(int8_t cnt);
void encoder_cnt_clr(void);
void encoder_speed_calc(void);
int32_t encoder_speed_get(void);

#endif
