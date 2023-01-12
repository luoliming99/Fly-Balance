#ifndef __ENCODER_H
#define __ENCODER_H

#include "ch32f20x.h"

#define ENCODER_L_PORT      GPIOA   
#define ENCODER_L_PIN       GPIO_Pin_12

#define ENCODER_R_PORT      GPIOB   
#define ENCODER_R_PIN       GPIO_Pin_4

typedef enum
{
    DIR_POS,  /* 正转 */
    DIR_NEG   /* 反转 */
} encoder_dir_e;


void encoder_init(void);
encoder_dir_e encoder_l_dir_get(void);
encoder_dir_e encoder_r_dir_get(void);
void encoder_l_cnt_inc(int8_t cnt);
void encoder_r_cnt_inc(int8_t cnt);
void encoder_l_cnt_clr(void);
void encoder_r_cnt_clr(void);
void encoder_l_speed_calc(void);
void encoder_r_speed_calc(void);
int32_t encoder_l_speed_get(void);
int32_t encoder_r_speed_get(void);

#endif
