#ifndef __BSP_IWDG_H
#define __BSP_IWDG_H

#include "ch32f20x.h"

void iwdg_feed_init(uint16_t prer, uint16_t rlr);

void comm_wdg_enable(void);

void comm_wdg_disable(void);

void comm_wdg_feed(void);

uint8_t is_comm_wdg_interrupted(void);

#endif
