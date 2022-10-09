#ifndef __CHARGE_H
#define __CHARGE_H

#include "ch32f20x.h"

#define CH_PORT         GPIOA
#define CH_STAT_PIN     GPIO_Pin_11
#define CH_VOLT_SW_PIN  GPIO_Pin_12

typedef enum
{
    STAT_CHARGING_END = 0,      /* 充电结束 */
    STAT_CHARGING,              /* 充电中 */
} charge_status_e;

void charge_manager_init(void);
charge_status_e charge_status_get(void);

#endif
