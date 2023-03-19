#ifndef __BALANCE_H
#define __BALANCE_H

#include "ch32f20x.h"
#include "common.h"

void task_balance_pid_control_5ms(float angle_measure);
void task_balance_pid_control_20ms(int16_t speed_target, int16_t turn_target, float speed_measure, float gyroz);
int task_balance_communication(int16_t *speed_target, int16_t *turn_target, float batt_volt, float speed_measure);
void task_balance_recv_data_handler(unlock_status_e *unlock_status, int16_t *speed_target, int16_t *turn_target);

#endif
