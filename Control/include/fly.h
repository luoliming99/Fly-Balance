#ifndef __FLY_H
#define __FLY_H

#include "ch32f20x.h"
#include "common.h"

/**
 * accelerator: 300 ~ 900
 * pitch_target: -10бу~ 10бу
 * yaw_target: -180бу~ 180бу
 * roll_target: -10бу~ 10бу
 */
void task_fly_pid_control_5ms(uint16_t accelerator, int16_t pitch_target, int16_t yaw_target, int16_t roll_target, mpu_result_t *mpu_data);

int task_fly_communication(unlock_status_e *unlock_status, uint16_t *accelerator, int16_t *pitch_target, int16_t *yaw_target, int16_t *roll_target, 
                        key_status_e *key_val, float batt_volt, mpu_result_t *mpu_data);

void task_fly_recv_data_handler(int16_t *yaw_target, key_status_e key_val);

#endif
