#ifndef __CAR_H
#define __CAR_H

#include "ch32f20x.h"
#include "common.h"
#include "motor.h"

void task_car_pid_control_5ms(float angle_measure, motor_status_e *motor_status);
void task_car_pid_control_20ms(int16_t speed_target, int16_t turn_target, float speed_measure, float gyroz);
int task_car_communication(unlock_status_e *unlock_status, int16_t *speed_target, int16_t *turn_target,
                            mpu_result_t *p_data, float batt_volt);

#endif
