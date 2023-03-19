#ifndef __FILTER_H
#define __FILTER_H

#include "common.h"

#define IIR_SAMP_PERIOD     MPU_READ_DATA_PERIOD    /* 采样周期 */
#define IIR_CUTOFF_FREQ     10                      /* 截止频率 */        

#define GYRO_FILTER_NUM     2

#define BATT_FILTER_NUM     20

#define SPEED_FILTER_NUM    10
#define GYROZ_FILTER_NUM    10

void acc_iir_filter(mpu_result_t *data_in, mpu_result_t *data_out);
void gyro_aver_filter(mpu_result_t *data_in, mpu_result_t *data_out);
void mpu_raw_data_filter(mpu_result_t *data_in, mpu_result_t *data_out);
float batt_aver_filter(float data_in);
float aver_speed_filter(float data_in);
float aver_gyroz_filter(int16_t data_in);

#endif
