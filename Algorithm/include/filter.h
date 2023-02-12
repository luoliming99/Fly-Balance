#ifndef __FILTER_H
#define __FILTER_H

#include "ctype.h"
#include "mpu6500.h"

#define IIR_ACC_SAMP_PERIOD     MPU_READ_DATA_PERIOD    /* 加速度计采样周期 */
#define IIR_ACC_CUTOFF_FREQ     10                      /* 加速度计IIR滤波器截止频率 */

#define GYRO_FILTER_NUM     2

#define BATT_FILTER_NUM     20

#define SPEED_FILTER_NUM    10
#define GYROZ_FILTER_NUM    10

void iir_acc_filter(mpu_result_t *data_in, mpu_result_t *data_out);
float lpf_speed_filter(float x, float y_last);
void aver_gyro_filter(mpu_result_t *data_in, mpu_result_t *data_out);
void mpu_raw_data_filter(mpu_result_t *data_in, mpu_result_t *data_out);
float aver_batt_filter(float data_in);
float aver_speed_filter(float data_in);
float aver_gyroz_filter(int16_t data_in);

#endif
