#ifndef __FILTER_H
#define __FILTER_H

#include "common.h"

#define ACC_SAMP_PERIOD     MPU_READ_DATA_PERIOD    /* ���ٶȼƲ������� */
#define ACC_CUTOFF_FREQ     10                      /* ���ٶȼƽ�ֹƵ�� */

#define GYRO_FILTER_NUM     2

#define SPEED_SAMP_PERIOD   0.02F                   /* ƽ�⳵�ٶȲ������� */
#define SPEED_CUTOFF_FREQ   2                       /* ƽ�⳵�ٶȽ�ֹƵ�� */ 

#define GYROZ_SAMP_PERIOD   0.02F                   /* ƽ�⳵ת���ٶȲ������� */
#define GYROZ_CUTOFF_FREQ   2                       /* ƽ�⳵ת���ٶȽ�ֹƵ�� */

#define BATT_FILTER_NUM     20

void acc_lpf_filter(mpu_result_t *data);
void gyro_aver_filter(mpu_result_t *data);
void gyro_aver_filter(mpu_result_t *data);
void mpu_raw_data_filter(mpu_result_t *data);
float speed_lpf_filter(float x, float y_last);
float gyroz_lpf_filter(float x, float y_last);
float batt_aver_filter(float data_in);


#endif
