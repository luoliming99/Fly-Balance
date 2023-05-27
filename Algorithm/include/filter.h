#ifndef __FILTER_H
#define __FILTER_H

#include "common.h"

#define ACC_SAMP_PERIOD     MPU_READ_DATA_PERIOD    /* 加速度计采样周期 */
#define ACC_CUTOFF_FREQ     10                      /* 加速度计截止频率 */

#define GYRO_FILTER_NUM     2

#define SPEED_SAMP_PERIOD   0.02F                   /* 平衡车速度采样周期 */
#define SPEED_CUTOFF_FREQ   2                       /* 平衡车速度截止频率 */ 

#define GYROZ_SAMP_PERIOD   0.02F                   /* 平衡车转向速度采样周期 */
#define GYROZ_CUTOFF_FREQ   2                       /* 平衡车转向速度截止频率 */

#define BATT_FILTER_NUM     20

void acc_lpf_filter(mpu_result_t *data);
void gyro_aver_filter(mpu_result_t *data);
void gyro_aver_filter(mpu_result_t *data);
void mpu_raw_data_filter(mpu_result_t *data);
float speed_lpf_filter(float x, float y_last);
float gyroz_lpf_filter(float x, float y_last);
float batt_aver_filter(float data_in);


#endif
