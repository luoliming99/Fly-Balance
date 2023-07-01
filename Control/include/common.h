#ifndef __COMMON_H
#define __COMMON_H

#include "ch32f20x.h"
#include "mpu6500.h"


#define FLY                     1
#define CAR                     2
#define PRODUCT                 FLY             /* ��Ʒ���壺FLY - �����������CAR - ����ƽ�⳵ */

#define PI                      3.1415927F
#define RADIAN_TO_ANGLE         57.2957795F     /* 180/PI */
#define GYRO_VAL_TO_RADIAN      0.0010653F      /* 4000/65535/180*PI */

#define MPU_READ_DATA_PERIOD    0.002F          /* ��ȡMPU������ԭʼ��������Ϊ2ms */


typedef enum
{
    NO_KEY_PRESS = 0,
    KEY_L_PRESS,
    KEY_R_PRESS,
} key_status_e;

typedef enum
{
    UNLOCK_INIT = 0,
    UNLOCK_SUCCESS
} unlock_status_e;


int task_imu_update(mpu_result_t *mpu_data);

#endif
