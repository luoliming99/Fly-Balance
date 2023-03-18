#ifndef __COMMON_H
#define __COMMON_H

#include "ch32f20x.h"
#include "mpu6500.h"

#define FLY                                     /* ��Ʒ���壺FLY - �����������BALANCE - ����ƽ�⳵ */

#define PI                      3.1415927
#define RADIAN_TO_ANGLE	        57.2957795      /* 180/PI */
#define GYRO_VAL_TO_RADIAN	    0.0010653       /* 4000/65535/180*PI */

#define MPU_READ_DATA_PERIOD    0.002           /* ��ȡMPU������ԭʼ��������Ϊ2ms */

#ifdef FLY
    #define BATT_VOLT_LIMIT     2.8             /* ��ص�ѹ������� */
#else
    #define BATT_VOLT_LIMIT     3.2
#endif

typedef enum
{
    NO_KEY_PRESS = 0,
    KEY_L_PRESS,
    KEY_R_PRESS,
} key_status_e;

typedef enum
{
    UNLOCK_INIT = 0,
    UNLOCK_MAX_ACC,
    UNLOCK_SUCCESS
} unlock_status_e;


int task_imu_update(mpu_result_t *mpu_data);

#endif
