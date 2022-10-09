#ifndef __MPU6500_H
#define __MPU6500_H

#include "ch32f20x.h"

typedef int (*pfn_i2c_read) (uint8_t slave_addr,
                             uint8_t reg_addr,
                             uint32_t nbytes,
                             uint8_t *p_data);
                                        
typedef int (*pfn_i2c_write) (uint8_t slave_addr,
                              uint8_t reg_addr,
                              uint32_t nbytes,
                              uint8_t *p_data);

#define MPU6500_I2C_ADDR        0x68

/* ¼Ä´æÆ÷ºê¶¨Òå */
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_TEMP_OUT_H		0X41
#define MPU_REG_WHO_AM_I        0x75


typedef struct 
{
    int16_t accel_xout;
    int16_t accel_yout;
    int16_t accel_zout;
    
    int16_t gyro_xout;
    int16_t gyro_yout;
    int16_t gyro_zout;
    
    int16_t temp_out;
    float   temp;
    
    float   pitch;      /* ¸©Ñö½Ç */
    float   yaw;        /* Æ«º½½Ç */
    float   roll;       /* ºá¹ö½Ç */
} mpu_result_t;


int mpu6500_init(pfn_i2c_read read, pfn_i2c_write write);
int mpu6500_read_raw_data(mpu_result_t *p_data);

int mpu_dmp_init(void);
int mpu_dmp_get_data(mpu_result_t *p_data);

#endif

