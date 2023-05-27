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

/* 寄存器宏定义 */
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_ACCEL_CONFIG_2  0x1D
#define MPU_REG_ACCEL_XOUT_H    0x3B
#define MPU_REG_TEMP_OUT_H		0X41
#define MPU_REG_WHO_AM_I        0x75


typedef struct 
{
    int16_t accel_xreg; /* 寄存器中的原始数据 */
    int16_t accel_yreg;
    int16_t accel_zreg;
    
    int16_t gyro_xreg;
    int16_t gyro_yreg;
    int16_t gyro_zreg;
    
    int16_t temp_reg;
    
    float   accel_x;    /* 滤波后的数据 */
    float   accel_y;
    float   accel_z;
    
    float   gyro_x;
    float   gyro_y;
    float   gyro_z;
    
    float   temp;
    float   pitch;      /* 俯仰角 */
    float   yaw;        /* 偏航角 */
    float   roll;       /* 横滚角 */
} mpu_result_t;


int mpu_init(pfn_i2c_read read, pfn_i2c_write write);
void mpu_read_raw_data(mpu_result_t *p_data);
void mpu_raw_data_calibration(mpu_result_t *p_data);

void imu_update(mpu_result_t *p_data);
void get_euler_angle(mpu_result_t *p_data);


#endif

