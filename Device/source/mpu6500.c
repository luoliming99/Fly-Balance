#include "mpu6500.h"
#include "ctype.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "led.h"
#include <math.h>

static pfn_i2c_read  i2c_read;
static pfn_i2c_write i2c_write;

static int16_t _mpu_offset[6] = {0};
static const int16_t _accel_val_limit = 1000;
static const int16_t _gyro_val_limit = 100;

static void _mpu_get_offset(void)
{
	mpu_result_t raw_data;
    uint8_t correct_cnt = 0;
    int32_t offset_sum[6] = {0};
    uint8_t i;
    
    do {
        led_set(LED_LF, TOGGLE);
        delay_ms(200);
        mpu_read_raw_data(&raw_data);
        if ((raw_data.accel_xout < -_accel_val_limit || raw_data.accel_xout > _accel_val_limit) ||
            (raw_data.accel_yout < -_accel_val_limit || raw_data.accel_yout > _accel_val_limit) ||
            (raw_data.gyro_xout < -_gyro_val_limit || raw_data.gyro_xout > _gyro_val_limit)     ||
            (raw_data.gyro_yout < -_gyro_val_limit || raw_data.gyro_yout > _gyro_val_limit)     ||
            (raw_data.gyro_zout < -_gyro_val_limit || raw_data.gyro_zout > _gyro_val_limit))
        {
            correct_cnt = 0;
            offset_sum[0] = 0;
            offset_sum[1] = 0;
            offset_sum[2] = 0;
            offset_sum[3] = 0;
            offset_sum[4] = 0;
            offset_sum[5] = 0;
            printf("Error:%d %d %d %d %d %d\r\n",
                    raw_data.accel_xout, raw_data.accel_yout, raw_data.accel_zout,
                    raw_data.gyro_xout, raw_data.gyro_yout, raw_data.gyro_zout);
            continue;
        }
        printf("%d %d %d %d %d %d\r\n",
                raw_data.accel_xout, raw_data.accel_yout, raw_data.accel_zout,
                raw_data.gyro_xout, raw_data.gyro_yout, raw_data.gyro_zout);
        offset_sum[0] += raw_data.accel_xout;
        offset_sum[1] += raw_data.accel_yout;
        offset_sum[2] += raw_data.accel_zout;
        offset_sum[3] += raw_data.gyro_xout;
        offset_sum[4] += raw_data.gyro_yout;
        offset_sum[5] += raw_data.gyro_zout;

        correct_cnt++;
    } while (correct_cnt < 16);
    led_set(LED_LF, OFF);
    
    for (i = 0; i < 6; i++)
    {
        _mpu_offset[i] = offset_sum[i] >> 4;
    }
    _mpu_offset[2] -= 8192; /* z轴加速度计的值应当为重力加速度g */
    printf("mpu offset:%d %d %d %d %d %d\r\n", 
            _mpu_offset[0], _mpu_offset[1], _mpu_offset[2], _mpu_offset[3], _mpu_offset[4], _mpu_offset[5]);
}

/******************************************************************************/
int mpu_init(pfn_i2c_read read, pfn_i2c_write write)
{
    uint8_t regval = 0;
    uint32_t i;
    
    /* 移植I2C通信接口函数 */
    if ((read == 0) || (write == 0)) {
        return -1;
    }
    i2c_read  = read;
    i2c_write = write;
    
    i2c_read(MPU6500_I2C_ADDR, MPU_REG_WHO_AM_I, 1, &regval);
    if (regval != 0x70)
    {
        return -2;
    }
    
    regval = 0x80;  /* Device reset */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_PWR_MGMT_1, 1, &regval);
    
    while ((regval & 0x80) != 0)  /* Wait device reset done */
    {
        for (i = 0; i < 100000; i++)
        {
            ;   /* 适当延时，大概20ms */
        }
        i2c_read(MPU6500_I2C_ADDR, MPU_REG_PWR_MGMT_1, 1, &regval);
    }
    
    regval = 0x03;  /* 退出Sleep模式，配置陀螺仪时钟源为z轴时钟PLL */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_PWR_MGMT_1, 1, &regval);
    
    regval = 0x01;  /* 配置采样频率分频器,div=2,fout=fsamp/div=500Hz */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_SMPLRT_DIV, 1, &regval);
    
    regval = 0x03;  /* 配置陀螺仪低通数字滤波器，bandWidth=41Hz,fsamp=1kHz */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_CONFIG, 1, &regval);
    
    regval = 0x18;  /* 配置角速度计满量程范围为±2000°/S */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_GYRO_CONFIG, 1, &regval);
    
    regval = 0x08;  /* 配置加速度计满量程范围为±4g */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_ACCEL_CONFIG, 1, &regval);
    
    regval = 0x03;  /* 配置加速度计低通数字滤波器，bandWidth=41Hz */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_ACCEL_CONFIG_2, 1, &regval);
    
    _mpu_get_offset();
    
    return 0;
}

/******************************************************************************/
void mpu_read_raw_data(mpu_result_t *p_data)
{
    uint8_t regval[14] = {0};
    
    /*
     * 0x3B~0x40: accel_out
     * 0x41~0x42: temp_out
     * 0x43~0x48: gyro_out
     */
    i2c_read(MPU6500_I2C_ADDR, MPU_REG_ACCEL_XOUT_H, 14, regval);
    
    p_data->accel_xout = ((uint16_t)regval[0] << 8) | regval[1];
    p_data->accel_yout = ((uint16_t)regval[2] << 8) | regval[3];
    p_data->accel_zout = ((uint16_t)regval[4] << 8) | regval[5];
    
    p_data->temp_out = ((uint16_t)regval[6] << 8) | regval[7];
    
    p_data->gyro_xout = ((uint16_t)regval[8] << 8)  | regval[9];
    p_data->gyro_yout = ((uint16_t)regval[10] << 8) | regval[11];
    p_data->gyro_zout = ((uint16_t)regval[12] << 8) | regval[13];
    
    p_data->temp = ((double)p_data->temp_out/340.0)+36.53;
}

/******************************************************************************/
void mpu_raw_data_calibration(mpu_result_t *p_data)
{
    p_data->accel_xout -= _mpu_offset[0];
    p_data->accel_yout -= _mpu_offset[1];
    p_data->accel_zout -= _mpu_offset[2];
    p_data->gyro_xout -= _mpu_offset[3];
    p_data->gyro_yout -= _mpu_offset[4];
    p_data->gyro_zout -= _mpu_offset[5];
}

/****************************** 以下为MPU相关函数 *****************************/

static float _fast_inv_sqrt(float x)
{
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5 - (0.5 * x * y * y));
    return y;
}

static const float _kp = 1.6, _ki = 0.001;
static const float _hlaf_period = MPU_READ_DATA_PERIOD / 2;
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;

/******************************************************************************/
void imu_update(mpu_result_t *p_data)
{
    float norm;
    float ax, ay, az;
    float gx, gy, gz;
    float vx, vy, vz;
    float ex, ey, ez;
    
    static float ex_int = 0, ey_int = 0, ez_int = 0;
    
    /* 加速度计的三维向量转成单位向量 */
    norm = _fast_inv_sqrt((float)p_data->accel_xout * p_data->accel_xout + 
                          (float)p_data->accel_yout * p_data->accel_yout + 
                          (float)p_data->accel_zout * p_data->accel_zout);
    ax = p_data->accel_xout * norm;
    ay = p_data->accel_yout * norm;
    az = p_data->accel_zout * norm;
    
    vx = 2 * (q1 * q3 - q0 * q2);
    vy = 2 * (q0 * q1 + q2 * q3);
    vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    
    ex = ay * vz - az * vy;
	ey = az * vx - ax * vz;
	ez = ax * vy - ay * vx;
    
    ex_int = ex_int + ex * _ki;
    ey_int = ey_int + ey * _ki;
    ez_int = ez_int + ez * _ki;
    
    
    gx = p_data->gyro_xout * GYRO_VAL_TO_RADIAN + _kp * ex + ex_int;
    gy = p_data->gyro_yout * GYRO_VAL_TO_RADIAN + _kp * ey + ey_int;
    gz = p_data->gyro_zout * GYRO_VAL_TO_RADIAN + _kp * ez + ez_int; 
    
    float q0_last = q0;
    float q1_last = q1;
    float q2_last = q2;
    float q3_last = q3;
    
    q0 = q0_last + (-q1_last * gx - q2_last * gy - q3_last * gz) * _hlaf_period;
    q1 = q1_last + ( q0_last * gx - q3_last * gy + q2_last * gz) * _hlaf_period;
    q2 = q2_last + ( q3_last * gx + q0_last * gy - q1_last * gz) * _hlaf_period;
    q3 = q3_last + (-q2_last * gx + q1_last * gy + q0_last * gz) * _hlaf_period;
    
    norm = _fast_inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;
}

/******************************************************************************/
void get_euler_angle(mpu_result_t *p_data)
{
    p_data->pitch = asin(2*(q0*q2 - q1*q3)) * RADIAN_TO_ANGLE;                         /* 俯仰角 */
    p_data->yaw   = atan2(2*(q1*q2 + q0*q3), 1 - 2*(q2*q2 + q3*q3)) * RADIAN_TO_ANGLE; /* 偏航角 */
    p_data->roll  = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * RADIAN_TO_ANGLE; /* 横滚角 */
}
