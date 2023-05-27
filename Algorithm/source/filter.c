#include "filter.h"

static const float _acc_lpf_coef = ACC_SAMP_PERIOD / (ACC_SAMP_PERIOD + 1 / (2.0 * PI * ACC_CUTOFF_FREQ));          /* =0.11 */

static const float _speed_lpf_coef = SPEED_SAMP_PERIOD / (SPEED_SAMP_PERIOD + 1 / (2.0 * PI * SPEED_CUTOFF_FREQ));  /* =0.20 */
static const float _gyroz_lpf_coef = GYROZ_SAMP_PERIOD / (GYROZ_SAMP_PERIOD + 1 / (2.0 * PI * GYROZ_CUTOFF_FREQ));  /* =0.20 */

/******************************************************************************/
void acc_lpf_filter(mpu_result_t *data)
{
    data->accel_x = data->accel_x + _acc_lpf_coef * (data->accel_xreg - data->accel_x);
    data->accel_y = data->accel_y + _acc_lpf_coef * (data->accel_yreg - data->accel_y);
    data->accel_z = data->accel_z + _acc_lpf_coef * (data->accel_zreg - data->accel_z);
}

/******************************************************************************/
void gyro_aver_filter(mpu_result_t *data)
{
    static int16_t buf_x[GYRO_FILTER_NUM] = {0};
    static int16_t buf_y[GYRO_FILTER_NUM] = {0};
    static int16_t buf_z[GYRO_FILTER_NUM] = {0};
    static uint8_t addr = 0;
    static uint8_t full = 0;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t i;
    
    buf_x[addr] = data->gyro_xreg;
    buf_y[addr] = data->gyro_yreg;
    buf_z[addr] = data->gyro_zreg;
    
    for (i = 0; i < GYRO_FILTER_NUM; i++)
    {
        sum_x += buf_x[i];
        sum_y += buf_y[i];
        sum_z += buf_z[i];
    }
    
    addr++;
    if (addr == GYRO_FILTER_NUM)
    {
        addr = 0;
        full = 1;
    }
    
    if (full == 0)
    {
        data->gyro_x = sum_x / addr;
        data->gyro_y = sum_y / addr;
        data->gyro_z = sum_z / addr;
    }
    else
    {
        data->gyro_x = sum_x / GYRO_FILTER_NUM;
        data->gyro_y = sum_y / GYRO_FILTER_NUM;
        data->gyro_z = sum_z / GYRO_FILTER_NUM;
    }
}

/******************************************************************************/
void mpu_raw_data_filter(mpu_result_t *data)
{
    acc_lpf_filter(data);
    gyro_aver_filter(data);
}

/******************************************************************************/
float speed_lpf_filter(float x, float y_last)
{
    return y_last + _speed_lpf_coef * (x - y_last);
}

/******************************************************************************/
float gyroz_lpf_filter(float x, float y_last)
{
    return y_last + _gyroz_lpf_coef * (x - y_last);
}

/******************************************************************************/
float batt_aver_filter(float data_in)
{
    static float buf[BATT_FILTER_NUM] = {0};
    static uint8_t addr = 0;
    static uint8_t full = 0;
    float sum = 0, data_out;
    uint8_t i;
    
    buf[addr] = data_in;
    
    for (i = 0; i < BATT_FILTER_NUM; i++)
    {
        sum += buf[i];
    }
    
    addr++;
    if (addr == BATT_FILTER_NUM)
    {
        addr = 0;
        full = 1;
    }
    
    if (full == 0)
    {
        data_out = sum / addr;
    }
    else
    {
        data_out = sum / BATT_FILTER_NUM;
    }
    return data_out;
}
