#include "filter.h"

static const float _iir_acc_coef = IIR_ACC_SAMP_PERIOD / (IIR_ACC_SAMP_PERIOD + 1 / (2.0 * PI * IIR_ACC_CUTOFF_FREQ));
static const float _lpf_speed_coef = 0.5;

/******************************************************************************/
void iir_acc_filter(mpu_result_t *data_in, mpu_result_t *data_out)
{
    data_out->accel_xout = data_out->accel_xout + _iir_acc_coef * (data_in->accel_xout - data_out->accel_xout);
    data_out->accel_yout = data_out->accel_yout + _iir_acc_coef * (data_in->accel_yout - data_out->accel_yout);
    data_out->accel_zout = data_out->accel_zout + _iir_acc_coef * (data_in->accel_zout - data_out->accel_zout);
}

/******************************************************************************/
float lpf_speed_filter(float x, float y_last)
{
    return _lpf_speed_coef * x + (1 - _lpf_speed_coef) * y_last;
}

/******************************************************************************/
void aver_gyro_filter(mpu_result_t *data_in, mpu_result_t *data_out)
{
    static int16_t buf_x[GYRO_FILTER_NUM] = {0};
    static int16_t buf_y[GYRO_FILTER_NUM] = {0};
    static int16_t buf_z[GYRO_FILTER_NUM] = {0};
    static uint8_t addr = 0;
    static uint8_t full = 0;
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t i;
    
    buf_x[addr] = data_in->gyro_xout;
    buf_y[addr] = data_in->gyro_yout;
    buf_z[addr] = data_in->gyro_zout;
    
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
        data_out->gyro_xout = sum_x / addr;
        data_out->gyro_yout = sum_y / addr;
        data_out->gyro_zout = sum_z / addr;
    }
    else
    {
        data_out->gyro_xout = sum_x / GYRO_FILTER_NUM;
        data_out->gyro_yout = sum_y / GYRO_FILTER_NUM;
        data_out->gyro_zout = sum_z / GYRO_FILTER_NUM;
    }
}

/******************************************************************************/
void mpu_raw_data_filter(mpu_result_t *data_in, mpu_result_t *data_out)
{
    iir_acc_filter(data_in, data_out);
    aver_gyro_filter(data_in, data_out);
}

/******************************************************************************/
float aver_batt_filter(float data_in)
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

/******************************************************************************/
float aver_speed_filter(float data_in)
{
    static float buf[SPEED_FILTER_NUM] = {0};
    static uint8_t addr = 0;
    static uint8_t full = 0;
    float sum = 0, data_out;
    uint8_t i;
    
    buf[addr] = data_in;
    
    for (i = 0; i < SPEED_FILTER_NUM; i++)
    {
        sum += buf[i];
    }
    
    addr++;
    if (addr == SPEED_FILTER_NUM)
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
        data_out = sum / SPEED_FILTER_NUM;
    }
    return data_out;
}

/******************************************************************************/
float aver_gyroz_filter(int16_t data_in)
{
    static float buf[GYROZ_FILTER_NUM] = {0};
    static uint8_t addr = 0;
    static uint8_t full = 0;
    float sum = 0, data_out;
    uint8_t i;
    
    buf[addr] = data_in;
    
    for (i = 0; i < GYROZ_FILTER_NUM; i++)
    {
        sum += buf[i];
    }
    
    addr++;
    if (addr == GYROZ_FILTER_NUM)
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
        data_out = sum / GYROZ_FILTER_NUM;
    }
    return data_out;
}
