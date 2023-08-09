#include "niming.h"
#include "bsp_uart.h"

/**
 * \brief 上报数据给匿名上位机
 * \param[in] fun   功能码(0xA0~0xAF)
 * \param[in] p_buf 上报数据缓存
 * \param[in] len   上报数据长度
 * \retval  0       上报成功
 * \retval -1       上报失败
 */
static int __niming_report(uint8_t fun, uint8_t *p_buf, uint8_t len)
{
    uint8_t buf[36];
    uint8_t i;
    
    if (len > 32) return -1;
    
    buf[len+3] = 0; /* 校验和清零 */
    buf[0] = 0x88;  /* 帧头 */
    buf[1] = fun;   /* 功能码 */
    buf[2] = len;   /* 数据长度 */
    /* 装载数据 */
    for (i = 0; i < len; i++)
    {
        buf[3+i] = p_buf[i];
    }
    /* 计算校验和 */
    for (i = 0; i < len+3; i++)
    {
        buf[len+3] += buf[i];
    }
    /* 串口发送数据 */
    uart_send_data_buf(DEBUG_USART, buf, len+4);
    
    return 0;
}

/******************************************************************************/
void niming_report_imu(mpu_result_t *p_data)
{
    uint8_t buf[28] = {0};
    
    buf[0] = (int16_t)p_data->accel_x >> 8;
    buf[1] = (int16_t)p_data->accel_x;
        
    buf[2] = (int16_t)p_data->accel_y >> 8;
    buf[3] = (int16_t)p_data->accel_y;
    
    buf[4] = (int16_t)p_data->accel_z >> 8;
    buf[5] = (int16_t)p_data->accel_z;
    
    buf[6] = (int16_t)p_data->gyro_x >> 8;
    buf[7] = (int16_t)p_data->gyro_x;
    
    buf[8] = (int16_t)p_data->gyro_y >> 8;
    buf[9] = (int16_t)p_data->gyro_y;
    
    buf[10] = (int16_t)p_data->gyro_z >> 8;
    buf[11] = (int16_t)p_data->gyro_z;
    
    /* buf[12]~buf[17]为磁场传感器数据 */
    
    buf[18] = (int16_t)(p_data->roll * 100) >> 8;
    buf[19] = (int16_t)(p_data->roll * 100);
    
    buf[20] = (int16_t)(p_data->pitch * 100) >> 8;
    buf[21] = (int16_t)(p_data->pitch * 100);
    
    buf[22] = (int16_t)(p_data->yaw * 10) >> 8;
    buf[23] = (int16_t)(p_data->yaw * 10);
    
    /* buf[24]~buf[27]应填充0 */
    
    __niming_report(0XAF, buf, 28);    /* 飞控显示帧：0XAF */
}

/******************************************************************************/
void niming_report_data(mpu_result_t *p_data)
{
    uint8_t buf[30] = {0};
    
    buf[0] = p_data->accel_xreg >> 8;
    buf[1] = p_data->accel_xreg;
        
    buf[2] = p_data->accel_yreg >> 8;
    buf[3] = p_data->accel_yreg;
    
    buf[4] = p_data->accel_zreg >> 8;
    buf[5] = p_data->accel_zreg;
    
    buf[6] = p_data->gyro_xreg >> 8;
    buf[7] = p_data->gyro_xreg;
    
    buf[8] = p_data->gyro_yreg >> 8;
    buf[9] = p_data->gyro_yreg;
    
    buf[10] = p_data->gyro_zreg >> 8;
    buf[11] = p_data->gyro_zreg;
    
    buf[12] = (int16_t)p_data->accel_x >> 8;
    buf[13] = (int16_t)p_data->accel_x;
        
    buf[14] = (int16_t)p_data->accel_y >> 8;
    buf[15] = (int16_t)p_data->accel_y;
    
    buf[16] = (int16_t)p_data->accel_z >> 8;
    buf[17] = (int16_t)p_data->accel_z;
    
    buf[18] = (int16_t)p_data->gyro_x >> 8;
    buf[19] = (int16_t)p_data->gyro_x;
    
    buf[20] = (int16_t)p_data->gyro_y >> 8;
    buf[21] = (int16_t)p_data->gyro_y;
    
    buf[22] = (int16_t)p_data->gyro_z >> 8;
    buf[23] = (int16_t)p_data->gyro_z;

    buf[24] = (int16_t)(p_data->roll * 100) >> 8;
    buf[25] = (int16_t)(p_data->roll * 100);
    
    buf[26] = (int16_t)(p_data->pitch * 100) >> 8;
    buf[27] = (int16_t)(p_data->pitch * 100);
    
    buf[28] = (int16_t)(p_data->yaw * 10) >> 8;
    buf[29] = (int16_t)(p_data->yaw * 10);
    
    __niming_report(0XA1, buf, 30);    /* 观察波形：0xA1 */
}
