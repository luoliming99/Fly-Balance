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
    uint8_t buf[32];
    uint8_t i;
    
    if (len > 28) return -1;
    
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
    
    buf[0] = p_data->accel_xout >> 8;
    buf[1] = p_data->accel_xout;
        
    buf[2] = p_data->accel_yout >> 8;
    buf[3] = p_data->accel_yout;
    
    buf[4] = p_data->accel_zout >> 8;
    buf[5] = p_data->accel_zout;
    
    buf[6] = p_data->gyro_xout >> 8;
    buf[7] = p_data->gyro_xout;
    
    buf[8] = p_data->gyro_yout >> 8;
    buf[9] = p_data->gyro_yout;
    
    buf[10] = p_data->gyro_zout >> 8;
    buf[11] = p_data->gyro_zout;
    
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
	uint8_t buf[18] = {0};
    
    buf[0] = p_data->accel_xout >> 8;
    buf[1] = p_data->accel_xout;
        
    buf[2] = p_data->accel_yout >> 8;
    buf[3] = p_data->accel_yout;
    
    buf[4] = p_data->accel_zout >> 8;
    buf[5] = p_data->accel_zout;
    
    buf[6] = p_data->gyro_xout >> 8;
    buf[7] = p_data->gyro_xout;
    
    buf[8] = p_data->gyro_yout >> 8;
    buf[9] = p_data->gyro_yout;
    
    buf[10] = p_data->gyro_zout >> 8;
    buf[11] = p_data->gyro_zout;

    buf[12] = (int16_t)(p_data->roll * 100) >> 8;
    buf[13] = (int16_t)(p_data->roll * 100);
    
    buf[14] = (int16_t)(p_data->pitch * 100) >> 8;
    buf[15] = (int16_t)(p_data->pitch * 100);
    
    buf[16] = (int16_t)(p_data->yaw * 10) >> 8;
    buf[17] = (int16_t)(p_data->yaw * 10);
    
	__niming_report(0XA1, buf, 18);    /* 观察波形：0xA1 */
}
