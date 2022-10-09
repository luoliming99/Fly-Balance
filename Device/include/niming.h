#ifndef __NIMING_H
#define __NIMING_H

#include "ch32f20x.h"
#include "mpu6500.h"

/**
 * \brief 上报飞控姿态数据给匿名上位机显示
 * \param[in] p_data    mpu6050测量结果数据
 * \note roll：横滚角，分辨率0.01度，-18000~18000对应-180.00~180.00
 *       pitch：俯仰角，分辨率0.01度，-9000~9000对应-90.00~90.00
 *       yaw：偏航角，分辨率0.1度，-1800~1800对应-180.0~180.0
 */
void niming_report_imu(mpu_result_t *p_data); 

void niming_report_data(mpu_result_t *p_data);

#endif
