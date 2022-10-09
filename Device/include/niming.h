#ifndef __NIMING_H
#define __NIMING_H

#include "ch32f20x.h"
#include "mpu6500.h"

/**
 * \brief �ϱ��ɿ���̬���ݸ�������λ����ʾ
 * \param[in] p_data    mpu6050�����������
 * \note roll������ǣ��ֱ���0.01�ȣ�-18000~18000��Ӧ-180.00~180.00
 *       pitch�������ǣ��ֱ���0.01�ȣ�-9000~9000��Ӧ-90.00~90.00
 *       yaw��ƫ���ǣ��ֱ���0.1�ȣ�-1800~1800��Ӧ-180.0~180.0
 */
void niming_report_imu(mpu_result_t *p_data); 

void niming_report_data(mpu_result_t *p_data);

#endif
