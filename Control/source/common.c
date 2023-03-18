#include "common.h"
#include "filter.h"

int task_imu_update(mpu_result_t *mpu_data)
{
    mpu_result_t mpu_raw_data;      /* mpuԭʼ���� */
    
    if (0 == mpu_data)
    {
        return -1;
    }
    
    mpu_read_raw_data(&mpu_raw_data);
    mpu_raw_data_calibration(&mpu_raw_data);
    mpu_raw_data_filter(&mpu_raw_data, mpu_data);
//    mpu_data = mpu_raw_data;
    imu_update(mpu_data);
    
    return 0;
}
