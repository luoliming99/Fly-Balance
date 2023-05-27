#include "common.h"
#include "filter.h"

int task_imu_update(mpu_result_t *mpu_data)
{
    if (0 == mpu_data)
    {
        return -1;
    }
    
    mpu_read_raw_data(mpu_data);
    mpu_raw_data_calibration(mpu_data);
    mpu_raw_data_filter(mpu_data);
    imu_update(mpu_data);
    
    return 0;
}
