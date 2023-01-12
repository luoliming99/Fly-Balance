#include "mpu6500.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>

static pfn_i2c_read  i2c_read;
static pfn_i2c_write i2c_write;


/******************************************************************************/
int mpu6500_init(pfn_i2c_read read, pfn_i2c_write write)
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
    
    regval = 0x07;  /* 配置采样频率分频器,div=8,fsamp=fout/div=125Hz */
	i2c_write(MPU6500_I2C_ADDR, MPU_REG_SMPLRT_DIV, 1, &regval);
    
    regval = 0x03;  /* 配置低通数字滤波器，bandWidth=44Hz,fout=1kHz */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_CONFIG, 1, &regval);
    
    regval = 0x18;  /* 配置角速度计满量程范围为±2000°/S */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_GYRO_CONFIG, 1, &regval);
    
    regval = 0x08;  /* 配置加速度计满量程范围为±4g */
    i2c_write(MPU6500_I2C_ADDR, MPU_REG_ACCEL_CONFIG, 1, &regval);
    
    return 0;
}

/******************************************************************************/
int mpu6500_read_raw_data(mpu_result_t *p_data)
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
    
    return 0;
}


/****************************** 以下为MPU相关函数 *****************************/

/* The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * TODO: The following matrices refer to the configuration on internal test
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static signed char __gyro_orientation[9] = {-1, 0, 0,
                                            0, -1, 0,
                                            0, 0, 1};

static unsigned short __inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;		// error
    return b;
}

/** Converts an orientation matrix made up of 0,+1,and -1 to a scalar representation.
* @param[in] mtx Orientation matrix to convert to a scalar.
* @return Description of orientation matrix. The lowest 2 bits (0 and 1) represent the column the one is on for the
* first row, with the bit number 2 being the sign. The next 2 bits (3 and 4) represent
* the column the one is on for the second row with bit number 5 being the sign.
* The next 2 bits (6 and 7) represent the column the one is on for the third row with
* bit number 8 being the sign. In binary the identity matrix would therefor be:
* 010_001_000 or 0x88 in hex.
*/
static unsigned short __inv_orientation_matrix_to_scalar(const signed char *mtx)
{

    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = __inv_row_2_scale(mtx);
    scalar |= __inv_row_2_scale(mtx + 3) << 3;
    scalar |= __inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static int __mpu_run_self_test(void)
{
	int ret;
	long gyro[3], accel[3]; 
    
	ret = mpu_run_6500_self_test(gyro, accel, 0);

    if (ret == 0x7) 
	{
		/* 
         * Test passed. We can trust the gyro data here, so let's push it down
		 * to the DMP.
		 */
		unsigned short accel_sens;
    	float gyro_sens;
        
		mpu_get_gyro_sens(&gyro_sens);
		gyro[0] = (long)(gyro[0] * gyro_sens);
		gyro[1] = (long)(gyro[1] * gyro_sens);
		gyro[2] = (long)(gyro[2] * gyro_sens);
		dmp_set_gyro_bias(gyro);
        
		mpu_get_accel_sens(&accel_sens);
		accel[0] *= accel_sens;
		accel[1] *= accel_sens;
		accel[2] *= accel_sens;
		dmp_set_accel_bias(accel);
        
		return 0;
	}
    return 1;
}

/******************************************************************************/
int mpu_dmp_init(void)
{
    int ret = 0;
    
    ret = mpu_init();       /* 设置量程、带宽、采样率 */
    if (ret) return -1;
    
    ret = mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);    /* 设置需用用到的传感器 */
    if (ret) return -2;
    
    ret = mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL); /* 设置FIFO */
    if (ret) return -3;
    
    ret = dmp_load_motion_driver_firmware();		        /* 加载DMP固件 */
    if (ret) return -4; 
    
    ret = dmp_set_orientation(
            __inv_orientation_matrix_to_scalar(__gyro_orientation));    /* 设置陀螺仪方向 */
    if (ret) return -5;
    
    ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
            DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
            DMP_FEATURE_GYRO_CAL);  /* 设置DMP功能 */
    if (ret) return -6; 
    
    ret = dmp_set_fifo_rate(100);	/* 设置DMP输出速率(最大不超过200Hz) */
    if (ret) return -7; 

    ret = __mpu_run_self_test();    /* 自检 */
    if (ret) return -8;
    
    ret = mpu_set_dmp_state(1);	    /* 使能DMP */
    if (ret) return -9;

    return 0;
}

#define q30 1073741824.0f   /* q30格式,long转float时的除数 */

/******************************************************************************/
int mpu_dmp_get_data(mpu_result_t *p_data)
{
    int ret = 0;
    short gyro[3], accel[3], sensors;
    long quat[4];
    unsigned long timestamp;
    unsigned char more;
    float qw, qx, qy, qz;       /* 四元数 */ 

    ret = dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more);
    if (ret) return -1;

    if (sensors & INV_WXYZ_QUAT)
    {
        qw = quat[0] / q30;
		qx = quat[1] / q30;
		qy = quat[2] / q30;
		qz = quat[3] / q30;
        
        p_data->pitch = asin(2*qw*qy - 2*qx*qz) * 57.3;                                 /* 俯仰角 */
        p_data->yaw   = atan2(2*(qx*qy + qw*qz), 1 - 2*(qy*qy + qz*qz)) * 57.3;         /* 偏航角 */
        p_data->roll  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy)) * 57.3;         /* 横滚角 */   
    }
    
    if (sensors & INV_XYZ_ACCEL)
    {
        p_data->accel_xout = accel[0];
        p_data->accel_yout = accel[1];
        p_data->accel_zout = accel[2];
    }
    
    if (sensors & INV_XYZ_GYRO)
    {
        p_data->gyro_xout = gyro[0];
        p_data->gyro_yout = gyro[1];
        p_data->gyro_zout = gyro[2];
    }
    
    return 0;
}



