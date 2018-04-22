#include "mpu6050.h"
//#include "IOI2C.h"
#include "usart.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "Sensor_Param.h"
#include "myiic.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
#define PRINT_ACCEL     (0x01)
#define PRINT_GYRO      (0x02)
#define PRINT_QUAT      (0x04)
#define ACCEL_ON        (0x01)
#define GYRO_ON         (0x02)
#define MOTION          (0)
#define NO_MOTION       (1)
#define DEFAULT_MPU_HZ  (200)
#define FLASH_SIZE      (512)
#define FLASH_MEM_START ((void*)0x1800)
#define q30  1073741824.0f
short gyro[3], accel[3], sensors;
float Pitch,Roll,Yaw_Mpu,Yaw_Com;
MPU6050_STRUCT mpu6050;
u8 mpu6050_buffer[14];
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
static signed char gyro_orientation[9] = {-1, 0, 0,
                                           0,-1, 0,
                                           0, 0, 1};

static  unsigned short inv_row_2_scale(const signed char *row)
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
        b = 7;      // error
    return b;
}


static  unsigned short inv_orientation_matrix_to_scalar(
    const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);
    if (result == 0x7) {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;
        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);
        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
		printf("setting bias succesfully ......\r\n");
    }
}



uint8_t buffer[14];

int16_t  MPU6050_FIFO[6][11];
int16_t Gx_offset=0,Gy_offset=0,Gz_offset=0;


/**************************实现函数********************************************
*函数原型:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*功　　能:	    将新的ADC数据更新到 FIFO数组，进行滤波处理
*******************************************************************************/

void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){	//FIFO 操作
MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[0][9]=ax;//将新的数据放置到 数据的最后面
MPU6050_FIFO[1][9]=ay;
MPU6050_FIFO[2][9]=az;
MPU6050_FIFO[3][9]=gx;
MPU6050_FIFO[4][9]=gy;
MPU6050_FIFO[5][9]=gz;

sum=0;
for(i=0;i<10;i++){	//求当前数组的合，再取平均值
   sum+=MPU6050_FIFO[0][i];
}
MPU6050_FIFO[0][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[1][i];
}
MPU6050_FIFO[1][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[2][i];
}
MPU6050_FIFO[2][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[3][i];
}
MPU6050_FIFO[3][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[4][i];
}
MPU6050_FIFO[4][10]=sum/10;

sum=0;
for(i=0;i<10;i++){
   sum+=MPU6050_FIFO[5][i];
}
MPU6050_FIFO[5][10]=sum/10;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setClockSource(uint8_t source)
*功　　能:	    设置  MPU6050 的时钟源
 * CLK_SEL | Clock Source
 * --------+--------------------------------------
 * 0       | Internal oscillator
 * 1       | PLL with X Gyro reference
 * 2       | PLL with Y Gyro reference
 * 3       | PLL with Z Gyro reference
 * 4       | PLL with external 32.768kHz reference
 * 5       | PLL with external 19.2MHz reference
 * 6       | Reserved
 * 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(uint8_t source){
    IICwriteBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);

}

/** Set full-scale gyroscope range.
 * @param range New full-scale gyroscope range value
 * @see getFullScaleRange()
 * @see MPU6050_GYRO_FS_250
 * @see MPU6050_RA_GYRO_CONFIG
 * @see MPU6050_GCONFIG_FS_SEL_BIT
 * @see MPU6050_GCONFIG_FS_SEL_LENGTH
 */
void MPU6050_setFullScaleGyroRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setFullScaleAccelRange(uint8_t range)
*功　　能:	    设置  MPU6050 加速度计的最大量程
*******************************************************************************/
void MPU6050_setFullScaleAccelRange(uint8_t range) {
    IICwriteBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setSleepEnabled(uint8_t enabled)
*功　　能:	    设置  MPU6050 是否进入睡眠模式
				enabled =1   睡觉
			    enabled =0   工作
*******************************************************************************/
void MPU6050_setSleepEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_getDeviceID(void)
*功　　能:	    读取  MPU6050 WHO_AM_I 标识	 将返回 0x68
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void) {

    IICreadBytes(devAddr, MPU6050_RA_WHO_AM_I, 1, buffer);
    return buffer[0];
}

/**************************实现函数********************************************
*函数原型:		uint8_t MPU6050_testConnection(void)
*功　　能:	    检测MPU6050 是否已经连接
*******************************************************************************/
uint8_t MPU6050_testConnection(void) {
   if(MPU6050_getDeviceID() == 0x68)  //0b01101000;
   return 1;
   	else return 0;
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CMasterModeEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CMasterModeEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_I2C_MST_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:		void MPU6050_setI2CBypassEnabled(uint8_t enabled)
*功　　能:	    设置 MPU6050 是否为AUX I2C线的主机
*******************************************************************************/
void MPU6050_setI2CBypassEnabled(uint8_t enabled) {
    IICwriteBit(devAddr, MPU6050_RA_INT_PIN_CFG, MPU6050_INTCFG_I2C_BYPASS_EN_BIT, enabled);
}

/**************************实现函数********************************************
*函数原型:
*功　　能:	    设置 采样率
*******************************************************************************/
void MPU6050_set_SMPLRT_DIV(uint16_t hz)
{
	IICwriteByte(0x68, MPU6050_RA_SMPLRT_DIV, 1000 / hz - 1);
	//I2C_Single_Write(MPU6050_ADDRESS,MPU_RA_SMPLRT_DIV, (1000/sample_rate - 1));
}

/**************************实现函数********************************************
*函数原型:
*功　　能:	    设置低通滤波截止频率
*******************************************************************************/
void MPU6050_setDLPF(uint8_t mode)
{
	IICwriteBits(0x68, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, mode);
}

__INLINE void MPU6050_Read(void)
{
	//I2C_FastMode = 1;
	i2cRead(0x68, MPU6050_RA_ACCEL_XOUT_H, 14, mpu6050_buffer);
}
/**************************************************************************
函数功能：MPU6050内置DMP的初始化
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void DMP_Init(void)//每一句最好都加一句delay，保险
{ 
   u8 temp[1]={0};
   i2cRead(0x68,0x75,1,temp);
	 
	 printf("mpu_set_sensor complete ......\r\n");
	if(temp[0]!=0x68)NVIC_SystemReset();
	 	  delay_ms(100);
	if(!mpu_init())
  {
		delay_ms(100);
	  if(!mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS))
		  //开启加速度计、陀螺仪、磁力计，使用电子罗盘，要加入INV_XYZ_COMPASS
		  printf("mpu_set_sensor complete ......\r\n");
	  delay_ms(100);
	  if(!mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL))
	  	 printf("mpu_configure_fifo complete ......\r\n");
	  delay_ms(100);
	  if(!mpu_set_sample_rate(DEFAULT_MPU_HZ))
	  	 printf("mpu_set_sample_rate complete ......\r\n");
	  delay_ms(100);
	  if(!dmp_load_motion_driver_firmware())
	  	printf("dmp_load_motion_driver_firmware complete ......\r\n");
	  delay_ms(100);
	  if(!dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation)))
	  	 printf("dmp_set_orientation complete ......\r\n");
	  delay_ms(100);
	  if(!dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_TAP |
	        DMP_FEATURE_ANDROID_ORIENT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO |
	        DMP_FEATURE_GYRO_CAL))
	  	 printf("dmp_enable_feature complete ......\r\n");
	  delay_ms(100);
	  if(!dmp_set_fifo_rate(DEFAULT_MPU_HZ))
	  	 printf("dmp_set_fifo_rate complete ......\r\n");
	  delay_ms(100);
	  run_self_test();
		  delay_ms(100);	
	  if(!mpu_set_dmp_state(1))
	  	 printf("mpu_set_dmp_state complete ......\r\n");
	}
	
}
/**************************************************************************
函数功能：读取MPU6050内置DMP的姿态信息
入口参数：无
返回  值：无
作    者：平衡小车之家
**************************************************************************/
void Read_DMP(void)
{	
	  unsigned long sensor_timestamp;
		unsigned char more;
		long quat[4];
				dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors, &more);		
				if (sensors & INV_WXYZ_QUAT )
				{    
					 q0=quat[0] / q30;
					 q1=quat[1] / q30;
					 q2=quat[2] / q30;
					 q3=quat[3] / q30;
					 Pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	
					 Roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; // roll
					 Yaw_Mpu  = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw
				}

}
/**************************************************************************
函数功能：读取MPU6050内置温度传感器数据
入口参数：无
返回  值：摄氏温度
作    者：平衡小车之家
**************************************************************************/
int Read_Temperature(void)
{	   
	  float Temp;
	  Temp=(I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_TEMP_OUT_L);
		if(Temp>32768) Temp-=65536;
		Temp=(36.53+Temp/340)*10;
	  return (int)Temp;
}


/**************************实现函数********************************************
*函数原型:		void MPU6050_initialize(void)
*功　　能:	    初始化 	MPU6050 以进入可用状态。
*******************************************************************************/
void MPU6050_initialize(void) {
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_YGYRO); //设置时钟
	//MPU6050_set_SMPLRT_DIV(1000);  //1000hz
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_2000);//陀螺仪最大量程 +-1000度每秒
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);	//加速度度最大量程 +-2G
	MPU6050_setSleepEnabled(0); //进入工作状态
	//MPU6050_setDLPF(MPU6050_DLPF_BW_42);  //42hz
	MPU6050_setI2CMasterModeEnabled(0);	 //不让MPU6050 控制AUXI2C
	MPU6050_setI2CBypassEnabled(1);	 //主控制器的I2C与	MPU6050的AUXI2C	直通。控制器可以直接访问HMC5883L
}



/**************************************************************************
函数功能：读取MPU6050传感器数据
入口参数：
作    者：匿名读数
**************************************************************************/
s32 sum_temp[7] = { 0,0,0,0,0,0,0 };
u16 acc_sum_cnt = 0, gyro_sum_cnt = 0;

void MPU6050_Data_Offset()
{
#ifdef ACC_ADJ_EN

	if (mpu6050.Acc_CALIBRATE == 1)
	{
		acc_sum_cnt++;
		sum_temp[A_X] += mpu6050.Acc_I16.x;
		sum_temp[A_Y] += mpu6050.Acc_I16.y;
		//sum_temp[A_Z] += mpu6050.Acc_I16.z - 65536 / 16;   // +-8G
		sum_temp[A_Z] += mpu6050.Acc_I16.z - 65536 / 4;   // +-2G
		sum_temp[TEM] += mpu6050.Tempreature;

		if (acc_sum_cnt >= OFFSET_AV_NUM)
		{
			mpu6050.Acc_Offset.x = sum_temp[A_X] / OFFSET_AV_NUM;
			mpu6050.Acc_Offset.y = sum_temp[A_Y] / OFFSET_AV_NUM;
			mpu6050.Acc_Offset.z = sum_temp[A_Z] / OFFSET_AV_NUM;
			mpu6050.Temprea_Offset = sum_temp[TEM] / OFFSET_AV_NUM;
			acc_sum_cnt = 0;
			mpu6050.Acc_CALIBRATE = 0;
			Param_SaveAccelOffset(&mpu6050.Acc_Offset);
			sum_temp[A_X] = sum_temp[A_Y] = sum_temp[A_Z] = sum_temp[TEM] = 0;
		}
	}

#endif

	if (mpu6050.Gyro_CALIBRATE)
	{
		gyro_sum_cnt++;
		sum_temp[G_X] += mpu6050.Gyro_I16.x;
		sum_temp[G_Y] += mpu6050.Gyro_I16.y;
		sum_temp[G_Z] += mpu6050.Gyro_I16.z;
		sum_temp[TEM] += mpu6050.Tempreature;

		if (gyro_sum_cnt >= OFFSET_AV_NUM)
		{
			mpu6050.Gyro_Offset.x = (float)sum_temp[G_X] / OFFSET_AV_NUM;
			mpu6050.Gyro_Offset.y = (float)sum_temp[G_Y] / OFFSET_AV_NUM;
			mpu6050.Gyro_Offset.z = (float)sum_temp[G_Z] / OFFSET_AV_NUM;
			mpu6050.Temprea_Offset = sum_temp[TEM] / OFFSET_AV_NUM;
			gyro_sum_cnt = 0;
			if (mpu6050.Gyro_CALIBRATE == 1)
				Param_SaveGyroOffset(&mpu6050.Gyro_Offset);
			mpu6050.Gyro_CALIBRATE = 0;
			sum_temp[G_X] = sum_temp[G_Y] = sum_temp[G_Z] = sum_temp[TEM] = 0;
		}
	}
}

void Transform(float itx, float ity, float itz, float *it_x, float *it_y, float *it_z)
{
	*it_x = itx;
	*it_y = ity;
	*it_z = itz;

}

s16 FILT_BUF[ITEMS][(FILTER_NUM + 1)];
uint8_t filter_cnt = 0, filter_cnt_old = 0;

float mpu6050_tmp[ITEMS];
float tmp[ITEMS];
float test_ang = 0, test_ang_old = 0, test_ang_d, test_fli_a, test_i;
//u8 mpu6050_buffer[14];

//__INLINE void MPU6050_Rea         下 d(void)
//{
//	//I2C_FastMode = 1;
//	IIC_Read_nByte(0x68, MPU6050_RA_ACCEL_XOUT_H, 14, mpu6050_buffer);
//}



void MPU6050_Data_Prepare(float T)
{
	u8 i;
	s32 FILT_TMP[ITEMS] = { 0,0,0,0,0,0,0 };
	//	float auto_offset_temp[3];
	float Gyro_tmp[3];

	MPU6050_Data_Offset(); //校准函数

						   /*读取buffer原始数据*/
	mpu6050.Acc_I16.x = ((((int16_t)mpu6050_buffer[0]) << 8) | mpu6050_buffer[1]);
	mpu6050.Acc_I16.y = ((((int16_t)mpu6050_buffer[2]) << 8) | mpu6050_buffer[3]);
	mpu6050.Acc_I16.z = ((((int16_t)mpu6050_buffer[4]) << 8) | mpu6050_buffer[5]);

	mpu6050.Gyro_I16.x = ((((int16_t)mpu6050_buffer[8]) << 8) | mpu6050_buffer[9]);
	mpu6050.Gyro_I16.y = ((((int16_t)mpu6050_buffer[10]) << 8) | mpu6050_buffer[11]);
	mpu6050.Gyro_I16.z = ((((int16_t)mpu6050_buffer[12]) << 8) | mpu6050_buffer[13]);

	Gyro_tmp[0] = mpu6050.Gyro_I16.x;//
	Gyro_tmp[1] = mpu6050.Gyro_I16.y;//
	Gyro_tmp[2] = mpu6050.Gyro_I16.z;//

	mpu6050.Tempreature = ((((int16_t)mpu6050_buffer[6]) << 8) | mpu6050_buffer[7]); //tempreature
	mpu6050.TEM_LPF += 2 * 3.14f *T *(mpu6050.Tempreature - mpu6050.TEM_LPF);
	mpu6050.Ftempreature = mpu6050.TEM_LPF / 340.0f + 36.5f;

	//======================================================================
	if (++filter_cnt > FILTER_NUM)
	{
		filter_cnt = 0;

	}
	else
	{
		filter_cnt_old = (filter_cnt == FILTER_NUM) ? 0 : (filter_cnt + 1);
	}
	//10 170 4056
	/* 得出校准后的数据 */
	mpu6050_tmp[A_X] = (mpu6050.Acc_I16.x - mpu6050.Acc_Offset.x);
	mpu6050_tmp[A_Y] = (mpu6050.Acc_I16.y - mpu6050.Acc_Offset.y);
	mpu6050_tmp[A_Z] = (mpu6050.Acc_I16.z - mpu6050.Acc_Offset.z);
	mpu6050_tmp[G_X] = Gyro_tmp[0] - mpu6050.Gyro_Offset.x;//
	mpu6050_tmp[G_Y] = Gyro_tmp[1] - mpu6050.Gyro_Offset.y;//
	mpu6050_tmp[G_Z] = Gyro_tmp[2] - mpu6050.Gyro_Offset.z;//


														   /* 更新滤波滑动窗口数组 */
	FILT_BUF[A_X][filter_cnt] = mpu6050_tmp[A_X];
	FILT_BUF[A_Y][filter_cnt] = mpu6050_tmp[A_Y];
	FILT_BUF[A_Z][filter_cnt] = mpu6050_tmp[A_Z];
	FILT_BUF[G_X][filter_cnt] = mpu6050_tmp[G_X];
	FILT_BUF[G_Y][filter_cnt] = mpu6050_tmp[G_Y];
	FILT_BUF[G_Z][filter_cnt] = mpu6050_tmp[G_Z];

	for (i = 0; i<FILTER_NUM; i++)
	{
		FILT_TMP[A_X] += FILT_BUF[A_X][i];
		FILT_TMP[A_Y] += FILT_BUF[A_Y][i];
		FILT_TMP[A_Z] += FILT_BUF[A_Z][i];
		FILT_TMP[G_X] += FILT_BUF[G_X][i];
		FILT_TMP[G_Y] += FILT_BUF[G_Y][i];
		FILT_TMP[G_Z] += FILT_BUF[G_Z][i];
	}


	tmp[A_X] = (float)(FILT_TMP[A_X]) / (float)FILTER_NUM;
	tmp[A_Y] = (float)(FILT_TMP[A_Y]) / (float)FILTER_NUM;
	tmp[A_Z] = (float)(FILT_TMP[A_Z]) / (float)FILTER_NUM;


	tmp[G_X] = (float)(FILT_TMP[G_X]) / (float)FILTER_NUM;
	tmp[G_Y] = (float)(FILT_TMP[G_Y]) / (float)FILTER_NUM;
	tmp[G_Z] = (float)(FILT_TMP[G_Z]) / (float)FILTER_NUM;


	/*坐标转换*/
	Transform(tmp[A_X], tmp[A_Y], tmp[A_Z], &mpu6050.Acc.x, &mpu6050.Acc.y, &mpu6050.Acc.z);
	Transform(tmp[G_X], tmp[G_Y], tmp[G_Z], &mpu6050.Gyro.x, &mpu6050.Gyro.y, &mpu6050.Gyro.z);

	mpu6050.Gyro_deg.x = mpu6050.Gyro.x *TO_ANGLE;
	mpu6050.Gyro_deg.y = mpu6050.Gyro.y *TO_ANGLE;
	mpu6050.Gyro_deg.z = mpu6050.Gyro.z *TO_ANGLE;



	//======================================================================
}


/**********************************************
* 陀螺仪矫正
**********************************************/
void MPU6050_GYRO_Calibrate(void)
{
	mpu6050.Gyro_CALIBRATE = 1;
}

/**********************************************
* 加速度计矫正
**********************************************/
void MPU6050_ACC_Calibrate(void)
{
	mpu6050.Acc_CALIBRATE = 1;
}


//------------------End of File----------------------------
