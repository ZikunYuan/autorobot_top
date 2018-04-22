#ifndef _IMU_H_
#define	_IMU_H_

#include "Sensor_Param.h"		
#include "stm32f4xx.h"
#define TO_M_S2 				0.23926f   	//   980cm/s2    +-8g   980/4096
typedef struct 
{
	xyz_f_t err;
	xyz_f_t err_tmp;
	xyz_f_t err_lpf;
	xyz_f_t err_Int;
	xyz_f_t g;
	
}ref_t;


extern float Roll_ANO, Pitch_ANO, Yaw_ANO;
extern xyz_f_t reference_v;
void IMUupdate(float half_T,float gx, float gy, float gz, float ax, float ay, float az,float *rol,float *pit,float *yaw);
extern float Roll,Pitch,Yaw;


#endif

