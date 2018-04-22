#ifndef __HMC5883L_H
#define __HMC5883L_H
#include"sys.h"
//#include "ioi2c.h"
#include "Sensor_Param.h"
#define SlaveAddress 0x3c
#define CALIBRATING_MAG_CYCLES              2000  //校准时间持续20s
extern u8 Mag_CALIBRATED;
extern float Pitch,Roll, Yaw_Mpu, Yaw_Com;


void Init_HMC5883(void);
void Write_HMC5883(u8 add, u8 da);
u8 Read_HMC5883(u8 REG_Address);
void Multiple_read_HMC5883(void);

#endif
