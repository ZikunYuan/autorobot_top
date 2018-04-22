#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
extern  float Bias,kp_b,kd_b;
extern float RealSpeed;

int EXTI4_IRQHandler(void);
float speed1(int NowPoint,int setspeed);
float speed2(int NowPoint, int setspeed);
int turn(int encoder_left,int encoder_right,float gyro);
void Set_Pwm1(int moto1);
void Set_Pwm2(int moto1);
void Xianfu_Pwm(void);
void Get_Angle(u8 way);
int myabs(int a);
float location1(int NowPoint,int NextPoint);
float location2(int NowPoint, int NextPoint);
#endif
