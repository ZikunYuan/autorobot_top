#ifndef __FILTER_H
#define __FILTER_H
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
extern float angle_x,angle_y,angle_z, angle_dot; 	
void Kalman_Filter_x(float Accel,float Gyro);
void Kalman_Filter_y(float Accel, float Gyro);
void Yijielvbo_X(float angle_m, float gyro_m);
void Yijielvbo_Y(float angle_m, float gyro_m);
#endif
