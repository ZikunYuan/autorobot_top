#include "show.h"
#include "control.h"
#include "oled.h"
#include "mpu6050.h"
#include "imu.h"
#include "encoder.h"

unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
float Vol;
extern u8 Way_Angle, Flag_Qian, Flag_Hou, Flag_Left, Flag_Right, Flag_sudu, Flag_Stop, Flag_Show,Bi_zhang;
extern int Temperature,Encoder_Left,Encoder_Right,Voltage;
extern u32 Distance;
extern float Angle_Balance,Gyro_Balance,Gyro_Turn,ki_v,kp_v;
extern float Pitch, Roll, Yaw_Mpu, Yaw_Com;
extern float CH1, CH4;
extern u16 Step1_Num, Step2_Num;
extern u16 pwm;
extern int turn_dir;
extern s16 pwm_num;
extern int Moto1;
extern float EncoderVelocity;
extern float Encoder_speed;
extern float Error_speed;
extern float pwm_speed;
extern int Location1Num, Location2Num;
/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{
	char s[10];

	OLED_ShowString(00,10,"Enco2��");
  sprintf(s, "%5d", Encoder2);
	OLED_ShowString(65, 10, s);
		

  OLED_ShowString(00,30,"Enco1��");
	sprintf(s, "%5d", Encoder1);
	OLED_ShowString(40, 30, s); 

	OLED_ShowString(00, 50, "LocaN��");
	sprintf(s, "%5d", Location1Num);
	OLED_ShowString(40, 50, s);

  //=============ˢ��=======================//
  OLED_Refresh_Gram();
} 




void APP_Show(void)
{

}

