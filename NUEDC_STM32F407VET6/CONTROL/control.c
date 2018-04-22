#include "control.h"
#include "filter.h"
#include "exti.h"
#include "motor.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "inv_mpu.h"
#include "dmpmap.h"
#include "MPU6050.h"
#include "encoder.h"
#include "math.h"
#include "ANO_DT.h"
#include "key.h"
#include "delay.h"
#include "show.h"
#include "usart.h"
#include "Relay.h"

//���õ��ı��� ������û�õ��ı���
int Moto1,Moto2;
int Now1_Encoder=0,Last1_Encoder=0;//�����ٶȻ�ʱ��¼ǰ�����������
extern u8 Speed2Flag, Location2Flag;
extern int Location1Num, Location2Num, Speed1Num;
extern int Encoder1, Encoder2;
	
void Delay(__IO uint32_t nCount)
{
	for (; nCount != 0; nCount--);
}


/**************************************************************************
�������ܣ��жϿ���RM35���
��ڲ�����
����  ֵ����
**************************************************************************/
void TIM6_DAC_IRQHandler(void)//10ms�ж�
{
	int pwmb = 0;
	
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
	    if(Location2Flag)
			{
				Encoder2=TIM2->CNT;
			  pwmb = location2(Encoder2,Location2Num);
	      Moto2 = pwmb;
	      Xianfu_Pwm();    //PWM����޷� pwm1�޷�4000��pwm2�޷�3000
	      Set_Pwm2(-Moto2);//���Ƶ������ת
			}
			if(Location2Flag==0)
			{
				Set_Pwm1(0);
				Set_Pwm2(0);
			}
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update);  //����жϱ�־λ
  }
}



/**************************************************************************
�������ܣ�λ�û� צ�Ӳ��ִ󽮵��
��ڲ�����
����  ֵ����
**************************************************************************/
float LastError1_L, PrevError1_L = 0;   //������Ҫ����
float Error1_L, SumError1_L, dError1_L = 0;
float kp1_L = 1.5, ki1_L = 1, kd1_L = 4;

float location1( int NowPoint,int NextPoint)//Nowpoint��ǰλ�� NextPointΪ�趨ֵ
{    
	float pwm;
	Error1_L = NextPoint - NowPoint;        // ƫ��
	PrevError1_L = Error1_L;
	SumError1_L += Error1_L;                    // ����
	dError1_L= -LastError1_L + PrevError1_L;     // ��ǰ΢��
	LastError1_L = PrevError1_L;
	if (SumError1_L>1560) SumError1_L = 0;
	if (SumError1_L<-1560) SumError1_L = 0;
	pwm = kp1_L * Error1_L + ki1_L *  SumError1_L + kd1_L * dError1_L;	
	if(NowPoint<(NextPoint+5)&&NowPoint>(NextPoint-5))
		pwm=0;
	return pwm;
}



/**************************************************************************
�������ܣ�λ�û� ��ת���ִ󽮵��
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
float LastError2_L, PrevError2_L = 0;  
float Error2_L, SumError2_L, dError2_L = 0;
float kp2_L = 2, ki2_L = 0, kd2_L = 0.5;

float location2(int NowPoint, int NextPoint)//NowpointΪ��ǰλ�� NextPointΪ����λ��
{    
	float pwm;
	Error2_L = NextPoint - NowPoint;        // ƫ��
	PrevError2_L = Error2_L;               //
	SumError2_L += Error2_L;                    // ����
	dError2_L = -LastError2_L + PrevError2_L;     // ��ǰ΢��
	LastError2_L = PrevError2_L;
	if (SumError2_L>1560) SumError2_L = 0;
	if (SumError2_L<-1560) SumError2_L = 0;
	pwm = kp2_L * Error2_L + ki2_L *  SumError2_L + kd2_L * dError2_L;
	if (NowPoint<(NextPoint + 5) && NowPoint>(NextPoint - 5))
		pwm = 0;
	return pwm;
}


/**************************************************************************
�������ܣ��ٶȻ�   ץ�򲿷�PID
��ڲ�������ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
����  ֵ����
**************************************************************************/
float LastPoint1_S, PrevError1_S,LastError1_S = 0;   //������Ҫ����
float Error1_S, SumError1_S, dError1_S = 0;
float kp1_S =3 , ki1_S = 0, kd1_S = 0.5;
float pwm1_S = 0;
float speed1(int NowPoint,int SetSpeed)//NowpointΪ��ǰλ��  SetSpeedΪ�����ٶ�
{     
	float pwm;
	float NowSpeed = 0;
	NowSpeed = NowPoint - LastPoint1_S;
	LastPoint1_S= NowPoint;
	Error1_S = (SetSpeed - NowSpeed)*1.0;        // ƫ��
	PrevError1_S = Error1_S;

	SumError1_S += Error1_S;                    // ����
	dError1_S = -LastError1_S + PrevError1_S;     // ��ǰ΢��
	LastError1_S = PrevError1_S;
	if (SumError1_S>1000) SumError1_S = 0;
	if (SumError1_S<1000) SumError1_S = 0;
	pwm = kp1_S * Error1_S + ki1_S *  SumError1_S + kd1_S * dError1_S+pwm1_S;
	pwm1_S = pwm;

	return pwm;
}



/**************************************************************************
�������ܣ��ٶȻ� ��ܴ󽮵��
��ڲ�����
����  ֵ����
**************************************************************************/
float LastPoint2_S, PrevError2_S, LastError2_S = 0;   
float Error2_S, SumError2_S, dError2_S = 0;
float kp2_S = 3, ki2_S = 0, kd2_S = 2;
float pwm2_S = 0;
float speed2(int NowPoint, int SetSpeed)
{    
	float pwm;
	float NowSpeed = 0;
	NowSpeed = NowPoint - LastPoint2_S;
	LastPoint2_S = NowPoint;
	Error2_S = (SetSpeed - NowSpeed)*2.0;        // ƫ��
	PrevError2_S = Error2_S;

	SumError2_S += Error2_S;                    // ����
	dError2_S = -LastError2_S + PrevError2_S;     // ��ǰ΢��
	LastError2_S = PrevError2_S;
	if (SumError2_S>2000) SumError2_S = 0;
	if (SumError2_S<2000) SumError2_S = 0;
	pwm = kp2_S * Error2_S + ki2_S *  SumError2_S + kd2_S * dError2_S + pwm2_S;
	pwm2_S = pwm;

	return pwm;
}

void Set_Pwm1(int moto1)
{
    if(moto1>0)			AIN2=1,			AIN1=0;
    else 	          AIN2=0,			AIN1=1;
    PWMA=myabs(moto1);
}

void Set_Pwm2(int moto2)
{
    if(moto2>0)	BIN1=1,			BIN2=0;
    else        BIN1=0,			BIN2=1;
    PWMB=myabs(moto2); 
}


void Xianfu_Pwm(void)
{
    int Amplitude=4150;    //
    if(Moto1<-Amplitude) Moto1=-Amplitude;
    if(Moto1>Amplitude)  Moto1=Amplitude;
    if(Moto2<-2000) Moto2=-2000;
    if(Moto2>2000)  Moto2=2000;

}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����int
����  ֵ��unsigned int
**************************************************************************/
int myabs(int a)
{
    int temp;
    if(a<0)  temp=-a;
    else temp=a;
    return temp;
}




