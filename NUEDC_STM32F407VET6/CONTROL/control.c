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

//有用到的变量 上面是没用到的变量
int Moto1,Moto2;
int Now1_Encoder=0,Last1_Encoder=0;//用于速度环时记录前后编码器读数
extern u8 Speed2Flag, Location2Flag;
extern int Location1Num, Location2Num, Speed1Num;
extern int Encoder1, Encoder2;
	
void Delay(__IO uint32_t nCount)
{
	for (; nCount != 0; nCount--);
}


/**************************************************************************
函数功能：中断控制RM35电机
入口参数：
返回  值：无
**************************************************************************/
void TIM6_DAC_IRQHandler(void)//10ms中断
{
	int pwmb = 0;
	
	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
	    if(Location2Flag)
			{
				Encoder2=TIM2->CNT;
			  pwmb = location2(Encoder2,Location2Num);
	      Moto2 = pwmb;
	      Xianfu_Pwm();    //PWM输出限幅 pwm1限幅4000，pwm2限幅3000
	      Set_Pwm2(-Moto2);//控制电机正反转
			}
			if(Location2Flag==0)
			{
				Set_Pwm1(0);
				Set_Pwm2(0);
			}
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update);  //清除中断标志位
  }
}



/**************************************************************************
函数功能：位置环 爪子部分大疆电机
入口参数：
返回  值：无
**************************************************************************/
float LastError1_L, PrevError1_L = 0;   //积分需要清零
float Error1_L, SumError1_L, dError1_L = 0;
float kp1_L = 1.5, ki1_L = 1, kd1_L = 4;

float location1( int NowPoint,int NextPoint)//Nowpoint当前位置 NextPoint为设定值
{    
	float pwm;
	Error1_L = NextPoint - NowPoint;        // 偏差
	PrevError1_L = Error1_L;
	SumError1_L += Error1_L;                    // 积分
	dError1_L= -LastError1_L + PrevError1_L;     // 当前微分
	LastError1_L = PrevError1_L;
	if (SumError1_L>1560) SumError1_L = 0;
	if (SumError1_L<-1560) SumError1_L = 0;
	pwm = kp1_L * Error1_L + ki1_L *  SumError1_L + kd1_L * dError1_L;	
	if(NowPoint<(NextPoint+5)&&NowPoint>(NextPoint-5))
		pwm=0;
	return pwm;
}



/**************************************************************************
函数功能：位置环 旋转部分大疆电机
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
float LastError2_L, PrevError2_L = 0;  
float Error2_L, SumError2_L, dError2_L = 0;
float kp2_L = 2, ki2_L = 0, kd2_L = 0.5;

float location2(int NowPoint, int NextPoint)//Nowpoint为当前位置 NextPoint为设置位置
{    
	float pwm;
	Error2_L = NextPoint - NowPoint;        // 偏差
	PrevError2_L = Error2_L;               //
	SumError2_L += Error2_L;                    // 积分
	dError2_L = -LastError2_L + PrevError2_L;     // 当前微分
	LastError2_L = PrevError2_L;
	if (SumError2_L>1560) SumError2_L = 0;
	if (SumError2_L<-1560) SumError2_L = 0;
	pwm = kp2_L * Error2_L + ki2_L *  SumError2_L + kd2_L * dError2_L;
	if (NowPoint<(NextPoint + 5) && NowPoint>(NextPoint - 5))
		pwm = 0;
	return pwm;
}


/**************************************************************************
函数功能：速度环   抓球部分PID
入口参数：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
返回  值：无
**************************************************************************/
float LastPoint1_S, PrevError1_S,LastError1_S = 0;   //积分需要清零
float Error1_S, SumError1_S, dError1_S = 0;
float kp1_S =3 , ki1_S = 0, kd1_S = 0.5;
float pwm1_S = 0;
float speed1(int NowPoint,int SetSpeed)//Nowpoint为当前位置  SetSpeed为设置速度
{     
	float pwm;
	float NowSpeed = 0;
	NowSpeed = NowPoint - LastPoint1_S;
	LastPoint1_S= NowPoint;
	Error1_S = (SetSpeed - NowSpeed)*1.0;        // 偏差
	PrevError1_S = Error1_S;

	SumError1_S += Error1_S;                    // 积分
	dError1_S = -LastError1_S + PrevError1_S;     // 当前微分
	LastError1_S = PrevError1_S;
	if (SumError1_S>1000) SumError1_S = 0;
	if (SumError1_S<1000) SumError1_S = 0;
	pwm = kp1_S * Error1_S + ki1_S *  SumError1_S + kd1_S * dError1_S+pwm1_S;
	pwm1_S = pwm;

	return pwm;
}



/**************************************************************************
函数功能：速度环 球架大疆电机
入口参数：
返回  值：无
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
	Error2_S = (SetSpeed - NowSpeed)*2.0;        // 偏差
	PrevError2_S = Error2_S;

	SumError2_S += Error2_S;                    // 积分
	dError2_S = -LastError2_S + PrevError2_S;     // 当前微分
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
函数功能：绝对值函数
入口参数：int
返回  值：unsigned int
**************************************************************************/
int myabs(int a)
{
    int temp;
    if(a<0)  temp=-a;
    else temp=a;
    return temp;
}




