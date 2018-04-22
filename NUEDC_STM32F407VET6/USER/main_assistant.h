#ifndef __MAIN_ASSISTANT_H
#define __MAIN_ASSISTANT_H

#include "sys.h"
#include "encoder.h"
#include "Beep.h"
#include "delay.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "inv_mpu.h"
#include "usart.h"
#include "motor.h"
#include "key.h"
#include "MPU6050.h"
#include "oled.h"
#include "encoder.h"
#include "exti.h"
#include "show.h"
#include "ANO_DT.h"
#include "led.h"
#include "HMC5883L.h"
#include "Task_Loop.h"
#include "SysTick.h"
#include "ak8975.h"
#include "myiic.h"
#include "Time.h"
#include "math.h"
#include "Stepper.h"
#include "control.h"
#include "epos_N.h"
#include "can.h"
#include "KS103.h"
#include <Sterring.h>
#include "usart3.h"
#include "Relay.h"
#include "auto_math.h"
#include "exti.h"
#include "RoboModule_can2.h"
#include "usart5.h"

//Elmo_ID
#define Elmo_ID 0xAA
#define UnderPan_ID 0xDD
//����ec40ָ��
#define Speed_Mode_Enable 0
#define Enable 1
#define Disable 2
#define Stop 3
#define Unstop 4
#define Reset 5
#define Erase_Reset_Flag 6
#define Speed_Position_Mode_Enable 7//���λ��ģʽ
#define Parameter_Enable 8//������ģʽ
#define Compensation 9//����

void Test_Pattern(void);
void OriginalPosition(void);
void Init_Movement(int num);//ץ��λ��
void Prepare_Movement(int num);//׼������λ��
void Launch_Movement(int num);//������ٶ� �����λ�� ����ֹͣλ��
void NextBallPosition(void);//�����תצ����ץ����λ��
void RM35_GoBack(void);

struct Ejection_parameter//��������ṹ��
{
    u16 Weight_of_Fire_Place;//����λ�õķ����ⲿ�������ģ�
    u16 Fire_Place;//�����λ��
    u16 Roll_Wait_Time;//˦���ֹͣʱ��
};

typedef struct Ejection_parameter Eje_value;

//����������
struct Elmo_parameter
{
    u8 capture;
    u8 prepare;
    u8 shoot;
    u8 shoot_swing;
};

typedef struct Elmo_parameter Para_value;

#endif