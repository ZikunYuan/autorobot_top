#ifndef _STEPPER_H
#define _STEPPER_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/6/16
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
#define Dir2 PEout(6)	// DS0
#define Dir1 PEout(5)	// DS1	
void Stepper1_TurnDir_StepNum( int step1);
void Stepper2_TurnDir_StepNum(int step2);
void Stepper1_PWM_Init(u32 arr,u32 psc);
void Stepper2_PWM_Init(u32 arr, u32 psc);
void Stepper_Init(void);
#endif

