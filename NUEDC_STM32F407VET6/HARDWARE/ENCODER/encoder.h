#ifndef __ENCODER_H
#define __ENCODER_H
#include <sys.h>	 

#define ENCODER_TIM_PERIOD (u32)(0xFFFFFFFF)   //�Զ���װ��ֵ ����TIM2��TIM5��32λ��
#define ENCODER1_TIMER 5                       //������1�õĶ�ʱ��5 32λ
#define ENCODER2_TIMER 2                       //������2�õĶ�ʱ��2 32λ
#define ENCODER3_TIMER 1                       //������4�õĶ�ʱ��1 16λ
void Encoder1_Init_TIM5(void);
void Encoder2_Init_TIM2(void);
void Encoder3_Init_TIM1(void);
int Read_Encoder(u8 TIMX);
void TIM5_IRQHandler(void);
void TIM2_IRQHandler(void);
void TIM1_BRK_TIM9_IRQHandler(void);

extern int Encoder1;
extern int Encoder2;
extern int Encoder3;  

#endif
